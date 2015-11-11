#include <iostream>     //cout,cerr
#include <fstream>      //ofstream
#include <iomanip>      //setw,setprecision
#include <queue>        //priority_queue
#include <list>         //list
#include <rw/rw.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rw/kinematics/Kinematics.hpp>
#include <rw/pathplanning/PathAnalyzer.hpp>
#include <rw/pathplanning/PlannerUtil.hpp>

/**
 * @brief The assignmentParameters struct Assignment parameters
 */
struct assignmentParameters {
    static constexpr auto workcellPath="../Kr16WallWorkCell/Scene.wc.xml";
    static constexpr double extend = 0.1; //For qualitative test and LUA output
    static constexpr double maxRRTTimeSeconds = 320.0; //For each RRT
    static constexpr double minExtend = 0.05; //Statistics, start with this
    static constexpr double maxExtend = 6.5; //Statistics, end with this (or a little smaller)
    static constexpr double incrExtend = 0.1; //Increment by this, starting from minExtend.
    static constexpr size_t numTrials = 10; //Repeated trials
    static constexpr size_t hugeNumTrials = 1000;
    static constexpr auto outputFile="../output.lua";
    //static constexpr auto statsFile="../statistics.txt";
};

/**
 * @brief The progressBar functor.
 * Usage:
 *  cout << progressBar(0.2);   // 20 %
 *  cout << progressBar(0.356); // 35.6 %
 */
struct progressBar {
    std::string operator()(float progress,std::ostream &out = std::cout,
                           unsigned int barWidth=70) {
        out << std::setw(5) << std::setprecision(3)
            <<((progress)>=1?100:(progress*100.0)) <<"% [";
        for(unsigned int i=0, pos=barWidth*progress;i<barWidth;++i) {
            if(i<pos) out << "=";
            else if(i==pos) out << ">";
            else out << " ";
        }
        out << "] "
            << "\r" << std::flush;
        return "";
    }
} progressBar; //Global instance.

using namespace std;
using namespace rw::common;
using namespace rw::kinematics;
using namespace rw::loaders;
using namespace rw::models;
using namespace rw::pathplanning;
using namespace rw::proximity;
using namespace rw::trajectory;
using namespace rwlibs::pathplanners;
using namespace rwlibs::proximitystrategies;

/**
 * @brief checkCollisions Checks a configuration for collisions.
 * @param device Device pointer.
 * @param state State pointer.
 * @param detector Collision Detector.
 * @param q Joint configuration to check.
 * @return true if collision-free. false otherwise.
 */
bool checkCollisions(Device::Ptr device, const State &state, const CollisionDetector &detector, const Q &q) {
    State testState = state;
    CollisionDetector::QueryResult data;
    device->setQ(q,testState);
    bool colFrom = detector.inCollision(testState,&data);
    if (colFrom) {
        cerr << "Configuration in collision: " << q << endl;
        cerr << "Colliding frames: " << endl;
        FramePairSet fps = data.collidingFrames;
        for (auto &i:fps)
            cerr << i.first->getName() << " " << i.second->getName() << endl;
        return false;
    }
    return true;
}

/**
 * @brief getRRTPlanner Wraps the calls to get a new RRT-connect planner.
 * @param device Device pointer.
 * @param state State.
 * @param detector Collision Detector.
 * @param extend Parameter for RRT algorithm, also known as epsilon.
 * @return RRT planner pointer.
 */
QToQPlanner::Ptr getRRTPlanner(Device::Ptr device,
                               const State &state,
                               CollisionDetector &detector,
                               double extend=0.05,
                               RRTPlanner::PlannerType plannerType = RRTPlanner::RRTConnect ) {
    PlannerConstraint constraint = PlannerConstraint::make(&detector,device,state);
    //Sampler will generate random (uniformly distributed) joint configurations within the constraints.
    QSampler::Ptr sampler = QSampler::makeConstrained(QSampler::makeUniform(device),constraint.getQConstraintPtr());
    //Euclidean metric weighs all joints the same -> 180 degrees in one joint weighs the same as 180 in the next.
    QMetric::Ptr metric = MetricFactory::makeEuclidean<Q>();
    return RRTPlanner::makeQToQPlanner(constraint,sampler,metric,extend,plannerType);
}


/**
 * @brief one Does one RRT-Connect run for the assignment.
 * @param workcell Workcell pointer
 * @param device Device pointer
 * @param extend Extend parameter
 * @param maxTime Max RRT time for path planning in seconds
 * @param luaFilename Output lua file name and path. If empty, no output will be written.
 */
void one(WorkCell::Ptr workcell, Device::Ptr device,
         const double extend=0.1, const double maxTime=60.0,
         const string &luaFilename=string()) {
    const State defaultState = workcell->getDefaultState();
    CollisionDetector detector(workcell,ProximityStrategyFactory::makeDefaultCollisionStrategy());
    //The RRT planner will be initialized when the state is changed to q_pick (which is the starting state).
    QToQPlanner::Ptr planner;// = getRRTPlanner(device,defaultState,detector,extend);

    // q_pick = (-3.142,-0.827,-3.002,-3.143,0.099,-1.573) [rad]
    // q_place = (1.571,0.006,0.030,0.153,0.762,4.490) [rad]
    const Q q_pick(6,-3.142,-0.827,-3.002,-3.143,0.099,-1.573);
    const Q q_place(6,1.571,0.006,0.030,0.153,0.762,4.490);
    //No reason to check for collisions before we have the robot in its initial state:
//    if (!( checkCollisions(device, defaultState, detector, q_pick)&&
//           checkCollisions(device, defaultState, detector, q_place)
//                ))
//        throw runtime_error("Collision!");



    //Tool frame is needed to grab the bottle.
    Frame *gripperFrame = workcell->findFrame("Tool");
    if(gripperFrame==nullptr) {
        throw runtime_error("Gripper frame \"Tool\" was not found in workcell \""+workcell->getFilename()+"\".");
    }
    //Bottle frame is needed to grab the bottle
    Frame *bottleFrame = workcell->findFrame<MovableFrame>("Bottle");
    if(bottleFrame==nullptr) {
        throw runtime_error("Bottle frame \"Bottle\" was not found in workcell \""+workcell->getFilename()+"\".");
    }
    //Table frame is needed to place the bottle (table "grabs" the bottle).
    Frame *tableFrame = workcell->findFrame("Table");
    if(tableFrame==nullptr) {
        throw runtime_error("Table frame \"Table\" was not found in workcell \""+workcell->getFilename()+"\".");
    }
    //The state we are going to modify (move) starts with default state
    State state = defaultState;
    //Move to bottle:
    device->setQ(q_pick,state);
    //Pick up bottle:
    Kinematics::gripFrame(bottleFrame,gripperFrame,state);

    //Initialize RRT planner.
    planner=getRRTPlanner(device,state,detector,extend,RRTPlanner::RRTConnect);
    //Collision check for q_pick and q_place
    if (!( checkCollisions(device, state, detector, q_pick)&&
           checkCollisions(device, state, detector, q_place)
                ))
        throw runtime_error("Collision!");

    //Path will store path from q_pick to q_init
    QPath path;
    Timer t;
    t.resetAndResume();
    //Get path (this is what we want to time in the statistics)
    planner->query(q_pick,q_place,path,maxTime);
    t.pause();
    cout << "Path of length " << path.size() << " found in " << t.getTime() << " seconds." << endl;
    if (t.getTime() >= maxTime)
        cout << "Notice: max time of " << maxTime << " seconds reached." << endl;



    //Analyze path in Time, Cartesian space and Joint Space.
    PathAnalyzer panlz(device,state);
    PathAnalyzer::CartesianAnalysis canlz = panlz.analyzeCartesian(path,gripperFrame);
    PathAnalyzer::JointSpaceAnalysis jsanlz = panlz.analyzeJointSpace(path);
    PathAnalyzer::TimeAnalysis tanlz = panlz.analyzeTime(path);
    cout << "Cartesian distance traveled by gripper: " << canlz.length << endl;
    cout << "Joint space distance, entire robot: " << jsanlz.length << endl;
    cout << "Time to execute path when considering only velocity limits: " << tanlz.time1 << endl;


    //Place bottle on table:
    Kinematics::gripFrame(bottleFrame,tableFrame,state);
    //Move away from bottle just a little bit:
    QPath littlebit(path.rbegin()+1,path.rbegin()+1+5); //littlebit is path from table and 5 steps back.

    //Output complete lua script to paste into RobWorkStudio:
    if(!luaFilename.empty()) {
        const string outputFile(luaFilename);
        ofstream output(outputFile);
        if(!output.is_open())
            throw runtime_error("Output file \""+outputFile+"\" could not be opened.");
        output << "wc = rws.getRobWorkStudio():getWorkCell()\n" <<
                  "state = wc:getDefaultState()\n" <<
                  "device = wc:findDevice(\"KukaKr16\")\n" <<
                  "gripper = wc:findFrame(\"Tool\")\n" <<
                  "bottle = wc:findFrame(\"Bottle\")\n" <<
                  "table = wc:findFrame(\"Table\")\n" <<
                  "function setQ(q)\n" <<
                  "\tqq = rw.Q(#q,q[1],q[2],q[3],q[4],q[5],q[6])\n" <<
                  "\tdevice:setQ(qq,state)\n"<<
                  "\trws.getRobWorkStudio():setState(state)\n"<<
                  "\trw.sleep(0.05)\n"<<
                  "end\n";

        output << "setQ({";
        for(size_t i=0; i<q_pick.size()-1; ++i)
            output << q_pick[i] << ',';
        output << q_pick[q_pick.size()-1] << "})\n";
        output << "rw.gripFrame(bottle,gripper,state)\n";
        for (auto &q:path) {
            output << "setQ({";
            for(size_t i=0; i< q.size()-1; ++i)
                output << q[i] << ',';
            output << q[q.size()-1] << "})\n";
        }

        output << "rw.gripFrame(bottle,table,state)\n";
        for (auto &q:littlebit) {
            output << "setQ({";
            for(size_t i=0; i< q.size()-1; ++i)
                output << q[i] << ',';
            output << q[q.size()-1] << "})\n";
        }

        output.close();
    }
}

// Here comes the statistics stuff

/**
 * @brief The pathCharacteristics struct Numbers that characterize a path
 * One data point for statistics.
 */
struct pathCharacteristics {
    double gripperDistanceCartesian;
    double robotDistanceJointSpace;
    double pathExecutionTime;
    double rrtExecutionTime;
    size_t number_of_states; //In practice, this is almost directly proportional to rrtExecutionTime.
    pathCharacteristics(double gripDist, double robDist, double ptime, double rtime, size_t length)
        : gripperDistanceCartesian(gripDist),
          robotDistanceJointSpace(robDist),
          pathExecutionTime(ptime),
          rrtExecutionTime(rtime),
          number_of_states(length)
    { }
};

/**
 * @brief The pathStatistic struct Simple path statistics for independent variable extend.
 * Medians are included as they are less sensitive to outliers coming
 * from CPU scheduling etc.
 * Complete overkill.
 * Only extend, rrt_exec_time_median, gripper_distance_median and jointspace_distance_median
 * are actually used in the Robotics Mandatory Exercise 2.
 */
struct pathStatistic {
    double extend;                      //
    size_t number_of_states_min;
    size_t number_of_states_max;
    double number_of_states_average;
    double number_of_states_median;
    double number_of_states_stddev;
    double rrt_exec_time_min;
    double rrt_exec_time_max;
    double rrt_exec_time_average;
    double rrt_exec_time_median;        //
    double rrt_exec_time_stddev;
    double gripper_distance_min;
    double gripper_distance_max;
    double gripper_distance_average;
    double gripper_distance_median;     //
    double gripper_distance_stddev;
    double jointspace_distance_min;
    double jointspace_distance_max;
    double jointspace_distance_average;
    double jointspace_distance_median;  //
    double jointspace_distance_stddev;
    double path_time_min;
    double path_time_max;
    double path_time_average;
    double path_time_median;
    double path_time_stddev;
    pathStatistic()
        : extend(0),
          number_of_states_min(numeric_limits<size_t>::max()),
          number_of_states_max(0),
          number_of_states_average(0),
          number_of_states_median(0),
          number_of_states_stddev(0),
          rrt_exec_time_min(numeric_limits<double>::max()),
          rrt_exec_time_max(0),
          rrt_exec_time_average(0),
          rrt_exec_time_median(0),
          rrt_exec_time_stddev(0),
          gripper_distance_min(numeric_limits<double>::max()),
          gripper_distance_max(0),
          gripper_distance_average(0),
          gripper_distance_median(0),
          gripper_distance_stddev(0),
          jointspace_distance_min(numeric_limits<double>::max()),
          jointspace_distance_max(0),
          jointspace_distance_average(0),
          jointspace_distance_median(0),
          jointspace_distance_stddev(0),
          path_time_min(numeric_limits<double>::max()),
          path_time_max(0),
          path_time_average(0),
          path_time_median(0),
          path_time_stddev(0)
    { }
};

/**
 * @brief analyzePaths Calculates path statistics based on (extend,path) data.
 * @param statmap Data container coupling extend values (keys) with data.
 * @return List of statistics. One element per unique extend value.
 */
list<pathStatistic> analyzePaths(const multimap<double,pathCharacteristics> &statmap) {
    list<pathStatistic> statistics;
    vector<double> extends;
    { //get all the different extend values in O(N)
        vector<pair<double,pathCharacteristics> > dedup;
        unique_copy(statmap.begin(),statmap.end(),back_inserter(dedup),
                    [](const pair<double,pathCharacteristics> &e1,
                    const pair<double,pathCharacteristics> &e2) { return e1.first==e2.first;});
        for(auto &e:dedup)
            extends.push_back(e.first);
    }
    for(auto extend:extends) {
        auto mapbounds = statmap.equal_range(extend);
        pathStatistic ps;
        size_t numTrials=0;
        //store extend in statistic
        ps.extend=extend;
        //Averages, minima and maxima:
        for_each(mapbounds.first,mapbounds.second,
                 [&ps,&numTrials](const pair<double,pathCharacteristics> &elem) {
            ++numTrials;
            ps.number_of_states_average+=elem.second.number_of_states;
            ps.rrt_exec_time_average+=elem.second.rrtExecutionTime;
            ps.gripper_distance_average+=elem.second.gripperDistanceCartesian;
            ps.jointspace_distance_average+=elem.second.robotDistanceJointSpace;
            ps.path_time_average+=elem.second.pathExecutionTime;
            if(elem.second.number_of_states<ps.number_of_states_min)
                ps.number_of_states_min=elem.second.number_of_states;
            if(elem.second.rrtExecutionTime<ps.rrt_exec_time_min)
                ps.rrt_exec_time_min=elem.second.rrtExecutionTime;
            if(elem.second.gripperDistanceCartesian<ps.gripper_distance_min)
                ps.gripper_distance_min=elem.second.gripperDistanceCartesian;
            if(elem.second.robotDistanceJointSpace<ps.jointspace_distance_min)
                ps.jointspace_distance_min=elem.second.robotDistanceJointSpace;
            if(elem.second.pathExecutionTime<ps.path_time_min)
                ps.path_time_min=elem.second.pathExecutionTime;
            if(elem.second.number_of_states>ps.number_of_states_max)
                ps.number_of_states_max=elem.second.number_of_states;
            if(elem.second.rrtExecutionTime>ps.rrt_exec_time_max)
                ps.rrt_exec_time_max=elem.second.rrtExecutionTime;
            if(elem.second.gripperDistanceCartesian>ps.gripper_distance_max)
                ps.gripper_distance_max=elem.second.gripperDistanceCartesian;
            if(elem.second.robotDistanceJointSpace>ps.jointspace_distance_max)
                ps.jointspace_distance_max=elem.second.robotDistanceJointSpace;
            if(elem.second.pathExecutionTime>ps.path_time_max)
                ps.path_time_max=elem.second.pathExecutionTime;
        });
        ps.number_of_states_average/=numTrials;
        ps.rrt_exec_time_average/=numTrials;
        ps.gripper_distance_average/=numTrials;
        ps.jointspace_distance_average/=numTrials;
        ps.path_time_average/=numTrials;

        //Median calculation using streaming median algorithm with two heaps (per measured variable)
        priority_queue<size_t> nsL; //max-heap
        priority_queue<size_t,priority_queue<size_t>::container_type,greater<size_t> > nsH; //min-heap
        priority_queue<double> rtL;
        priority_queue<double,priority_queue<double>::container_type,greater<double> > rtH;
        priority_queue<double> gdL;
        priority_queue<double,priority_queue<double>::container_type,greater<double> > gdH;
        priority_queue<double> jdL;
        priority_queue<double,priority_queue<double>::container_type,greater<double> > jdH;
        priority_queue<double> ptL;
        priority_queue<double,priority_queue<double>::container_type,greater<double> > ptH;
        //Std. deviation (variance first) and median:
        for_each(mapbounds.first,mapbounds.second,
                 [&ps,&nsL,&nsH,&rtL,&rtH,&gdL,&gdH,&jdL,&jdH,&ptL,&ptH](const pair<double,pathCharacteristics> &elem) {
            //Variance summation
            ps.number_of_states_stddev+=std::pow(elem.second.number_of_states-ps.number_of_states_average,2);
            ps.rrt_exec_time_stddev+=std::pow(elem.second.rrtExecutionTime-ps.rrt_exec_time_average,2);
            ps.gripper_distance_stddev+=std::pow(elem.second.gripperDistanceCartesian-ps.gripper_distance_average,2);
            ps.jointspace_distance_stddev+=std::pow(elem.second.robotDistanceJointSpace-ps.jointspace_distance_average,2);
            ps.path_time_stddev+=std::pow(elem.second.pathExecutionTime-ps.path_time_average,2);
            //Streaming median algorithm; Maintain heaps
            //Number of states
            if(nsL.empty()) nsL.push(elem.second.number_of_states);
            else if(elem.second.number_of_states<=nsL.top()) nsL.push(elem.second.number_of_states);
            else nsH.push(elem.second.number_of_states);
            if(nsL.size()>nsH.size()) {
                nsH.push(nsL.top());
                nsL.pop();
            }
            else if(nsL.size()<nsH.size()) {
                nsL.push(nsH.top());
                nsH.pop();
            }
            //RRT algorithm time
            if(rtL.empty()) rtL.push(elem.second.rrtExecutionTime);
            else if(elem.second.rrtExecutionTime<=rtL.top()) rtL.push(elem.second.rrtExecutionTime);
            else rtH.push(elem.second.rrtExecutionTime);
            if(rtL.size()>rtH.size()) {
                rtH.push(rtL.top());
                rtL.pop();
            }
            else if(rtL.size()<rtH.size()) {
                rtL.push(rtH.top());
                rtH.pop();
            }
            //Gripper cartesian distance
            if(gdL.empty()) gdL.push(elem.second.gripperDistanceCartesian);
            else if(elem.second.gripperDistanceCartesian<=gdL.top()) gdL.push(elem.second.gripperDistanceCartesian);
            else gdH.push(elem.second.gripperDistanceCartesian);
            if(gdL.size()>gdH.size()) {
                gdH.push(gdL.top());
                gdL.pop();
            }
            else if(gdL.size()<gdH.size()) {
                gdL.push(gdH.top());
                gdH.pop();
            }
            //Joint space distance
            if(jdL.empty()) jdL.push(elem.second.robotDistanceJointSpace);
            else if(elem.second.robotDistanceJointSpace<=jdL.top()) jdL.push(elem.second.robotDistanceJointSpace);
            else jdH.push(elem.second.robotDistanceJointSpace);
            if(jdL.size()>jdH.size()) {
                jdH.push(jdL.top());
                jdL.pop();
            }
            else if(jdL.size()<jdH.size()) {
                jdL.push(jdH.top());
                jdH.pop();
            }
            //Path execution time
            if(ptL.empty()) ptL.push(elem.second.pathExecutionTime);
            else if(elem.second.pathExecutionTime<=ptL.top()) ptL.push(elem.second.pathExecutionTime);
            else ptH.push(elem.second.pathExecutionTime);
            if(ptL.size()>ptH.size()) {
                ptH.push(ptL.top());
                ptL.pop();
            }
            else if(ptL.size()<ptH.size()) {
                ptL.push(ptH.top());
                ptH.pop();
            }
        });

        //Variance calculation:
        if(numTrials>1) {
            ps.number_of_states_stddev/=(numTrials-1);
            ps.rrt_exec_time_stddev/=(numTrials-1);
            ps.gripper_distance_stddev/=(numTrials-1);
            ps.jointspace_distance_stddev/=(numTrials-1);
            ps.path_time_stddev/=(numTrials-1);
        }
        //Convert to std. deviations:
        ps.number_of_states_stddev=std::sqrt(ps.number_of_states_stddev);
        ps.rrt_exec_time_stddev=std::sqrt(ps.rrt_exec_time_stddev);
        ps.gripper_distance_stddev=std::sqrt(ps.gripper_distance_stddev);
        ps.jointspace_distance_stddev=std::sqrt(ps.jointspace_distance_stddev);
        ps.path_time_stddev=std::sqrt(ps.path_time_stddev);
        //Get medians from heaps:
        ps.number_of_states_median=(nsL.size()<nsH.size())?(nsH.top()):((nsL.size()>nsH.size())?(nsL.top()):((nsL.top()+nsH.top())/2.0));
        ps.rrt_exec_time_median=(rtL.size()<rtH.size())?(rtH.top()):((rtL.size()>rtH.size())?(rtL.top()):((rtL.top()+rtH.top())/2.0));
        ps.gripper_distance_median=(gdL.size()<gdH.size())?(gdH.top()):((gdL.size()>gdH.size())?(gdL.top()):((gdL.top()+gdH.top())/2.0));
        ps.jointspace_distance_median=(jdL.size()<jdH.size())?(jdH.top()):((jdL.size()>jdH.size())?(jdL.top()):((jdL.top()+jdH.top())/2.0));
        ps.path_time_median=(ptL.size()<ptH.size())?(ptH.top()):((ptL.size()>ptH.size())?(ptL.top()):((ptL.top()+ptH.top())/2.0));

        //Push the statistics calculated for this extend value to the list
        statistics.push_back(ps);
    }
    return move(statistics);
}

/**
 * @brief stats The equivalent of calling one() a lot of times with statistics.
 * @param workcell Workcell pointers.
 * @param device Device pointer.
 * @param maxTimePerRRT Maximum allowed time per RRT execution.
 * @param minExtend Minimum value of extend parameter.
 * @param maxExtend Maximum value of extend parameter.
 * @param increaseStep Extend parameter step size.
 * @param numTrials Number of trials for each unique extend value.
 * @param statsFilename Name of output file (with path)
 *
 * Runs the RRT algorithm a lot of times, and stores data for each path found.
 * Afterwards, passes the collected data to a call to analyzePaths()
 * and stores everything (statistics and data) in a file.
 */
double stats(
        WorkCell::Ptr workcell,
        Device::Ptr device,
        RRTPlanner::PlannerType plannerType=RRTPlanner::RRTConnect,
        double maxTimePerRRT=240.0,
        double minExtend=0.05,
        double maxExtend=0.5,
        double increaseStep = 0.01,
        size_t numTrials=10,
        const string statsFilename=string()
        ) {
    const State defaultState = workcell->getDefaultState();
    CollisionDetector detector(workcell,ProximityStrategyFactory::makeDefaultCollisionStrategy());
    QToQPlanner::Ptr planner;
    const Q q_pick(6,-3.142,-0.827,-3.002,-3.143,0.099,-1.573);
    const Q q_place(6,1.571,0.006,0.030,0.153,0.762,4.490);
    Frame *gripperFrame = workcell->findFrame("Tool");
    if(gripperFrame==nullptr)
        throw runtime_error("Gripper frame \"Tool\" was not found in workcell \""+workcell->getFilename()+"\".");
    Frame *bottleFrame = workcell->findFrame<MovableFrame>("Bottle");
    if(bottleFrame==nullptr)
        throw runtime_error("Bottle frame \"Bottle\" was not found in workcell \""+workcell->getFilename()+"\".");
    Frame *tableFrame = workcell->findFrame("Table");
    if(tableFrame==nullptr)
        throw runtime_error("Table frame \"Table\" was not found in workcell \""+workcell->getFilename()+"\".");
    State state = defaultState;


    PathAnalyzer panlz(device,state);

    //Move to bottle:
    device->setQ(q_pick,state);
    //Pick up bottle:
    Kinematics::gripFrame(bottleFrame,gripperFrame,state);

    multimap<double,pathCharacteristics> statmap;

    //Collect data:

    const float done=static_cast<float>(numTrials);
    float progress=0.0;
    const double lastExtend = (maxExtend-increaseStep);
    cout << "Collecting RRT data with extend ranging from " << minExtend << " to " << lastExtend << ".\n";
    cout << "Output filename: \"" << statsFilename << "\"." << endl;
    cout << progressBar(progress);
    for(size_t i=0; i<numTrials; ++i) {
        progress = (static_cast<float>(i))/done;
        for(double extend=minExtend, dif=(lastExtend-minExtend)*done; extend<maxExtend; extend+=increaseStep) {
            device->setQ(q_pick,state);
            //Initialize RRT planner.
            planner=getRRTPlanner(device,state,detector,extend,plannerType);
            //Collision check for q_pick and q_place
            if (!( checkCollisions(device, state, detector, q_pick)&&
                   checkCollisions(device, state, detector, q_place)
                   ))
                throw runtime_error("Collision!");

            //Path will store path from q_pick to q_init
            QPath path;
            Timer t;
            t.resetAndResume();
            //Get path (this is what we want to time)
            planner->query(q_pick,q_place,path,maxTimePerRRT);
            t.pause();

            PathAnalyzer::CartesianAnalysis canlz = panlz.analyzeCartesian(path,gripperFrame);
            PathAnalyzer::JointSpaceAnalysis jsanlz = panlz.analyzeJointSpace(path);
            PathAnalyzer::TimeAnalysis tanlz = panlz.analyzeTime(path);
            pathCharacteristics mypc(canlz.length,jsanlz.length,tanlz.time1,t.getTimeMs(),path.size());
            //statmap.insert(statmap.end(),make_pair(extend,make_pair(t.getTimeMs(),path.size())));
            statmap.insert(statmap.end(),make_pair(extend,mypc));
            float rprog = progress + (extend/dif);
            cout << progressBar(rprog);
        }
    }
    cout << progressBar(1) << endl;


    //Calculate statistics using collected data:
    list<pathStatistic> statistics=analyzePaths(statmap);

    //Output everything to file:
    ofstream output(statsFilename);
    if(!output.is_open())
        throw runtime_error("Output file \""+statsFilename+"\" could not be opened.");
//    //Output statistics
//    output << "#Statistics:\n#Max time per RRT [s]: " << maxTimePerRRT << '\n'
//           << "#extend,number_of_states_min,number_of_states_max,number_of_states_average,"
//              "number_of_states_median,number_of_states_stddev,rrt_exec_time_min,"
//              "rrt_exec_time_max,rrt_exec_time_average,rrt_exec_time_median,"
//              "rrt_exec_time_stddev,gripper_distance_min,gripper_distance_max,"
//              "gripper_distance_average,gripper_distance_median,gripper_distance_stddev,"
//              "jointspace_distance_min,jointspace_distance_max,jointspace_distance_average,"
//              "jointspace_distance_median,jointspace_distance_stddev,path_time_min,"
//              "path_time_max,path_time_average,path_time_median,path_time_stddev\n";
//    for(auto &s:statistics) {
//        output << setprecision(14) <<
//                  s.extend << ',' <<
//                  s.number_of_states_min << ',' <<
//                  s.number_of_states_max << ',' <<
//                  s.number_of_states_average << ',' <<
//                  s.number_of_states_median << ',' <<
//                  s.number_of_states_stddev << ',' <<
//                  s.rrt_exec_time_min << ',' <<
//                  s.rrt_exec_time_max << ',' <<
//                  s.rrt_exec_time_average << ',' <<
//                  s.rrt_exec_time_median << ',' <<
//                  s.rrt_exec_time_stddev << ',' <<
//                  s.gripper_distance_min << ',' <<
//                  s.gripper_distance_max << ',' <<
//                  s.gripper_distance_average << ',' <<
//                  s.gripper_distance_median << ',' <<
//                  s.gripper_distance_stddev << ',' <<
//                  s.jointspace_distance_min << ',' <<
//                  s.jointspace_distance_max << ',' <<
//                  s.jointspace_distance_average << ',' <<
//                  s.jointspace_distance_median << ',' <<
//                  s.jointspace_distance_stddev << ',' <<
//                  s.path_time_min << ',' <<
//                  s.path_time_max << ',' <<
//                  s.path_time_average << ',' <<
//                  s.path_time_median << ',' <<
//                  s.path_time_stddev << '\n';
//    }


    //Output data used for statistics
    output << "extend,number_of_states,RRT_execution_time,"
              "gripper_distance_cart,robot_distance_jointspace,path_exec_time\n";
    for(auto &d:statmap)
        output << d.first << ',' <<
                  d.second.number_of_states << ',' <<
                  d.second.rrtExecutionTime << ',' <<
                  d.second.gripperDistanceCartesian << ',' <<
                  d.second.robotDistanceJointSpace << ',' <<
                  d.second.pathExecutionTime << '\n';

    output.close();


    double theShortestDistance = 1000000.0;
    double theOptimalExtend = 0.0;
    for(auto &s:statistics) {
        if((s.jointspace_distance_median)<theShortestDistance) {
            theShortestDistance=(s.jointspace_distance_median);
            theOptimalExtend = s.extend;
        }
    }


    return theOptimalExtend;
}

/**
 * @brief main Loads workcell and calls one() and stats()
 * @return 0 if successful.
 *
 * Calls one() once, generating the output LUA script.
 * Then calls stats() once, generating the output statistics text file.
 * @see assignmentParameters.
 */
int main(int argc, char **argv)
{
    cout << "Program started.\n" << endl;
    if ( !(argc >= 2) ) {
        cerr << "Input problem. Just pass the number of trials as argument." << endl;
        return -1;
    }
    string filename(argv[1]);

    stringstream ss;
    ss << filename;
    size_t num_trials;
    ss >> num_trials;



    cout << "Program started. Number of trials: " << num_trials << endl;
    Math::seed();

    const string workcellPath(assignmentParameters::workcellPath);
    const string deviceName("KukaKr16");

    WorkCell::Ptr workcell;
    Device::Ptr device;
    try {
        workcell = WorkCellLoader::Factory::load(workcellPath);
    } catch(exception& e) {
        cout << "Workcell could not be loaded from \"" << workcellPath << "\"." << endl;
        throw e;
    }
    device = workcell->findDevice(deviceName);
    if(device==nullptr) {
        throw runtime_error("Device \""+deviceName+"\" not found in workcell \""+workcellPath+"\".");
    }

    one(workcell,
        device,
        assignmentParameters::extend,
        assignmentParameters::maxRRTTimeSeconds,
        assignmentParameters::outputFile);

    double rrtconExtend = stats(workcell,
          device,
          RRTPlanner::RRTConnect,
          assignmentParameters::maxRRTTimeSeconds,
          assignmentParameters::minExtend,
          assignmentParameters::maxExtend,
          assignmentParameters::incrExtend,
          num_trials,
          "../rrtconnect.txt");
    double rrtbiExtend = stats(workcell,
          device,
          RRTPlanner::RRTBidirectional,
          assignmentParameters::maxRRTTimeSeconds,
          assignmentParameters::minExtend,
          assignmentParameters::maxExtend,
          assignmentParameters::incrExtend,
          num_trials,
          "../rrtbidirectional.txt");
    double rrtbalExtend = stats(workcell,
          device,
          RRTPlanner::RRTBalancedBidirectional,
          assignmentParameters::maxRRTTimeSeconds,
          assignmentParameters::minExtend,
          assignmentParameters::maxExtend,
          assignmentParameters::incrExtend,
          num_trials,
          "../rrtbalancedbidirectional.txt");



    //for the optimal extends, MORE STATISTICS!
    stats(workcell,
          device,
          RRTPlanner::RRTConnect,
          assignmentParameters::maxRRTTimeSeconds,
          rrtconExtend,rrtconExtend+assignmentParameters::incrExtend/2.0,
          assignmentParameters::incrExtend,
          assignmentParameters::hugeNumTrials,
          "../rrtconnect_optimal_extend.txt"
          );
    stats(workcell,
          device,
          RRTPlanner::RRTBidirectional,
          assignmentParameters::maxRRTTimeSeconds,
          rrtbiExtend,rrtbiExtend+assignmentParameters::incrExtend/2.0,
          assignmentParameters::incrExtend,
          assignmentParameters::hugeNumTrials,
          "../rrtbidirectional_optimal_extend.txt"
          );
    stats(workcell,
          device,
          RRTPlanner::RRTBalancedBidirectional,
          assignmentParameters::maxRRTTimeSeconds,
          rrtbalExtend,rrtbalExtend+assignmentParameters::incrExtend/2.0,
          assignmentParameters::incrExtend,
          assignmentParameters::hugeNumTrials,
          "../rrtbalancedbidirectional_optimal_extend.txt"
          );


    cout << "Program ended." << endl;
    return 0;
}
