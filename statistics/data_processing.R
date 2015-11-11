source("explorative.R")

# extend,number_of_states,RRT_execution_time,gripper_distance_cart,robot_distance_jointspace,path_exec_time
eps_tested = seq(0.05,6.4,0.1)
rrtconnect       = read.csv("rrtconnect.txt")
for( i in 2:length(rrtconnect$extend) ){
	if(rrtconnect$extend[i] > rrtconnect$extend[i-1]){
		number_of_tests = i -1 
		break
	}
}

rrtbidirect = read.csv("rrtbidirectional.txt")
rrtbalanced = read.csv("rrtbalancedbidirectional.txt")

# names       = c("RRT Bidirectional","RRT Connected Bidirectional","RRT Connect")
# colors      = c("red","green","blue")
# pch         = array(15,3)

names       = c("RRT Bidirectional","RRT Connected Bidirectional")
colors      = c("red","green")
pch         = array(15,2)
            
bidirect    = see_correlation(rrtbidirect$extend, rrtbidirect$robot_distance_jointspace, eps_tested, number_of_tests, method = names[1] )
balanced    = see_correlation(rrtbidirect$extend, rrtbalanced$robot_distance_jointspace, eps_tested, number_of_tests, method = names[2] )
# connect     = see_correlation( rrtconnect$extend,  rrtconnect$robot_distance_jointspace, eps_tested, number_of_tests, method = names[3] )
            
bid_time    = find_median(rrtbidirect$RRT_execution_time,number_of_tests)$median
bal_time    = find_median(rrtbalanced$RRT_execution_time,number_of_tests)$median
# con_time    = find_median( rrtconnect$RRT_execution_time,number_of_tests)$median

 plot(eps_tested, bid_time, col=colors[1], type="l", lwd=3,ylab="time")
lines(eps_tested, bal_time, col=colors[2], type="l", lwd=3)
# lines(eps_tested, con_time, col=colors[3], type="l", lwd=3)
legend("topright",legend=names,pch=pch,cex=0.8,col=colors)

 plot(eps_tested, bidirect$median, col=colors[1], type="l", lwd=3)
lines(eps_tested, balanced$median, col=colors[2], type="l", lwd=3)
# lines(eps_tested, connect$median,  col=colors[3], type="l", lwd=3)
legend("bottomright",legend=names,pch=pch,cex=0.8,col=colors)

