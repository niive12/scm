source("explorative.R")

cat("------------------- Explorative experiment -------------------\n")

# extend,number_of_states,RRT_execution_time,gripper_distance_cart,robot_distance_jointspace,path_exec_time
rrtbidirect = read.csv("rrtbidirectional.txt")
rrtbalanced = read.csv("rrtbalancedbidirectional.txt")
# rrtconnect       = read.csv("rrtconnect.txt")

eps_tested = seq(0.05,6.4,0.1)
for( i in 2:length(rrtbidirect$extend) ){
	if(rrtbidirect$extend[i] > rrtbidirect$extend[i-1]){
		number_of_tests = i -1 
		break
	}
}

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

 plot(eps_tested, bidirect$median, col=colors[1], type="l", lwd=3,ylab="distance")
lines(eps_tested, balanced$median, col=colors[2], type="l", lwd=3)
# lines(eps_tested, connect$median,  col=colors[3], type="l", lwd=3)
legend("bottomright",legend=names,pch=pch,cex=0.8,col=colors)

cat("------------------- Comparative experiment -------------------\n")

bidirect_op       = read.csv("rrtbidirectional_optimal_extend.txt")
balanced_op       = read.csv("rrtbalancedbidirectional_optimal_extend.txt")

# is the data normally distributed?
n = length(bidirect_op$extend)
hist(bidirect_op$robot_distance_jointspace)
hist(balanced_op$robot_distance_jointspace)

qqplot(rnorm(10^3),bidirect_op$robot_distance_jointspace)
qqplot(rnorm(10^3),balanced_op$robot_distance_jointspace)

#conclusion, if you disregard enough of the outliers, sure.
alpha = 0.05

cat("Is the variances equal?\n")
p_var = 1 - var.test(bidirect_op$robot_distance_jointspace, balanced_op$robot_distance_jointspace)$p.value

cat(c("P-value is: ",p_var,"\t"))

if (p_var > alpha) {
  cat("H0 accepted, The variance are significantly different.\n")
} else {
  cat("H0 rejected, They have an equal variance.\n")
}


cat("Is the mean equal?\n")

SSE = var(bidirect_op$robot_distance_jointspace)/sqrt(n)
SSE_pooled = sqrt(var(bidirect_op$robot_distance_jointspace)^2/n + var(balanced_op$robot_distance_jointspace)^2/n )

mean_bid = mean(bidirect_op$robot_distance_jointspace)
mean_bal = mean(balanced_op$robot_distance_jointspace)

Z = (mean_bid-mean_bal) / SSE
p_value = 1 - pnorm(Z, mean=mean_bid, sd=var(bidirect_op$robot_distance_jointspace))
cat(c("P-value is: ",p_value,"\t"))

if (p_value > alpha) {
  cat("H0 accepted, The mean are significantly different.\n")
} else {
  cat("H0 rejected, They have an equal mean.\n")
}

