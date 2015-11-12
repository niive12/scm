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

# names       = c("RRT Bidirectional","RRT Balanced Bidirectional","RRT Connect")
# colors      = c("red","green","blue")
# pch         = array(15,3)

names       = c("RRT Bidirectional","RRT Balanced Bidirectional")
colors      = c("red","blue")
pch         = array(15,2)

figwidth = 10
figheight = 5

setEPS()
postscript("../document/graphics/bidirectional_correlation.eps",width=figwidth,height=figheight)
bidirect    = see_correlation(rrtbidirect$extend, rrtbidirect$robot_distance_jointspace, eps_tested, number_of_tests, method = names[1] )
postscript("../document/graphics/balanced_correlation.eps",width=figwidth,height=figheight)
balanced    = see_correlation(rrtbidirect$extend, rrtbalanced$robot_distance_jointspace, eps_tested, number_of_tests, method = names[2] )
# connect     = see_correlation( rrtconnect$extend,  rrtconnect$robot_distance_jointspace, eps_tested, number_of_tests, method = names[3] )
            
bid_time    = find_median(rrtbidirect$RRT_execution_time,number_of_tests)$median
bal_time    = find_median(rrtbalanced$RRT_execution_time,number_of_tests)$median
# con_time    = find_median( rrtconnect$RRT_execution_time,number_of_tests)$median
cat("------------------- Comparative experiment -------------------\n")

postscript("../document/graphics/compare_time.eps",width=figwidth,height=figheight)
 plot(eps_tested, bid_time, col=colors[1], type="l", lwd=3,xlab=expression(paste(epsilon," [Jointspace units]")),ylab=expression(paste("Time [S]")))
lines(eps_tested, bal_time, col=colors[2], type="l", lwd=3)
# lines(eps_tested, con_time, col=colors[3], type="l", lwd=3)
legend("topright",legend=names,pch=pch,cex=0.8,col=colors)

postscript("../document/graphics/compare_distance.eps",width=figwidth,height=figheight)
 plot(eps_tested, bidirect$median, col=colors[1], type="l", lwd=3,xlab=expression(paste(epsilon," [Jointspace units]")),ylab=expression(paste(d[J]," [Jointspace units]")))
lines(eps_tested, balanced$median, col=colors[2], type="l", lwd=3)
# lines(eps_tested, connect$median,  col=colors[3], type="l", lwd=3)
legend("bottomright",legend=names,pch=pch,cex=0.8,col=colors)


bidirect_op       = read.csv("rrtbidirectional_optimal_extend.txt")
balanced_op       = read.csv("rrtbalancedbidirectional_optimal_extend.txt")

# is the data normally distributed?
n = length(bidirect_op$extend)
postscript("../document/graphics/hist_op_bi.eps",width=figwidth,height=figheight)
hist(bidirect_op$robot_distance_jointspace,xlab=expression(paste(d[J]," [Jointspace units]")),main=NULL)
postscript("../document/graphics/hist_op_ba.eps",width=figwidth,height=figheight)
hist(balanced_op$robot_distance_jointspace,xlab=expression(paste(d[J]," [Jointspace units]")),main=NULL)

postscript("../document/graphics/qq_op_bi.eps",width=figwidth,height=figheight)
qqplot(rnorm(10^3),bidirect_op$robot_distance_jointspace,ylab=expression(paste(d[J]," [Jointspace units]")))
postscript("../document/graphics/qq_op_ba.eps",width=figwidth,height=figheight)
qqplot(rnorm(10^3),balanced_op$robot_distance_jointspace,ylab=expression(paste(d[J]," [Jointspace units]")))

cat("The data is not normally distributed. We transform the data into log space.\n")

transformed_bi = log(bidirect_op$robot_distance_jointspace)
transformed_ba = log(balanced_op$robot_distance_jointspace)

postscript("../document/graphics/hist_tran_op_bi.eps",width=figwidth,height=figheight)
hist(transformed_bi,xlab=expression(paste(d[J]," [Jointspace units]")),main=NULL)
postscript("../document/graphics/hist_tran_op_ba.eps",width=figwidth,height=figheight)
hist(transformed_ba,xlab=expression(paste(d[J]," [Jointspace units]")),main=NULL)

postscript("../document/graphics/qq_tran_op_bi.eps",width=figwidth,height=figheight)
qqplot(rnorm(10^3),transformed_bi,ylab=expression(paste(d[J]," [Jointspace units]")))
postscript("../document/graphics/qq_tran_op_ba.eps",width=figwidth,height=figheight)
qqplot(rnorm(10^3),transformed_ba,ylab=expression(paste(d[J]," [Jointspace units]")))

cat("just to be certain: shapiro wilks test for normality\n")
cat(c("p-value bidirectional: ", shapiro.test(transformed_bi)$p.value, ", balanced: ", shapiro.test(transformed_ba)$p.value,"\n"))
cat("This is ignored... We have a lot of data points\n")

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

p_value = t.test(transformed_bi,transformed_ba,alternative="two.sided", var.equal=FALSE)$p.value
cat(c("P-value is: ",p_value,"\t"))

if (p_value > alpha) {
  cat("H0 accepted, The mean are significantly different.\n")
} else {
  cat("H0 rejected, They have an equal mean.\n")
}