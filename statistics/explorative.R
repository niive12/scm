source("functions.R")

see_correlation <- function(eps, dist, eps_tested, number_of_tests, method){
	median = find_median(dist,number_of_tests)$median

	plot(eps, dist,col="blue",main=NULL,xlab=expression(paste(epsilon," [Jointspace units]")),ylab=expression(paste(d[J]," [Jointspace units]")))
	lines(eps_tested, median, col="red", type="l",lwd=3);

	cat(c("The extend parameter and the distance traveled in joint space for ",method," is "))
	if( cor.test(eps,dist, method="spearman")$p.value > 0.05 ){
		cat("not ")
	}
	cat("correlated. with a p-value of ",cor.test(eps,dist, method="spearman")$p.value, ".\n");
	optimal_extend = eps_tested[which(median == min(median))]

	cat(c("The optimal extend for ",method,":\t", optimal_extend, "\n"))
	return(list(optimal_extend=optimal_extend,median=median))
}