mean_bar_plot <- function(input, labels=1, xlab=NULL, ylab=NULL,main=NULL, error=TRUE){
	if(labels[1] == 1){
		labels = 1:dim(input)[2]
	}
	input.mean = apply(input,2,mean)
	input.var = apply(input,2,var)
	limits = c(0, max(input.mean))
	if(error){
		limits = c(0, max(input.mean+input.var))
	}
	mid_bar = barplot(input.mean, names.arg=labels,ylim=limits, col="gray", axis.lty=1, main=main, xlab=xlab, ylab=ylab)
	if(error){
		arrows(mid_bar,input.mean+input.var, mid_bar, input.mean-input.var, angle=90, code=3, length=0.1)
	}
}

eps_tested = seq(0.05,6.5,0.1)
number_of_tets = 30
time =   matrix(0,number_of_tets,length(eps_tested))
length = matrix(0,number_of_tets,length(eps_tested))
epsilon = matrix(0,number_of_tets,length(eps_tested))


rrtconnect       = read.csv("rrtconnect.txt")
setEPS()
postscript("timeVSepsilon.eps",height = 4, width = 8)
mean_bar_plot(time,xlab="epsilon", ylab="Average Time [ms]", labels=eps_tested,error=FALSE)
q = dev.off()

setEPS()
postscript("distVSepsilon.eps",height = 4, width = 8)
mean_bar_plot(length,xlab="epsilon", ylab="Average Path Length [m]", labels=eps_tested,error=TRUE)
q = dev.off()

rrtbidirectional = read.csv("rrtbidirectional.txt")
rrtbalanced      = read.csv("rrtbalanced.txt")

# find median.
# median_length * median_time

cor.test(length,epsilon, method = "spearman")