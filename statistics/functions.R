find_median <- function(input, n_trials){
	median = array(0,((length(input)/n_trials)-1))
	mean   = array(0,((length(input)/n_trials)-1))
	for(i in 1:length(median)){
		index = (i-1)*n_trials + 1
		median[i] = median(input[index:(index+n_trials)])
		mean[i]   = mean(input[index:(index+n_trials)])
	}
	return(list(median=median,mean=mean))
}
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