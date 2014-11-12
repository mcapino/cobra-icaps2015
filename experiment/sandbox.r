### Load the libs 
source("functions.r")

### Load the data
runs <- load.and.preprocess('ubremen-r27')
maxagents <- max(runs$nagents) + 5

### scatter plot of all gathered raw data

ggplot(runs, aes(instance, runs$avgtasktime/1000, color=alg, shape=alg)) + geom_point()

###

successrate.nagents(runs[runs$alg != 'BASE',])
avgtaskprolong.vs.nagents(runs[runs$alg != 'BASE',])
