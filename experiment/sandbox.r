### Load the libs 
source("functions.r")

### Load the data
runs <- load.and.preprocess('warehouse-r25')

### scatter plot of all gathered raw data

ggplot(runs, aes(instance, runs$avgProlongT/1000, color=alg, shape=alg)) + geom_point()
ggplot(runs, aes(instance, runs$varProlongT/1000, color=alg, shape=alg)) + geom_point()
ggplot(runs, aes(instance, runs$avgProlongR/1000, color=alg, shape=alg)) + geom_point()
ggplot(runs, aes(instance, runs$varProlongR/1000, color=alg, shape=alg)) + geom_point()

ggplot(runs, aes(instance, runs$avgPlan/1000, color=alg, shape=alg)) + geom_point()
ggplot(runs, aes(instance, runs$varPlan/1000, color=alg, shape=alg)) + geom_point()

###

successrate.nagents(runs)
avgtaskprolong.vs.nagents(runs)

