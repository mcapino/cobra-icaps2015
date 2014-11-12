### Load the libs 
source("functions.r")

### Load the data
runs <- load.and.preprocess('ubremen-r27')

### scatter plot of all gathered raw data

ggplot(runs, aes(instance, runs$avgtasktime/1000, color=alg, shape=alg)) + geom_point()
ggplot(runs, aes(instance, runs$makespan/1000, color=alg, shape=alg)) + geom_point()
ggplot(runs, aes(instance, runs$avgtravelt/1000, color=alg, shape=alg)) + geom_point()
ggplot(runs, aes(instance, runs$avgtravelr/1000, color=alg, shape=alg)) + geom_point()
ggplot(runs, aes(instance, runs$avgttprolong/1000, color=alg, shape=alg)) + geom_point()
ggplot(runs, aes(instance, runs$avgtrprolong/1000, color=alg, shape=alg)) + geom_point()

###

successrate.nagents(runs[runs$alg != 'BASE',])
avgtaskprolong.vs.nagents(runs[runs$alg != 'BASE',])
avgttprolong.vs.nagents(runs[runs$alg != 'BASE',])
avgtrprolong.vs.nagents(runs[runs$alg != 'BASE',])
