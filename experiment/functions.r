library(plyr)
library(ggplot2)
library(RColorBrewer)
library(gridExtra)

pd <- position_dodge(2)

load.and.preprocess <- function(env) {
  dir <- paste("instances/",env, sep="")
  imgdir <- paste(dir, "/figs/", sep="")
  plotsdir <- paste(dir, "/../plots/", sep="")
  runs <- read.csv(file=paste(dir, "/data.out.head", sep=""), head=TRUE, sep=";")
  runs <- runs[order(runs$instance, runs$alg),]
  runs$alg <- revalue(runs$alg, c("DFCFS" = "COBRA"))
  runs$avgtasktime[runs$avgtasktime < 1] <- NA
  
  algs <- unique(runs$alg)
  for (instance in unique(runs$instance)) {
    for (alg in unique(runs$alg)) {
      runs$avgtaskprolong[runs$instance==instance & runs$alg==alg] <- 
        runs$avgtasktime[runs$instance==instance & runs$alg==alg] - runs$avgtasktime[runs$instance==instance & runs$alg=="BASE"] 
    }
  }

  return(runs) 
}

common.runs <- function(runs, algs) {
  solved.by.all <- unique(runs$instance)
  for (alg in algs) {
    solved.by.all <- intersect(solved.by.all, unique(runs[runs$alg==alg & runs$status=="SUCCESS", "instance"]))                              
  }
  
  common.runs <- runs[is.element(runs$instance, solved.by.all) & is.element(runs$alg,algs), ] 
  return(common.runs)
}

successrate.nagents <- function(runs) {
  #xbreaks <- unique(runs$nagents)
  successrate <- ddply(runs, .(nagents, alg), summarise,                     
                       successrate = sum(status=="SUCCESS") / length(unique(instance))
  )
  
  plot <- ggplot(successrate, aes(nagents, successrate*100, color=alg, linetype=alg, shape=alg)) + 
    geom_point(size=3) + geom_line(size=1) +
    scale_y_continuous(limits=c(0,100), name=("Instances solved [%]")) +  
    scale_x_continuous(limits=c(0,maxagents), name=("No of robots")) +
    ggtitle("Success rate") +
    theme_bw()
  
  return(plot)
}

avgtasktime.vs.nagents <- function(runs) {  
  time.sum <- ddply(runs, .(nagents, alg), summarise,  
                    N = sum(!is.na(time)),
                    mean = mean(avgtasktime, na.rm=TRUE),
                    med = median(avgtasktime, na.rm=TRUE),
                    sd = sd(avgtasktime, na.rm=TRUE),
                    se = sd / sqrt(N),
                    min = min(avgtasktime, na.rm=TRUE),
                    max = max(avgtasktime, na.rm=TRUE))
  
  plot <- ggplot(time.sum, aes(x=nagents, y=mean/1000, color=alg, linetype=alg, shape=alg)) +
    geom_errorbar(aes(ymin=(mean-se)/1000, ymax=(mean+se)/1000), width=1, position=pd, size=0.5, alpha=0.5) +
    geom_line(size=1, position=pd)+ 
    geom_point(size=3, position=pd, fill="white")+   
    scale_y_continuous(limits=c(min(time.sum$mean-time.sum$se, na.rm=TRUE)/1000, max(time.sum$mean+time.sum$se, na.rm=TRUE)/1000), name="avg. time to finish a tasks [s]") +
    scale_x_continuous(limits=c(0,maxagents), name="no of robots [-]") +  
    theme_bw() +
    ggtitle("Avg. time to finish a task")
  
  return(plot)
}

avgtaskprolong.vs.nagents <- function(runs) {  
  time.sum <- ddply(runs, .(nagents, alg), summarise,  
                    N = sum(!is.na(time)),
                    mean = mean(avgtaskprolong, na.rm=TRUE),
                    med = median(avgtaskprolong, na.rm=TRUE),
                    sd = sd(avgtaskprolong, na.rm=TRUE),
                    se = sd / sqrt(N),
                    min = min(avgtaskprolong, na.rm=TRUE),
                    max = max(avgtaskprolong, na.rm=TRUE))
  
  plot <- ggplot(time.sum, aes(x=nagents, y=mean/1000, color=alg, linetype=alg, shape=alg)) +
    geom_errorbar(aes(ymin=(mean-sd)/1000, ymax=(mean+sd)/1000), width=1, position=pd, size=1, alpha=0.5) +
    geom_line(size=1, position=pd)+ 
    geom_point(size=3, position=pd, fill="white")+   
    scale_y_continuous(limits=c(min(time.sum$mean-time.sum$se, na.rm=TRUE)/1000, max(time.sum$mean+time.sum$se, na.rm=TRUE)/1000), name="avg. prolongation [s]") +
    scale_x_continuous(limits=c(0,maxagents), name="no of robots [-]") +  
    theme_bw() +
    ggtitle("Avg. task prolongation due to collision avoidance")
  
  return(plot)
}

