### Load the libs 

library(plyr)
library(ggplot2)
library(RColorBrewer)
library(gridExtra)

### Load the data

env <- "ubremen-r27"

dir <- paste("instances/",env, sep="")
imgdir <- paste(dir, "/figs/", sep="")
plotsdir <- paste(dir, "/../plots/", sep="")
runs <- read.csv(file=paste(dir, "/data.out.head", sep=""), head=TRUE, sep=";")
runs <- runs[order(runs$instance, runs$alg),]
runs$time[runs$time==0] <- NA

maxagents <- max(runs$nagents)

#runs$alg = factor(runs$alg,levels=c("BASE", "DFCFS", "ORCA"))
runs$alg <- revalue(runs$alg, c("DFCFS" = "COBRA"))

### scatter plot of all gathered raw data

ggplot(runs, aes(instance, time/1000, color=alg, shape=alg)) + geom_point()

###

common.runs <- function(runs, algs) {
  solved.by.all <- unique(runs$instance)
  for (alg in algs) {
    solved.by.all <- intersect(solved.by.all, unique(runs[runs$alg==alg & runs$status=="SUCCESS", "instance"]))                              
  }
  
  common.runs <- runs[is.element(runs$instance, solved.by.all) & is.element(runs$alg,algs), ] 
  return(common.runs)
}


############################################
######### MAIN GRAPHS BEGIN ################
############################################

pd <- position_dodge(2)

### success rate ####
succ.nagents <- function(runs, timelimit) {
  x <- runs[!is.na(runs$time) && runs$time<timelimit, ]
  
  succ <- ddply(x, .(nagents, alg), summarise,  
                solved = sum(status=="SUCCESS"),
                total = length(status)
  )
  
  plot <- ggplot(succ, aes(x=nagents, y=solved, color=alg, linetype=alg))+
    geom_line(size=1, position=pd)+ 
    geom_point(aes(shape=alg), position=pd, size=4, fill="white") + 
    scale_y_continuous(limits=c(0,max(succ$total)), name=paste("instances solved out of ", max(succ$total), "[-]")  ) +
    scale_x_continuous(limits=c(0,maxagents+3), name="number of robots [-]") +
    theme_bw() + 
    theme(legend.position = "bottom", legend.direction = "horizontal") +
    ggtitle("Success rate")
  
  return(plot)  
}
succ.nagents(runs, Inf)
ggsave(filename=paste(imgdir, "succ.vs.nagents.pdf", sep=""), width=4, height=4)

### runtime ###

runtime.vs.nagents <- function(runs) {  
  time.sum <- ddply(runs, .(nagents, alg), summarise,  
                    N = sum(!is.na(time)),
                    mean = mean(time),
                    med = median(time),
                    sd = sd(time),
                    se = sd / sqrt(N),
                    min = min(time),
                    max = max(time))
  
  plot <- ggplot(time.sum, aes(x=nagents, y=mean/1000, color=alg, linetype=alg, shape=alg)) +
    geom_errorbar(aes(ymin=(mean-se)/1000, ymax=(mean+se)/1000), width=1, position=pd, size=0.5, alpha=0.5) +
    geom_line(size=1, position=pd)+ 
    geom_point(size=4, position=pd, fill="white")+   
    scale_y_continuous(limits=c(min(time.sum$mean-time.sum$se, na.rm=TRUE)/1000, max(time.sum$mean+time.sum$se, na.rm=TRUE)/1000), name="time to finish all tasks [s]") +
    scale_x_continuous(limits=c(0,maxagents+3), name="no of robots [-]") +  
    theme_bw() +
    ggtitle("Avg. time to solution")
  
  return(plot)
}

runtime.vs.nagents(common.runs(runs, c("COBRA","BASE")))
ggsave(filename=paste(imgdir, "runtime.vs.nagents.pdf", sep=""), width=4, height=4)

## speedup ~ no of agents ##

speedup.vs.nagents <- function(runs) {
  x <-runs
  for (alg in c("PP", "ADPP","SDPP")) {
    x$speedup[x$alg==alg] <- 1/(x[x$alg==alg, "time"]/x[x$alg=="PP", "time"])
  }
  
  for (alg in c("RPP","ADRPP","SDRPP")) {
    x$speedup[x$alg==alg] <- 1/(x[x$alg==alg, "time"]/x[x$alg=="RPP", "time"])
  }
  
  # summarize
  
  speedup.sum <- ddply(x, .(nagents, alg, radius), 
                       summarise,  
                       N = sum(!is.na(speedup)),
                       mean = mean(speedup),
                       med = median(speedup),
                       sd = sd(speedup),
                       se = sd / sqrt(N),
                       min = min(speedup),
                       max = max(speedup))
  
  maxy <- max(speedup.sum$mean+speedup.sum$se, na.rm=TRUE)
  plot <- ggplot(speedup.sum, aes(x=nagents, y=mean, color=alg, shape=alg, linetype=alg))+
    geom_errorbar(aes(ymin=mean-se, ymax=mean+se), width=2, position=pd, size=0.5, alpha=0.5) +
    #geom_errorbar(aes(ymin=mean-sd, ymax=mean+sd), width=0, position=pd, size=2, alpha=0.7) +
    geom_line(size=1, position=pd)+ 
    geom_point(size=4, fill="white", position=pd)+   
    #geom_point(aes(y=med), size=3, shape=18, position=pd)+   
    #geom_text(aes(label=N, y=0, size=2), colour="black") + 
    scale_y_continuous(limits=c(0,maxy), name="avg. speed-up rel. to PP/RPP [-]") + 
    scale_x_continuous(limits=c(0, maxagents+3), name="number of robots [-]") + 
    geom_hline(yintercept = 1, linetype = "longdash", colour="black", alpha=0.5) + 
    
    scale_color_manual(values=get.color(unique(speedup.sum$alg)), name="method") +
    scale_linetype_manual(values=get.linetype(unique(speedup.sum$alg)), name="method") +
    scale_shape_manual(values=get.shape(unique(speedup.sum$alg)), name="method") +
    theme_bw() +
    ggtitle("3: Avg. speed-up rel. to centralized impl.")
  
  return(plot)
}
speedup.vs.nagents(common.runs(runs, .("PP","RPP","ADPP","ADRPP", "SDPP", "SDRPP")))
ggsave(filename=paste(imgdir, "speedup.vs.nagents.pdf", sep=""), width=4, height=4)


## replans vs. no of agents ##

replans.per.agent.vs.nagents <- function(runs) {  
  exp.sum <- ddply(runs, .(nagents, alg, radius), summarise,  
                   N = sum(!is.na(replans.per.agent)),
                   mean = mean(replans.per.agent, na.rm=TRUE),
                   sd = sd(replans.per.agent, na.rm=TRUE),
                   se = sd / sqrt(N),
                   min = min(replans.per.agent, na.rm=TRUE),
                   max = max(replans.per.agent, na.rm=TRUE))
  
  plot <- ggplot(exp.sum[exp.sum$alg != "CPP",], aes(x=nagents, y=mean, color=alg, shape=alg, linetype=alg))+
    geom_errorbar(aes(ymin=mean-se, ymax=mean+se), width=2, position=pd, size=0.5, alpha=0.5) +
    geom_line(size=1, position=pd)+ 
    geom_point(size=4, position=pd, fill="white")+   
    geom_hline(yintercept = 2, linetype = "longdash", colour="black", alpha=0.5) + 
    scale_x_continuous(limits=c(0,maxagents+3),name="number of robots [-]") +
    scale_y_continuous(name="avg. replannings per robot  [-]") +
    
    scale_color_manual(values=get.color(unique(exp.sum$alg)), name="method") +
    scale_linetype_manual(values=get.linetype(unique(exp.sum$alg)), name="method") +
    scale_shape_manual(values=get.shape(unique(exp.sum$alg)), name="method") +
    
    theme_bw() +
    ggtitle("4: Avg. number of replannings per robot")
  
  return(plot)
}

replans.per.agent.vs.nagents(common.runs(runs, .("ADPP", "ADRPP", "SDPP", "SDRPP")))
ggsave(filename=paste(imgdir, "replans.per.agent.vs.nagents.pdf", sep=""), width=4, height=4)

### quality ###

prolong.vs.nagents <- function(runs) {
  x <- runs
  
  for (alg in unique(runs$alg)) {
    x$prolong[x$alg==alg] <- 100*((x[x$alg==alg, "cost"]-x[x$alg=="BASEST", "cost"])/x[x$alg=="BASEST", "cost"])
  }
  
  # summarize
  
  prolong.sum <- ddply(x[x$alg != "BASEST",], .(nagents, alg), summarise,  
                       N = sum(!is.na(prolong)),
                       mean = mean(prolong, na.rm=TRUE),
                       med = median(prolong, na.rm=TRUE),
                       sd = sd(prolong, na.rm=TRUE),
                       se = sd / sqrt(N),
                       min = min(prolong, na.rm=TRUE),
                       max = max(prolong, na.rm=TRUE))
  
  plot <- ggplot(prolong.sum, aes(x=nagents, y=mean,  color=alg, shape=alg, linetype=alg))+
    geom_errorbar(aes(ymin=mean-se, ymax=mean+se), width=2, position=pd, size=0.5, alpha=0.5) +
    geom_line(size=1, position=pd)+ 
    geom_point(size=4, fill="white", position=pd)+ 
    #geom_text(aes(label=N, y=100, size=2), colour="black") + 
    scale_x_continuous(limits=c(0,maxagents+3), name="number of robots [-]") +
    scale_y_continuous(name="prolongation [%]") +
    
    scale_color_manual(values=get.color(unique(prolong.sum$alg)), name="method") +
    scale_linetype_manual(values=get.linetype(unique(prolong.sum$alg)), name="method") +
    scale_shape_manual(values=get.shape(unique(prolong.sum$alg)), name="method") +
    
    theme_bw()  +
    ggtitle("5: Avg. prolongation")
  return(plot)
}

prolong.vs.nagents(common.runs(runs, .("PP","RPP","ADPP","ADRPP", "SDPP", "SDRPP", "BASEST")))
ggsave(filename=paste(imgdir, "prolong.vs.nagents.pdf", sep=""), width=4, height=4)


########### Comparison with ORCA  ###################

prolong.vs.nagents(common.runs(runs, .("ADRPP", "ORCA", "BASEST")))
ggsave(filename=paste(imgdir, "prolong.vs.nagents.orca.pdf", sep=""), width=4, height=4)


#####################################################
#####################################################






####### GRID OF ALL PLOTS ##########

pd <- position_dodge(2)

success <- 
  succ.nagents(runs[is.element(runs$alg,.("PP", "RPP", "SDPP", "SDRPP", "ADPP", "ADRPP", "ORCA")),], Inf)

runtime <-
  runtime.vs.nagents(common.runs(runs, .("PP","RPP","ADPP","ADRPP", "SDPP", "SDRPP")))

speedup <-
  speedup.vs.nagents(common.runs(runs, .("PP","RPP","ADPP","ADRPP", "SDPP", "SDRPP")))

replans <-
  replans.per.agent.vs.nagents(common.runs(runs, .("ADPP", "ADRPP", "SDPP", "SDRPP")))

prolong <-
  prolong.vs.nagents(common.runs(runs, .("PP","RPP","ADPP","ADRPP", "SDPP", "SDRPP", "BASEST")))

g_legend<-function(p){
  tmp <- ggplotGrob(p)
  leg <- which(sapply(tmp$grobs, function(x) x$name) == "guide-box")
  legend <- tmp$grobs[[leg]]
  return(legend)}

legend <- g_legend(success)
lwidth <- sum(legend$width)
lheight <- sum(legend$heights)

grid.plots <- arrangeGrob(
  success + theme(legend.position="none"), 
  runtime + theme(legend.position="none"),
  speedup + theme(legend.position="none"),
  replans + theme(legend.position="none"),
  prolong + theme(legend.position="none"),  
  ncol=1)

ggsave(filename=paste(plotsdir, env,".pdf", sep=""), plot=grid.plots, width=5, height=18)
grid.plots

# grid.plots.w.legend <- arrangeGrob(
#   grid.plots,
#   legend,
#   heights=unit.c(unit(1, "npc") - lheight, lheight),
#   ncol=1)
# 
# grid.plots.w.legend
# 
# ggsave(filename=paste(plotsdir, env,"-leg.pdf", sep=""), plot=grid.plots.w.legend, width=8, height=10)
