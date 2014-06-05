library(plyr)
library(ggplot2)

cpp.log <- read.csv("cpp.log", head=TRUE, sep=";")
cpp.log$alg <- "CPP"
adpp.log <- read.csv("adpp.log", head=TRUE, sep=";")
adpp.log$alg <- "ADPP"
sdpp.log <- read.csv("sdpp.log", head=TRUE, sep=";")
sdpp.log$alg <- "SDPP"

log <- rbind(cpp.log, adpp.log, sdpp.log)

log$us.per.state <- ""
log$us.per.state[log$expstates != 0] <- as.character(round(((log$duration[log$expstates != 0] / as.numeric(log$expstates[log$expstates != 0])))*1000000))
log$us.per.state[!is.na(log$us.per.state) & log$us.per.state != ""] <- paste(log$us.per.state[!is.na(log$us.per.state) & log$us.per.state != ""], "us/st")

log$expstates <- paste(log$expstates, "st")
log$expstates[log$expstates=="0 st"] <- ""

log$duration.str <- ""
log$duration.str[round(log$duration*1000) > 10] <- paste(round(log$duration[round(log$duration*1000) > 10]*1000), "ms")


ggplot(log[log$start < 2 & log$type=="EVENT_HANDLED", ], aes(x=process, ymin=(start), ymax=(start+duration), color=process)) + 
  geom_linerange(size=8) + 
  geom_point(aes(y=start), colour="black", size=1) + 
  geom_text(aes(y=start+duration/2, label=duration.str), colour="black", angle=0, size=3) +
  geom_text(aes(y=start+duration/2-duration/5, label=expstates), colour="blue", angle=0, size=3) +
  geom_text(aes(y=start+duration/2-2*duration/5, label=us.per.state), colour="blue", angle=0, size=3) +
  facet_wrap(~alg, ncol=3) +
  theme_bw()  
  

