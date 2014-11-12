### Load the libs 

source("functions.r")

# the plots will be saved to this directory


make.plots <- function(imgdir, env) {
  runs <- load.and.preprocess(env)
  maxagents <- max(runs$nagents) + 3
  dir.create(file.path(imgdir, env), showWarnings = FALSE)
  
  
  successrate.nagents(runs[runs$alg != 'BASE',])
  ggsave(filename=paste(imgdir, env, "/success.pdf", sep=""), width=6, height=3.2)
  
  avgtaskprolong.vs.nagents(runs[runs$alg != 'BASE',])
  ggsave(filename=paste(imgdir, env, "/prolongation.pdf", sep=""), width=6, height=3.2)
}

imgdir <- "plots/"
make.plots(imgdir=imgdir, env="ubremen-r27")
make.plots(imgdir="/home/capino/projects/deconfliction/repo-doc/icaps2015/plots/", env="ubremen-r27")

