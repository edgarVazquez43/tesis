setwd(dir = "/home/edgar/github/tesis/resultados")

myData <- read.csv(file="completeTask.csv", header =T, sep=",")
attach(myData)
t.test(sinOrientacion,conOrientacion,alternative="two.sided",var.equal=T, paired=T)
