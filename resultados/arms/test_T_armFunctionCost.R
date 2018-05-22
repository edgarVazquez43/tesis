setwd(dir = "/home/edgar/github/tesis/resultados/arms")

myData <- read.csv(file="costFuction_TEST_T.csv", header =T, sep=",")
attach(myData)
t.test(moveIt,geometrico,alternative="two.sided",var.equal=T, paired=T)

