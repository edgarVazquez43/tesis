setwd(dir = "/home/edgar/github/tesis/resultados/objetos/Alturas")

myData <- read.csv(file="errores_cereal.csv", header =T, sep=",")
attach(myData)
t.test(Actual.cereal,Previo.cereal,alternative="two.sided",var.equal=T, paired=T)

myData_3 <- read.csv(file="errores_jugo.csv", header =T, sep=",")
attach(myData_3)
t.test(Actual.jugo,Previo.jugo,alternative="two.sided",var.equal=T, paired=T)

myData_4 <- read.csv(file="errores_milk.csv", header =T, sep=",")
attach(myData_4)
t.test(Actual.milk,Previo.milk,alternative="two.sided",var.equal=T, paired=T)

myData_1 <- read.csv(file="errores_chocolate.csv", header =T, sep=",")
attach(myData_1)
t.test(Actual.chocolate,Previo.chocolate,alternative="two.sided",var.equal=T, paired=T)

myData_2 <- read.csv(file="errores_joystick.csv", header =T, sep=",")
attach(myData_2)
t.test(Actual.joystick,Previo.joystick,alternative="two.sided",var.equal=T, paired=T)

