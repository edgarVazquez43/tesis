#!/usr/bin/python

import os, sys, datetime

now = datetime.datetime.now()

commands = [
    "pdflatex -shell-escape " + sys.argv[1] + ".tex",
    "bibtex " + sys.argv[1] + ".aux",
    "pdflatex -shell-escape " + sys.argv[1] + ".tex",
    "pdflatex -shell-escape " + sys.argv[1] + ".tex",
    "mv " + sys.argv[1] + ".pdf ../pdf",
    "cp ../pdf/" + sys.argv[1] + ".pdf ../pdf/Desarrollo_de_un_sistema_de_deteccion_y_manipulacion_de_objetos.pdf",
    "clear",
    "rm ../pdf/Tesis_Edgar_*",
    "echo Compiling Succes...",
    "echo",
    "echo Remove old tesis files...",
    "cp ../pdf/" + sys.argv[1] + ".pdf ../pdf/Tesis_Edgar_" + str(now.year) + "-" + str(now.month) + "-" + str(now.day) + ".pdf",
    "echo Compiling on:  " + str(now)
]

for c in commands:
    os.system(c)
