#!/usr/bin/python

import os, sys

commands = [
    "pdflatex " + sys.argv[1] + ".tex",
    "bibtex " + sys.argv[1] + ".aux",
    "pdflatex " + sys.argv[1] + ".tex",
    "pdflatex " + sys.argv[1] + ".tex",
    "mv " + sys.argv[1] + ".pdf ../pdf",
    "cp ../pdf/" + sys.argv[1] + ".pdf ../pdf/Desarrollo_de_un_sistema_de_deteccion_y_manipulacion_de_objetos.pdf"
]

for c in commands:
    os.system(c)
