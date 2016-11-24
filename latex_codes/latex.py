#!/usr/bin/python

import subprocess, sys

commands = [
    ['pdflatex', sys.argv[1] + '.txt'],
    ['bibtex', sys.argv[1] + '.aux'],
    ['pdflatex', sys.argv[1] + '.txt'],
    ['pdflatex', sys.argv[1] + '.txt']
]

for c in commands:
    subprocess.call(c)
