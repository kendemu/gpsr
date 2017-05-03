#!/usr/bin/env python
import csv
path = "/home/demulab/myprog/src/gpsr/config/"
questions = open(path+"speech_recognition_questions.csv").read()
questions = questions.split("\n")
answer = [questions[i].split(";")[1] for i in range(len(questions))]
questions = [questions[i].split(";")[0] for i in range(len(questions))]
for i in range(len(questions)):
    print questions[i],answer[i]
