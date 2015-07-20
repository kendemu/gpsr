#!/usr/bin/env python
import csv
from nltk import *
path = "/home/demulab/myprog/src/gpsr/config/"
questions = open(path+"speech_recognition_questions.csv").read()
questions = questions.split("\n")
answer = [questions[i].split(";")[1] for i in range(len(questions))]
questions = [questions[i].split(";")[0] for i in range(len(questions))]
questions_token = [word_tokenize(questions[i]) for i in range(len(questions))]
questions_token_tag = [pos_tag(questions_token[i]) for i in range(len(questions_token))]

in_ = []

for i in range(len(questions_token_tag)):
    for j in range(len(questions_token_tag[i])):
        if questions_token_tag[i][j][1] == "IN":
            in_.append(questions_token_tag[i][j][0])
            print questions_token_tag[i][j][0]

print len(set(in_))
print len(in_)    
