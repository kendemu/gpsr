#!/usr/bin/env python
import sys
import rospy
import math
import nltk
from compiler.ast import flatten
from nltk import *
from nltk.corpus import wordnet as wn
from std_msgs.msg import String
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
from subprocess import call
import time
import os
from geometry_msgs.msg import Twist

path = "/home/demulab/catkin_ws/src/mini-voice-client"

navigation_pub = rospy.Publisher("/gpsr_navigation", String)
question_pub = rospy.Publisher("/gpsr_question", String)


def speak(text):
    call(["./speak.sh", text, path])


class GPSR:
    def __init__(self):
        self.speech = rospy.Subscriber("/voice_recog", String,self.speechcallback)
        self.sp_control = rospy.Publisher("/speech_control", String)
        self.speech_input = ""
        self.token = []
        self.token_tag = []
        self.soundhandle = SoundClient()
        self.voice = "voice_kal_diphone"
        self.grasp_table = ["grass", "graphs"]
        self.move_table =["movie", "mood", "moods", "moved", "movie", "due", "new", "mooted", "newt", "mu", "muti"]
        self.find_table =["kind"]
        self.leave_table = ["leaves"]
        self.bring_table = ["bringing"]
        self.put_table = ["printed"]
        self.command_table = ["find", "move", "grasp", "put", "bring", "introduce", "guide", "leave", "answer"]
        self.command_order = []
        self.objective_order = []
        os.chdir(path)
        
    def tokenize(self,s_input):
        self.token = word_tokenize(s_input)
        if len(self.token) > 0:
            self.token[0] = self.token[0].lower()
        self.token_tag = pos_tag(self.token)
        self.command_order = []

        token_tag_new = [list(self.token_tag[i]) for i in range(len(self.token_tag))]
        for i in range(len(self.token_tag)):
            if self.token[i] == "move" or self.token[i] == "grasp":
                token_tag_new[i][1] = "VB"
            if self.token[i] == "bed" or self.token[i] == "couch" or self.token[i] == "noodles":
                token_tag_new[i][1] = "NN"
            if self.token[i] in self.grasp_table:
                self.token[i] = "grasp"
                token_tag_new[i][0] = "grasp"
                token_tag_new[i][1] = "VB"
            if self.token[i] in self.move_table:
                self.token[i] = "move"
                token_tag_new[i][0] = "move"
                token_tag_new[i][1] = "VB"
            if self.token[i] in self.find_table:
                self.token[i] = "find"
                token_tag_new[i][0] = "find"
                token_tag_new[i][1] = "VB"
            if self.token[i] in self.put_table:
                self.token[i] = "put"
                token_tag_new[i][0] = "put"
                token_tag_new[i][1] = "VB"                
            if self.token[i] is "piercing":
                self.token[i] = "person"
                token_tag_new[i][0] = "person"
                token_tag_new[i][1] = "NN"
            if self.token[i] in self.leave_table:
                self.token[i] = "leave"
                token_tag_new[i][0] = "leave"
                token_tag_new[i][1] = "VB"
            if token_tag_new[i][0] in self.command_table:
                self.command_order.append(token_tag_new[i][0])

        self.token_tag = [tuple(token_tag_new[i]) for i in range(len(self.token_tag))]
        return self.token

    def synonym(self,s_input):
        synset_list = wn.synsets(s_input)
        lemmas = flatten([synset_list[i].lemma_names() for i in range(len(synset_list))])
        vocab = list(set(lemmas))
        return vocab

    def PRPProcessor(self):
        predict = []
        self.objective_order = []

        for i in range(len(self.token_tag)):
            if self.token_tag[i][1] == "NN" or self.token_tag[i][1] == "NNP":
                predict.append(i)
                self.objective_order.append(self.token_tag[i])
        continuous = []
        for i in range(len(self.token_tag)):
            mini = [10000,len(predict)-1]
            if self.token_tag[i][1] == "PRP":
                if self.token_tag[i][0] == "yourself":
                    self.token[i] = "Mini"
                    self.token_tag[i] = tuple(["Mini","NNP"])
                elif self.token_tag[i][0] == "me":
                    self.token[i] = "operator"
                    self.token_tag[i] = tuple(["operator","NNP"])

                else:
                    for j in range(len(predict)):
                        if mini[0] > i - predict[j] and i - predict[j] > 0:
                            mini = [i-predict[j],predict[j]]
                        j = mini[1]
                        while j > 0:
                            if self.token_tag[j-1][1] == "NNP" or self.token_tag[j-1][1] == "NN":
                                continuous.append(self.token[j-1])
                            else: break
                            j-= 1
                        continuous.reverse()
                        if len(continuous) > 0:
                            self.token[i] = " ".join(continuous)+" "+self.token[mini[1]]
                        else:
                            self.token[i] = self.token[mini[1]]
                            print self.token[i]
                            self.token_tag[i] = tuple([self.token[i], "NNP"])

    def speechcallback(self,data):
        print data
        token = self.tokenize(str(data).replace("data:",""))
        self.PRPProcessor()
        syn_list = []
        print "mean token"
        for i in range(len(token)):
            if self.token_tag[i][1] != "CC" and self.token_tag[i][1] != "DT" and self.token_tag[i][1] != "TO":
                print token[i], self.token_tag[i]
                syn_list.append(self.synonym(token[i]))
        print "synonyms"
        _syn = []
        for i in range(len(syn_list)):
            syn = pos_tag(syn_list[i])
            __syn = []
            for j in range(len(syn)):
                if syn[j][1] == self.token_tag[i][1]:
                    if self.token_tag[i][1] != "NNP":
                        __syn.append(syn[j][0])
            syn.append(__syn)

        # for i in range(len(_syn)):
        #     print _syn[i]

        #for i in range(len(token)):
        #    self.soundhandle.say("your objective."+" ".join(token),self.voice)
        #    print token[i]
        if len(self.command_order) is 0 or len(self.objective_order) is 0:
            com = String()
            com.data = "stop"
            self.sp_control.publish(com)
            speak("Sentence invalid")
            speak("Repeat again")
            com.data = "speak"
            self.sp_control.publish(com)

        else:
            com = String()
            com.data = "stop"
            self.sp_control.publish(com)
            speak("your commands are ")
            for i in range(len(self.command_order)):
                speak("command "+str(i))
                speak(self.command_order[i])
                if self.command_order[i] == "move":
                    msg = String()
                    msg.data = self.objective_order[i]
                    navigation_pub.publish(msg)
                elif self.command_order[i] == "leave":
                    msg = String()
                    msg.data = "leave"
                    navigation_pub.publish(msg)
                elif self.command_order[i] == "answer":
                    msg = String()
                    msg.data = "answer"
                    question_pub.publish(msg)

            speak("your objectives are")
            for i in range(len(self.objective_order)):
                speak("objective " + str(i))
                speak(self.objective_order[i])
        #for i in range(len(self.command_order)):
            #rospy.loginfo("%s %s",self.command_order[i], self.objective_order[i][0])
            com.data = "speak"
            self.sp_control.publish(com)

        rospy.sleep(3)

def main(args):
    rospy.init_node("gpsr")
    gpsr = GPSR()
    rospy.spin()

if __name__ == "__main__":
    main(sys.argv)
