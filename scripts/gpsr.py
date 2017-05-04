#!/usr/bin/env python
import sys
import rospy
import math
import nltk
from compiler.ast import flatten
from nltk import *
from nltk.corpus import wordnet as wn
from nltk.corpus import cmudict
from std_msgs.msg import String
from subprocess import call
from operator import add
import numpy as np
import time
import os
import ngram

path = "/home/demulab/catkin_ws/src/mini-voice-client2017"

navigation_pub = rospy.Publisher("/gpsr/navigation/input", String)

def speak(text):
    rospy.loginfo(text)
    call(["./speak.sh", text, path])


class GPSR:
    def __init__(self):
        self.speech = rospy.Subscriber("/voice_recog", String,self.speechcallback)
        self.question_res = rospy.Subscriber("/gpsr/question/result", String, self.questionResult)
        self.navigation_res = rospy.Subscriber("/gpsr/navigation/result", String, self.navigationResult)

        self.pro_dict = cmudict.dict()

        self.sp_control = rospy.Publisher("/speech_control", String)
        self.speech_input = ""
        self.token = []
        self.token_tag = []
        self.grasp_table = ["grass", "graphs"]
        self.move_table =["movie", "mood", "moods", "moved", "movie", "due", "new", "mooted", "newt", "mu", "muti"]
        self.find_table =["kind"]
        self.leave_table = ["leaves"]
        self.bring_table = ["bringing"]
        self.put_table = ["printed"]
        self.command_table = ["find", "move", "grasp", "put", "bring", "introduce", "guide", "leave", "answer", "navigate", "ask", "drive", "go", "tell", "place", "take", "get", "hand", "deliver", "locate", "look", "give", "say", "pick_up", "look_for"]
        self.search_categories = ["find", "look_for", "locate"]
        self.move_categories = ["go", "move", "drive", "navigate"]
        self.manip_categories = ["grasp", "pick_up", "get", "take"]
        self.speak_categories = ["tell", "say"]
        self.deliver_categories = ["deliver", "hand", "place", "give", "bring"]
        self.furnitures = {"bookshelf":"children's library","sofa":"living room","tv":"living room","table":"living room","bar table":"kitchen and dining room","table set one":"kitchen and dining room","table set two":"kichen and dining room", "bar_table" : "kitchen and dining room", "table_set_one":"kitchen and dining room", "table_set_two" : "kitchen and dining room", "operator": "operator"}
        self.furnitures_list = ["bookshelf", "sofa", "tv", "table", "bar table", "table set one", "table set two", "children library", "living room", "kitchen and dining room", "hallway", "operator"]
        self.rooms = ["children's library","living room","kitchen and dining room","hallway","exit", "operator"]
        self.nav_state = "waiting"
        self.ques_state = "waiting"

        #search: find, look_for
        #move: go, move, guide, navigate, drive
        #language: introduce, answer, ask, tell, say
        #object: grasp, put, bring, place, take, get, hand deliver, pick_up
        self.command_order = []
        self.objective_order = []
        os.chdir(path)
        
    def synsets(word, POS):
        word_list = wordnet.synsets(word)
        word_list = filter(lambda n : n.name().find(".%s." % (POS)) != -1, word_list)
        return word_list

    def tokenize(self,s_input):
        s_input = s_input.replace("bar table", "bar_table")
        s_input = s_input.replace("table set one", "table_set_one")
        s_input = s_input.replace("table set two", "table_set_two")
        print s_input

        self.token = word_tokenize(s_input)
        if len(self.token) > 0:
            self.token[0] = self.token[0].lower()
        self.token_tag = pos_tag(self.token)
        self.PRPProcessor()

        self.command_order = []

        token_tag_new = [list(self.token_tag[i]) for i in range(len(self.token_tag))]
        for i in range(len(self.token_tag)):
            if self.token[i] == "move" or self.token[i] == "grasp":
                token_tag_new[i][1] = "VB"
            if self.token[i] == "bar_table":
                token_tag_new[i][1] = "NNP"

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

    def mostSimilar(self, input_list, input_data):
        G = ngram.NGram(input_list)
        print G.search(input_data)
        return G.find(input_data)

    def PRPProcessor(self):
        predict = []
        self.objective_order = []


        for i in range(len(self.token_tag)):
            if self.token_tag[i][1] == "PRP":
                if self.token_tag[i][0] == "yourself":
                    self.token[i] = "Mini"
                    self.token_tag[i] = tuple(["Mini","NNP"])
                elif self.token_tag[i][0] == "me":
                    self.token[i] = "operator"
                    self.token_tag[i] = tuple(["operator","NNP"])

        for i in range(len(self.token_tag)):
            if self.token_tag[i][1] == "NN" or self.token_tag[i][1] == "NNP":
                predict.append(i)
                self.objective_order.append(self.token_tag[i][0])

        continuous = []
        for i in range(len(self.token_tag)):
            mini = [10000,len(predict)-1]
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
                        self.token_tag[i] = tuple([self.token[i], "NNP"])

    def stopVoiceRecog(self):
        com = String()
        com.data = "stop"
        self.sp_control.publish(com)

    def startVoiceRecog(self):
        com = String()
        com.data = "speak"
        self.sp_control.publish(com)

    def navigation(self, place):
        speak("I am moving to %s" % place)
        com = String()
        com.data = place
        self.nav_state = "running"
        navigation_pub.publish(com)

    def navigationResult(self, msg):
        self.nav_state = msg

    def navigationReset(self):
        self.nav_state = "waiting"

    def questionResult(self, msg):
        self.ques_state = msg

    def questionReset(self):
        self.ques_state = "waiting"

    def estimateWord(self, target_dict,target_list, target_word):
        if target_word in target_dict:
            f_pro_dict = []
            for j in range(len(target_list)):
                if self.furnitures_list[j].find(" ") != -1:
                    f_multi = target_list[j].split()
                    f = []
                    for k in range(len(f_multi)):
                        f = f + ['_'] + target_dict[f_multi[k]][0]
                    f_pro_dict.append("".join(f))
                else:
                    f_pro_dict.append("".join(target_dict[target_list[j]][0]))

            f_dict = dict([(f_pro_dict[j], target_list[j]) for j in range(len(target_list))])
            return f_dict[self.mostSimilar(f_pro_dict, "".join(target_dict[target_word][0]))]

        else:
            return self.mostSimilar(target_list, target_word)

    def speechcallback(self,data):
        print data
        token = self.tokenize(str(data).replace("data:",""))
        #self.PRPProcessor()
        print self.token

        #unable to hear you, or the sentence is invalid
        if len(self.command_order) is 0 or len(self.objective_order) is 0:
            self.stopVoiceRecog()
            speak("Error 1 : Unable to hear you, or the sentence is invalid")
            speak("Repeat again")
            self.startVoiceRecog()

        else:
            self.stopVoiceRecog()
            speak("your commands are ")
            print self.command_order
            print self.objective_order
            command_table = []
            for (command, objective) in zip(self.command_order, self.objective_order):
                command_ = "%s %s" % (command, objective)
                speak(command_)
                command_table.append(command_)
                

            #executing command
            i = 0

            while i < range(len(self.command_order)):
                if i == len(self.command_order):
                    speak("All commands accomplished.")
                    speak("I am leaving.")
                    speak("Bye.")
                    self.navigation("exit")
                    break

                if self.command_order[i] in self.move_categories:
                    if self.nav_state is "waiting":
                        if self.objective_order[i] in self.furnitures:
                            self.navigation(self.objective_order[i])
                        elif self.objective_order[i] in self.rooms:
                            self.navigation(self.objective_order[i])
                        else:
                            place = self.estimateWord(self.pro_dict, self.furnitures_list, self.objective_order[i])
                            if place is None:
                                speak("I don't know the location %s." % (place))
                                speak("Skipping command to number %d" % (i + 2))
                                i = i + 1
                            else:
                                self.navigation(place)

                                    

                    elif self.nav_state is "running":
                        rospy.loginfo("going to destination")
                    else:
                        speak("Arrived at destination.")
                        speak("Moving to next command.")
                        self.navigationReset()
                        i += 1


                elif self.command_order[i] == "leave":
                    speak("I am leaving.")
                    msg = String()
                    msg.data = "leave"
                    navigation_pub.publish(msg)
                    i += 1

                elif self.command_order[i] == "follow":
                    speak("I am following.")
                    msg = String()
                    msg.data = "follow"
                    navigation_pub.publish(msg)
                    i += 1

                elif self.command_order[i] == "answer":
                    speak("I am going to answer a question.")
                    if self.ques_state is "waiting":
                        self.startVoiceRecog()

                    elif self.ques_state is "running":
                        rospy.loginfo("answering question")

                    else:
                        self.stopVoiceRecog()
                        speak(sef.ques_state)
                        speak("Answered question.")
                        speak("Moving to next command.")
                        i += 1

                elif self.command_order[i] == "ask":
                    speak("I'm going to ask you a question.")
                    speak("Skipping to next command.")
                    i += 1

                elif self.command_order[i] in self.manip_categories:
                    speak("I don't have arms. Sorry.")
                    speak("Skipping to next command.")
                    i += 1

                elif self.command_order[i] in self.speak_categories:
                    speak("I have to talk to you.")
                    speak("Skipping to next command.")
                    i += 1

                elif self.command_order[i] in self.search_categories:
                    if self.nav_state is "waiting":
                        place = self.command_order[i + 1] if len(self.command_order[i]) > i + 1 else self.command_order[i - 1]
                        speak("I have to search somebody located in %s." % (place))
                        if place in self.furnitures:
                            self.navigation(place)
                        elif place in self.rooms:
                            self.navigation(place)
                        else:
                            place = self.estimateWord(self.pro_dict, self.furnitures_list, place)
                            if place is None:
                                speak("I don't know the location %s." % (place))
                                speak("Skipping to command number %d" % (i + 2))
                                i = i + 1
                            else:
                                self.navigation(place)

                    elif self.nav_state is "running":
                        rospy.loginfo("moving to destination.")

                    elif self.nav_state is not "running":
                        speak("Arrived at destination.")
                        speak("Moving to next command.")
                        self.navigationReset()
                        i += 1

                elif self.command_order[i] in self.deliver_categories:
                    if self.nav_state is "waiting":
                        place = self.furnitures["operator"]
                        speak("I am going to deliver.")
                        self.navigation(place)
                    elif self.nav_state is "running":
                        rospy.loginfo("returning back to operator.")

                    else:
                        speak("Returned back to operator.")
                        speak("Moving to next command.")
                        self.navigationReset()
                        i += 1

            self.startVoiceRecog()

        rospy.sleep(3)

def main(args):
    rospy.init_node("gpsr")
    gpsr = GPSR()
    rospy.spin()

if __name__ == "__main__":
    main(sys.argv)
