#!/usr/bin/env python
# -*- coding:utf-8 -*-  

import rospy
import MeCab
import string
import tagger
from std_msgs.msg import String
#import sys
import nltk
from nltk.corpus import wordnet as wn
#from sound_play.msg import String
#from sound_play.libsoundplay import SoundClient
#from subprocess import call
#import time
#from geometry_msgs.msg import Twist

MECAB_MODE = 'mecabrc'
PARSE_TEXT_ENCODING = 'utf-8'

#path = "/home/demulab/catkin_ws/src/mini-voice-client"

class GPSR:
  def __init__(self):
    self.speech = rospy.Subscriber('voice_recog', String, self.NLP)

  def NLP(self, message):  
    #parse()メゾットは、文字列を解析し、任意で解析によって作り出された値を変換する
    mini = self.parse(message.data)

    print mini
    #join()メゾットは、配列内のすべての要素を文字列として連結する
    print "all:",",".join(mini['all'])
    print "noun:",",".join(mini['noun'])
    print "verb:",",".join(mini['verb'])


  def parse(self, unicode_string):
    print "starting parse"
    tagger = MeCab.Tagger(MECAB_MODE)
    text = unicode_string.encode(PARSE_TEXT_ENCODING)
    node = tagger.parseToNode(text)

    nouns = []
    verds = []


    parsed_words_dict = {}

    while node:
      print "inside of loop"
      pos = node.feature.split(",")
      word = node.surface.decode("utf-8") #MeCabに戻ってきた文字列はdecodeする
      if pos == "nouns":
        nouns.append(pos)
      elif pos == "verbs":
        verbs.append(pos)
        parsed_words_dict={
          "nouns": nouns,
          "verbs": verbs
        }
      node = node.next

    print "out of loop"
    return parsed_words_dict

if __name__ == "__main__":
  rospy.init_node('gpsr')
  print "starting GPSR"
  gpsr = GPSR()
  rospy.spin()
  #rate = rospy.Rate(100)
  #while not rospy.is_shutdown():
  #  rate.sleep()
