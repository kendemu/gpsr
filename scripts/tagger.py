#!/usr/bin/env python
from nltk import *
text = word_tokenize(raw_input())
print pos_tag(text)
