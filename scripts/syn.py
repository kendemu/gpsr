#!/usr/bin/env python
from nltk.corpus import wordnet as wn
from compiler.ast import flatten
synset_list = wn.synsets(raw_input())
lemmas = flatten([synset_list[i].lemma_names() for i in range(len(synset_list))])
vocab = list(set(lemmas))
print len(vocab)
for i in range(len(vocab)):
    print vocab[i]
