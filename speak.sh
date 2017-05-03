#!/bin/bash

cd $2

filename=$(echo $1 | md5sum)

echo $filename

echo $1 | text2wave -o "$filename".wav
gst-launch-1.0 playbin uri=file://$2/"$filename".wav audio-sink='alsasink device="hw:0,0"'
#rm $filename.wav
