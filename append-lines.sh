#!/bin/bash

#set the folders
FLD=/Users/federicomeloni/Documents/GitHub/Advancing-Volumes/results
LOG=/Users/federicomeloni/Documents/GitHub/Advancing-Volumes/results/log.csv

rm $LOG
touch $LOG

#append the last line of line.txt in every subfolder of the results folder to the log file
for LN in `find $FLD/* -name line.txt -type f`
do
  tail -n 1 $LN >> $LOG
  echo "" >> $LOG
done
