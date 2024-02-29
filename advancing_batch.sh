#!/bin/bash

#set the folders (linux)
#FLD=/home/federico/Documents/GitHub/Advancing-Volumes/data/
#RES=/home/federico/Documents/GitHub/Advancing-Volumes/results/
#EXE=/home/federico/Documents/GitHub/Advancing-Volumes/cmake-build-release/Advancing-Volumes
#set the folders (mac)
FLD=/Users/federicheddu/Documents/VOLMAP/G1
EXE=/Users/federicheddu/Documents/GitHub/Advancing-Volumes/cmake-build-release/Advancing-Volumes

#execution counter
CNT=0

#start the execution of the program with every .mesh file in the folder passed as argument, 10 programs in parallel,
#and, when one of them finishes, start a new one until all the files are processed

#touch $LOG
#echo "" >> $RES/log.txt
#echo "NEW EXECUTION - $(date)" >> $RES/log.txt

for MESH in `find $FLD -name "*.mesh" -type f`
do

  #create a folder with the name of the mesh file
  #mkdir -p $RES/$(basename $MESH .mesh)
  #create a log file inside the folder
  #touch $RES/$(basename $MESH .mesh)/log.txt
  #create line for log
  #touch $RES/$(basename $MESH .mesh)/line.txt

  #delete every .mesh file in the folder
  #rm -f $RES/$(basename $MESH .mesh)/*.mesh

  #append the date to the log file with a blank line before and after
  #echo "" >> $RES/$(basename $MESH .mesh)/log.txt
  #echo "NEW EXECUTION" >> $RES/$(basename $MESH .mesh)/log.txt
  #echo $(date) >> $RES/$(basename $MESH .mesh)/log.txt
  #echo "" >> $RES/$(basename $MESH .mesh)/log.txt

  #echo the name and the time of start of the execution of the program
  #echo "[$CNT] Executing $(basename $MESH .mesh) at $(date)"
  echo "[$CNT] Executing $MESH at $(date)"

  #start the execution of the program with the mesh file passed as argument
  #timeout 30m $EXE -t $MESH &
  gtimeout 30m $EXE -t $MESH &

  #increment the counter
  ((CNT++))
  #wait until there are more than 10 programs running
  while [ $(jobs -pr | wc -l) -ge 10 ]
  do
    sleep 1
  done

done

echo "End - ($(date))" >> $RES/log.txt