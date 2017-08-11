#!/bin/bash

oldIntVal=0
intVal=0

oldRstVal=0
rstVal=0

while true; do
   intVal=`cat /sys/class/gpio/gpio410/value`
   rstVal=`cat /sys/class/gpio/gpio416/value`
   if [ $intVal != $oldIntVal ]; then
      echo "INT Changed at $(date '+%H:%M:%S.%N'): $oldIntVal -> $intVal"
      oldIntVal=$intVal
   fi
   if [ $rstVal != $oldRstVal ]; then
      echo "RST Changed at $(date '+%H:%M:%S.%N'): $oldRstVal -> $rstVal"
      oldRstVal=$rstVal
   fi
   #break
done

echo "INT old $oldIntVal, curr $intVal"
echo "RST old $oldRstVal, curr $rstVal"
date "+%H:%M:%S.%N"
