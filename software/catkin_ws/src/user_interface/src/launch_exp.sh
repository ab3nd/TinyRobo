#!/bin/bash

#Get command line configuration: Participant ID and 
#experiment condition, used for creating file name

#From https://stackoverflow.com/questions/192249/how-do-i-parse-command-line-arguments-in-bash
for i in "$@"
do
case $i in
    -u=*|--user=*)
    USER_ID="${i#*=}"
    shift # past argument=value
    ;;
    -c=*|--condition=*)
    CONDITION="${i#*=}"
    shift # past argument=value
    ;;
    -l=*|--lib=*)
    LIBPATH="${i#*=}"
    shift # past argument=value
    ;;
    #--default)
    #DEFAULT=YES
    #shift # past argument with no value
    #;;
    *)
          # unknown option
    ;;
esac
done

#This is for specifing files, which I don't need
#if [[ -n $1 ]]; then
#    echo "Last line of file specified as non-opt/last argument:"
#    tail -1 $1
#fi 

#Get the date/time for the file name
DATE=`date +%Y-%m-%d.%H-%M-%S`

#TODO error check condtion, it needs to be 1,10,100,1000 or X
echo "USER_ID    = ${USER_ID}"
echo "CONDITION  = ${CONDITION}"
echo "DATE       = ${DATE}"

#Base output file name for videos
FNAME="user-${USER_ID}_cond-${CONDITION}_${DATE}"

#Names for the cameras
UPPER_CAM_FILE="${FNAME}_upper.mpg"
LOWER_CAM_FILE="${FNAME}_lower.mpg"

# Launch VLC recording instances and save the PIDs
vlc http://c09.lan/video.mjpg --sout=file/ts:${LOWER_CAM_FILE} &
LOWER_PID=$!
vlc http://c35.lan/video.mjpg --sout=file/ts:${UPPER_CAM_FILE} &
UPPER_PID=$!

#Launch the program, needs root to get touches
sudo /home/csrobot/TinyRobo/software/ui_test/ui_test.py -- -i ${USER_ID} -c ${CONDITION}

#Close the VLC instances
kill -TERM ${LOWER_PID}
kill -TERM ${UPPER_PID}

#Make a directory to put the files in and move them
USER_DIR="/home/csrobot/experiment_logs/${USER_ID}"
mkdir -p ${USER_DIR}
mv ${UPPER_CAM_FILE} ${USER_DIR}
mv ${LOWER_CAM_FILE} ${USER_DIR}
mv *.pickle ${USER_DIR}
