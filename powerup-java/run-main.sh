#!/bin/bash

OPENCV_SHARE=/usr/local/share/OpenCV/java

# If there are many opencv jars in the local library then take only one, randomly
for jar in ${OPENCV_SHARE}/opencv-*.jar
do
	opencv_jar=$jar
	break
done

set -x
java -cp ${opencv_jar}:./bin/ \
	-Djava.library.path=${OPENCV_SHARE}/ \
	org.error3130.powerup.Main $@

