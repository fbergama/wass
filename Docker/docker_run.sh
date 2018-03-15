#!/bin/bash

if [ $# -ne 3 ]
  then
    echo "usage: docker_run <CONF_DIR> <INPUT_DIR> <OUTPUT_DIR>"
fi

DATA_CONF_DIR=$1
DATA_IN_DIR=$2
DATA_OUT_DIR=$3

echo Mapping $DATA_CONF_DIR to /DATA_CONF
echo Mapping $DATA_IN_DIR to /DATA_IN
echo Mapping $DATA_OUT_DIR to /DATA_OUT

docker run -p 8080:8080 -e LOCAL_USER_ID=`id -u $USER` -v $DATA_CONF_DIR:/DATA_CONF -v $DATA_IN_DIR:/DATA_IN -v $DATA_OUT_DIR:/DATA_OUT -i -t  wass

