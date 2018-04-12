#!/bin/bash

if [ $# -ne 3 ]
  then
    echo "usage: docker_run <CONF_DIR> <INPUT_DIR> <OUTPUT_DIR>"
    exit 0
fi

DATA_CONF_DIR=$1
if [ ! -d "$DATA_CONF_DIR" ]; then
    echo Invalid configuration directory: $DATA_CONF_DIR
    exit 0
fi

DATA_IN_DIR=$2
if [ ! -d "$DATA_IN_DIR" ]; then
    echo Invalid input directory: $DATA_IN_DIR
    exit 0
fi

DATA_OUT_DIR=$3
if [ ! -d "$DATA_OUT_DIR" ]; then
    echo Invalid output directory: $DATA_OUT_DIR
    exit 0
fi

echo Mapping $DATA_CONF_DIR to /DATA_CONF
echo Mapping $DATA_IN_DIR to /DATA_IN
echo Mapping $DATA_OUT_DIR to /DATA_OUT

sudo docker run -p 8080:8080 -e LOCAL_USER_ID=`id -u $USER` -v $DATA_CONF_DIR:/DATA_CONF -v $DATA_IN_DIR:/DATA_IN -v $DATA_OUT_DIR:/DATA_OUT -i -t  wass

