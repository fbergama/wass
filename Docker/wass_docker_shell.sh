#!/bin/bash

MAPPINGPARAMS=()

DATA_CONF_DIR=$1
if [ -d "$DATA_CONF_DIR" ]; then
    echo Mapping $DATA_CONF_DIR to /DATA_CONF
    MAPPINGPARAMS+=( -v $DATA_CONF_DIR:/DATA_CONF)
fi

DATA_IN_DIR=$2
if [ -d "$DATA_IN_DIR" ]; then
    echo Mapping $DATA_IN_DIR to /DATA_IN
    MAPPINGPARAMS+=( -v $DATA_IN_DIR:/DATA_IN)
fi

DATA_OUT_DIR=$3
if [ -d "$DATA_OUT_DIR" ]; then
    echo Mapping $DATA_OUT_DIR to /DATA_OUT
    MAPPINGPARAMS+=( -v $DATA_OUT_DIR:/DATA_OUT)
fi

if [ -z "$MAPPINGPARAMS" ]; then
    echo "Note: you can run this script with $0 <CONF_DIR> <DATA_IN_DIR> <DATA_OUT_DIR>"
fi

docker run -p 8080:8080 -e LOCAL_USER_ID=`id -u $USER` "${MAPPINGPARAMS[@]}" -i -t --entrypoint /bin/bash  wass
