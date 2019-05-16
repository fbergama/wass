#!/bin/bash

WASS_ROOT=`pwd`/../


CONFIG_DIR=`pwd`/WASS_TEST/synth/config/
OUTDIR=`pwd`/output
INDIR=`pwd`/WASS_TEST/synth/input/

echo "Creating ${OUTDIR}"
mkdir $OUTDIR


until bash $WASS_ROOT/Docker/wass_auto_advance.sh; do  sleep 1;   done > /dev/null 2>&1 &
#ADV_PID=$!

$WASS_ROOT/Docker/wass_docker_run.sh $CONFIG_DIR $LOCAL_DIR/$INDIR $LOCAL_DIR/$OUTDIR

#echo "Killing auto_advance script"
#kill $ADV_PID

