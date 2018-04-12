#!/bin/bash
SOURCE=`pwd`/out/wass/*

#DEST=`pwd`/temp
#DEST=bergamasco@daffyduck.dsi.unive.it:/home/bergamasco/temp/homepage
DEST=wass@ihoh.dsi.unive.it:/public/wass/

docpad generate --env static
rsync -h -v -r -P -t $SOURCE $DEST

