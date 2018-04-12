#!/bin/bash

USER_ID=${LOCAL_USER_ID:-9001}
echo "Starting with UID : $USER_ID"
useradd --shell /bin/bash -u $USER_ID -o -c "" -m user


export HOME=/home/user
#chown -R user:user /DATA_OUT
cd ext && nohup ./launch_redis.sh &
#nohup node Wass 2>&1  | tee -a wass.log &
exec /usr/local/bin/gosu user node Wass

