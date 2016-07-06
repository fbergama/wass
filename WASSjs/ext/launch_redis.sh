#!/bin/bash

REDIS_BIN_DIR='redis-2.8.19/src/'
REDIS_EXE='redis-server'
REDIS_CONFIG='redis.conf'

exec $REDIS_BIN_DIR$REDIS_EXE $REDIS_CONFIG
