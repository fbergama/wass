#!/bin/bash
#docker build  --build-arg USER_ID=$(id -u ${USER}) --build-arg GROUP_ID=$(id -g ${USER}) -t wass .

docker build --no-cache=true --build-arg BUILD_DATE=$(date -u +'%Y-%m-%dT%H:%M:%SZ') --build-arg VCS_REF=$(sed -n -e 's/^SET( WASS_VERSION_STR \"\(...\).* .*/v\1_/p' src/CMakeLists.txt)$(git describe --all --long) -t wass:latest .

