#!/bin/bash

#docker build --no-cache=true --build-arg BUILD_DATE=$(date -u +'%Y-%m-%dT%H:%M:%SZ') --build-arg VCS_REF=$(sed -n -e 's/^SET( WASS_VERSION_STR \"\(...\).* .*/v\1_/p' src/CMakeLists.txt)$(git describe --all --long) -t bergamasco/wass:latest .

docker build --no-cache=true -f Docker/Dockerfile.runall -t bergamasco/wass:runall .


