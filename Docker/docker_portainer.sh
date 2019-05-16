#!/bin/bash

docker run -d -p 9000:9000 -v /var/run/docker.sock:/var/run/docker.sock portainer/portainer
echo "Open your browser at http://localhost:9000"
