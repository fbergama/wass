#!/bin/bash

sleep 10

STATUS_STR=$(curl --silent http://localhost:8080/fullstatus)

#echo "Current status: ${STATUS_STR}"
if [ "${STATUS_STR/*\"prepare\":true*/1}" == '1' ]; then
    echo "Prepared"
else
#    echo "Not prepared"
    curl http://localhost:8080/doprepare
fi

if [ "${STATUS_STR/*\"match\":true*/1}" == '1' ]; then
    echo "Matched"
else
#    echo "Not matched"
    curl http://localhost:8080/domatch
fi

if [ "${STATUS_STR/*\"matchmerge\":true*/1}" == '1' ]; then
    echo "Autocalibrated"
else
#    echo "Not autocalibrated"
    curl http://localhost:8080/domatchmerge
fi

if [ "${STATUS_STR/*\"dense\":true*/1}" == '1' ]; then
    echo "Dense reconstructed"
    exit 0
else
    echo "Not dense reconstructed"
    curl http://localhost:8080/dodense
fi

exit -1
