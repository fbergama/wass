#!/bin/bash


if [ -f "/DATA_CONF/worksession.json" ]; then
    echo "Copying worksession.json from /DATA_CONF/"
    cp /DATA_CONF/worksession.json ./worksession.json
fi

if [ -f "/DATA_CONF/settings.json" ]; then
    echo "Copying settings.json from /DATA_CONF/"
    cp /DATA_CONF/settings.json ./settings.json
fi

exec node Wass

