#!/bin/bash

USER_ID=${LOCAL_USER_ID:-9001}
echo "Starting with UID : $USER_ID"
useradd --shell /bin/bash -u $USER_ID -o -c "" -m user

export HOME=/home/user

if [ -f "/DATA_CONF/worksession.json" ]; then
    echo "Copying worksession.json from /DATA_CONF/"
    cp /DATA_CONF/worksession.json ./worksession.json 
fi

if [ -f "/DATA_CONF/settings.json" ]; then
    echo "Copying settings.json from /DATA_CONF/"
    cp /DATA_CONF/settings.json ./settings.json 
fi

export PATH=/wass/dist/bin:$PATH
cd ..
exec /usr/local/bin/gosu user /bin/bash

