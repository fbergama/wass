#!/bin/sh

set -e
echo "Entrypoint"
echo "WASS running with ID:GID " $(id -u):$(id -g)


if [ "$(id -u)" = '0' ]; then
    echo "running WASS as root is not allowed. Switching to default user"
    exec gosu wass "$0" "$@"
fi


exec "$@"
