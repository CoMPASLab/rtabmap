#!/usr/bin/env bash

# Get script directory and cd
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
cd ${SCRIPT_DIR}

if [[ -z $@ ]]; then
    NUM_THREADS=16
else
    NUM_THREADS=$@
fi

docker build --rm -t rtabmap -f MBARI/Dockerfile . --build-arg NUM_THREADS=${NUM_THREADS} 