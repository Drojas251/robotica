#!/bin/bash

# Define ANSI color codes
RED='\033[0;31m'
NC='\033[0m' # No Color


if [ -n "$VIRTUAL_ENV" ]; then
    echo "You are currently in a virtual environment: $VIRTUAL_ENV"
else
    echo -e "${RED}You are not in a virtual environment. Please run source start_venv.sh ${NC}"
    source start_venv.sh
fi

pip install pip==20.3.4

# Build core package
cd ./robotica_datatypes
source ./build.sh
cd ../

cd ./robotica_core
source ./build.sh
cd ../

cd ./robotica_plugins
source ./build.sh
cd ../