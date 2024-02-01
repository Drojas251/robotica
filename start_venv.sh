#!/bin/bash

cur_dir=$(pwd)
venv_dir="$cur_dir/venv"

if [ -n "$VIRTUAL_ENV" ]; then
    echo "You are currently in a virtual environment: $VIRTUAL_ENV"
else
    if [[ -d "$venv_dir" ]]; then
        source venv/bin/activate
        echo "venv activated!"
    else
        echo "Creating venv $venv_dir"
        python3 -m venv venv
        echo "venv created!"

        source venv/bin/activate
        echo "venv activated!"

    fi
fi

cd ./robotica_core
source ./setup_env.sh
cd ../