#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

cur_dir=$(pwd)lsl
venv_dir="../venv"

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

pip install pip==20.3.4
pip install $SCRIPT_DIR/robotica_datatypes-0.1.0-py3-none-any.whl
pip install $SCRIPT_DIR/robotica_core-0.1.0-py3-none-any.whl
pip install $SCRIPT_DIR/robotica_plugins-0.1.0-py3-none-any.whl