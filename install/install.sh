#!/bin/bash

sudo apt install python3.8-venv python3.8-dev
sudo apt install virtualenv

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
        python3.8 -m venv venv
        echo "venv created!"

        source venv/bin/activate
        echo "venv activated!"

    fi
fi

pip install pip==20.3.4
pip install $SCRIPT_DIR/robotica_datatypes-0.1.0-py3-none-any.whl
pip install $SCRIPT_DIR/robotica_core-0.1.0-py3-none-any.whl
pip install $SCRIPT_DIR/robotica_plugins-0.1.0-py3-none-any.whl

# Create .robotica directory structure in home directory
ROBOTICA_DIR="$HOME/.robotica"
WORKSPACE_DIR="$ROBOTICA_DIR/workspaces"
PLUGINS_DIR="$ROBOTICA_DIR/plugins"
ACTIVE_WS_FILE="$WORKSPACE_DIR/active_ws.yml"
RUN_PLUGINS_FILE="$PLUGINS_DIR/run_plugins"

mkdir -p "$WORKSPACE_DIR"
mkdir -p "$PLUGINS_DIR"


# Create active_ws.yml if it doesn't already exist
if [[ ! -f "$ACTIVE_WS_FILE" ]]; then
    echo "# Active workspace config" > "$ACTIVE_WS_FILE"
    echo "workspaces: []" >> "$ACTIVE_WS_FILE"
    echo "Created $ACTIVE_WS_FILE"
else
    echo "$ACTIVE_WS_FILE already exists."
fi

# Create active_ws.yml if it doesn't already exist
if [[ ! -f "$RUN_PLUGINS_FILE" ]]; then
    echo "# Active workspace config" > "$RUN_PLUGINS_FILE"
    #echo "active_workspace: default" >> "$RUN_PLUGINS_FILE"
    echo "Created $RUN_PLUGINS_FILE"
else
    echo "$RUN_PLUGINS_FILE already exists."
fi