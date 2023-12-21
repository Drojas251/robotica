#!/bin/bash

launch_robotica_sim() {
        if [ -z "$1" ] || [ -z "$2" ]; then
            echo "Usage: runmyscript <argument>"
        else
            echo "$1"
            echo "$2"
            python3 "$ROBOTICA_CORE_DIR/robotica_core/simulation/sim_app.py" "$1" "$2"
        fi
}
