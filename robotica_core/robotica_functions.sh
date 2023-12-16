#!/bin/bash

launch_robotica_sim() {
        if [ -z "$1" ]; then
            echo "Usage: runmyscript <argument>"
        else
            python3 "$ROBOTICA_CORE_DIR/robotica_core/simulation/sim_app.py" "$1"
        fi
}
