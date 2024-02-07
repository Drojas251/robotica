#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Set an environment variable with the current directory
export ROBOTICA_PLUGIN_PATH="$DIR/robotica_plugins"

# Run the setup.py sdist command
python setup.py sdist

# Install the package locally using pip
pip install -e .
