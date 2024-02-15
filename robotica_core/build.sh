#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Set an environment variable with the current directory
export ROBOTICA_CORE_DIR="$DIR"

# Check if the environment variable is set
if [ -z "$ROBOTICA_CORE_DIR" ]; then
    echo "Error: The environment variable ROBOTICA_CORE_DIR is not set."
fi

# Run the setup.py sdist command
python setup.py sdist

# Install the package locally using pip
pip install -e .
