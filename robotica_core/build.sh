#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Set an environment variable with the current directory
export ROBOTICA_CORE_DIR="$DIR"

# Check if the environment variable is set
if [ -z "$ROBOTICA_CORE_DIR" ]; then
    echo "Error: The environment variable ROBOTICA_CORE_DIR is not set."
fi

# Check if the script has been sourced in ~/.bashrc
if ! grep -q "source $ROBOTICA_CORE_DIR/robotica_functions.sh" ~/.bashrc; then
    # If not, append the source line to ~/.bashrc
    echo -e "\n# Source custom robotica functions" >> ~/.bashrc
    echo "source $ROBOTICA_CORE_DIR/robotica_functions.sh" >> ~/.bashrc
    echo "Custom functions sourced in ~/.bashrc. Restart your shell or run 'source ~/.bashrc' to apply."
else
    # If already sourced, print a message
    echo "Custom functions are already sourced in ~/.bashrc. No changes made."
fi


# Run the setup.py sdist command
python setup.py sdist

# Install the package locally using pip
pip install -e .
