#!/bin/bash

# Run the setup.py sdist command
python setup.py sdist

# Install the package locally using pip
pip install -e .
