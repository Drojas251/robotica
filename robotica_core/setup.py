from setuptools import setup, find_packages

setup(
    name='robotica_core',
    version='0.1.0',
    packages=find_packages(),
    install_requires=[
        "robotica_datatypes",
        "numpy",
        "matplotlib",
        "pyyaml",
        "pyzmq",
        "PyQt5",
        "notebook",
        "ipykernel",
        "ipython==8.12.0",

    ],
    author='Your Name',
    author_email='your.email@example.com',
    description='Description of your package',
    license='Your License',
)