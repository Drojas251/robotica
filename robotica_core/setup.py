from setuptools import setup, find_packages

setup(
    name='robotica_core',
    version='0.1.0',
    packages=find_packages(),
    install_requires=[
        "robotica_datatypes",
        "numpy>=1.20",
        "matplotlib>=3.3.4",
        "pyyaml",
        "pyzmq==19.0.2",

    ],
    author='Your Name',
    author_email='your.email@example.com',
    description='Description of your package',
    license='Your License',
)