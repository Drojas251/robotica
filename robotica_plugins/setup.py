from setuptools import setup, find_packages

setup(
    name='robotica_plugins',
    version='0.1.0',
    packages=find_packages(),
    install_requires=[
        "robotica_core",
        "robotica_datatypes",
    ],
    entry_points={
        'console_scripts': [
            'build_plugins = robotica_plugins.build_plugins:build',
        ],
    },
    author='Your Name',
    author_email='your.email@example.com',
    description='Description of your package',
    license='Your License',
)