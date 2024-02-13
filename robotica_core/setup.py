from setuptools import setup, find_packages

setup(
    name='robotica_core',
    version='0.1.0',
    packages=find_packages(),
    install_requires=[
        "robotica_datatypes",
        "argcomplete",
        "numpy",
        "matplotlib",
        "pyyaml",
        "pyzmq",
        "PyQt5",
        "python-fcl",

    ],
    entry_points={
        'console_scripts': [
            'robotica_sim = robotica_core.simulation.sim_app:app',
            'robotica_create_workspace = robotica_core.utils.robotica_create_workspace:create_robotica_ws',
            'robotica_generate_plugin = robotica_core.utils.generate_plugin:generate_plugin',
        ],
    },
    author='Your Name',
    author_email='your.email@example.com',
    description='Description of your package',
    license='Your License',
)