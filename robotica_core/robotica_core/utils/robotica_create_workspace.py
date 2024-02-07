import os
import argparse
import subprocess

def generate_setup_file(ws_name):
    return f'''
from setuptools import setup, find_packages

setup(
    name='{ws_name}',
    version='0.1.0',
    packages=find_packages(),
    install_requires=[
        "robotica_plugins",
        "robotica_datatypes",
    ],
    author='Your Name',
    author_email='your.email@example.com',
    description='Description of your package',
    license='Your License',
)

'''

def create_ws(ws_dict, root_path):
    os.makedirs(root_path)
    files = ws_dict['files']

    for file_name, file_content in files.items():
        file_path = os.path.join(root_path, file_name)
        with open(file_path, 'w') as file:
            file.write(file_content)

    for dir_name, dir_content in ws_dict['dirs'].items():
        dir_path = os.path.join(root_path, dir_name)
        create_ws(dir_content, dir_path)


def create_robotica_ws():
    parser = argparse.ArgumentParser(description='Create a robotica workspace')
    parser.add_argument('--ws-name', required=True, help='Workspace name')
    args = parser.parse_args()

    ws_name = args.ws_name

    setup_file = generate_setup_file(ws_name)
    ws = {
        'dirs':{
            'plugins':{
                'dirs':{
                    'kinematics':{
                        'dirs':{
                            'plugins':{
                                'dirs':{},
                                'files':{
                                    '__init__.py': '',
                                }
                            },
                        },
                        'files':{
                            '__init__.py': '',
                            'definition.py':'',
                        }
                    },
                    'path_planners':{
                        'dirs':{
                            'plugins':{
                                'dirs':{},
                                'files':{
                                    '__init__.py': '',
                                }
                            },
                        },
                        'files':{
                            '__init__.py': '',
                            'definition.py':'',
                        }
                    },
                    'trajectory_planners':{
                        'dirs':{
                            'plugins':{
                                'dirs':{},
                                'files':{
                                    '__init__.py': '',
                                }
                            },
                        },
                        'files':{
                            '__init__.py': '',
                            'definition.py':'',
                        }
                    },
                },
                'files':{
                    '__init__.py': '',
                }
            },
            'robots':{
                'dirs':{},
                'files':{
                    '__init__.py': '',
                }
            },
            'environments':{
                'dirs':{},
                'files':{
                    '__init__.py': '',
                }
            },
            'scripts':{
                'dirs':{},
                'files':{
                    '__init__.py': '',
                }
            },
        },
        'files': {
            'README.md': '# My Workspace\n\nThis is a sample workspace.',
            'robotica_ws.txt': f'workspace:={ws_name}',
            '__init__.py': '',
            'setup.py': setup_file,
        }
    }
    create_ws(ws, ws_name)
    curr_path = os.getcwd()
    ws_path = os.path.join(curr_path, ws_name)
    os.chdir(ws_path)
    subprocess.run(['pip', 'install', '-e', '.'])


if __name__ == "__main__":
    # Define workspace details
    create_robotica_ws()