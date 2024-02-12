import os
import argparse
import subprocess
from robotica_core.utils.workspace_manager import add_workspace

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

def build_plugins_script():
    return f'''
#############################################
# DO NOT EDIT THIS FILE
#
# This file is auto-generated and is used to
# build the plugins developed in this package. 
##############################################

import os
from robotica_plugins.utils.build_plugins import build_plugins

# Build Plugins
script_directory = os.path.dirname(os.path.abspath(__file__))
build_plugins(script_directory)

'''

def init_definition_file(plugin_type):
    return f'''
#############################################
# DO NOT EDIT THIS FILE
#
# This file is auto-generated and is used to
# load the plugins developed in this package. 
##############################################

{plugin_type} = {{}}
'''

def robot_interface_file():
    return f'''
from robotica_plugins.utils.robot_api import RobotAPI
from plugins.kinematics.definition import Kinematics_Plugins
from plugins.trajectory_planners.definition import Trajectory_Planners_Plugins
from plugins.path_planners.definition import Path_Planners_Plugins

class RobotInterface:
    def __init__(self, robot_yml_file, world_yml_file=None):
        self.api = RobotAPI(
            robot_yml_file, 
            world_yml_file,
            kinematics_plugins = Kinematics_Plugins, 
            cartesian_traj_plugins = Trajectory_Planners_Plugins,
            path_planner_plugins = Path_Planners_Plugins,
        )

    #############################
    # Robot Application Interface
    #############################
        
    def plan_and_execute(self, goal):
        self.api.plan_and_execute(goal)

    def execute_path(self, path):
        self.api.execute_path(path)
    
    def joint_move(self, joint_trajectory):
        self.api.joint_move(joint_trajectory)
    
    def get_curr_joint_pos(self):
        return self.api.get_current_joint_positions()
    
    def get_curr_ee_pos(self):
        pass
    
    ######################
    # Robot Core Interface
    ######################
        
    def inverse_kinematics(self, point, cfg=0):
        return self.api.kinematics.inverse_kinematics(point, cfg)
    
    def forward_kinematics(self, joints):
        return self.api.kinematics.forward_kinematics(joints)
    
    def plan(self, start, goal):
        return self.api.path_planner.plan(start, goal)
'''

def gen_example_robot_yml():
    return f'''
robot_name: "example_2_link_robot"
dh_params:
  theta: [1.3,-0.9]
  a: [0.3, 0.25]
  d: [0.0, 0.0]
  alpha: [0.0, 0.0]
joint_limits:
  j1:
    position:
      min: -2.44
      max: 2.44
    velocity:
      min: 1
      max: 2
  j2:
    position:
      min: -2.5
      max: 2.5
    velocity:
      min: 1
      max: 3
'''

def gen_example_world_yml():
    return f'''
objects:
  obj_1:
    type: "rectangle"
    pose:
      x: -0.5
      y: 0.5
    size:
      x: 0.15
      y: 0.15
  obj_2:
    type: "rectangle"
    pose:
      x: 0.5
      y: 0.5
    size:
      x: 0.15
      y: 0.15
'''

def gen_applcation_script():
    return f'''
import os
from scripts.robot_interface import RobotInterface

# Example robot and world configuration files.
# Modify these files or create new ones for your application.
# The path to these files are required to initialize the RobotInterface.
robot_path = os.path.join("..", "configs/robots", "example_robot.yml")
world_path = os.path.join("..", "configs/worlds", "example_world.yml")

robot = RobotInterface(robot_path, world_yml_file=world_path)

robot.plan_and_execute((0.05, 0.54))
robot.plan_and_execute((0.1, 0.24))
robot.plan_and_execute((0.25, 0.5))
'''

def gen_test_kinematics_plugin():
    return f'''
import numpy as np
from robotica_plugins.kinematics.kinematics_plugin_interface import KinematicPluginInterface

class ExampleKinematicsPlugin(KinematicPluginInterface):
    def __init__(self, robot_model):
        KinematicPluginInterface.__init__(self, robot_model)
        
    def compute_forward_kinematics(self):
        """
        Description:
            Forward kinematics refers to the use of the kinematic equations of a robot to compute 
            the position of the end-effector from specified values for the joint parameters.
            Joint Angles (Theta_1, Theta_2) <-> Position of End-Effector (x, y)
                    
        Return:
            ee_point [Float Array]: End effector Pose in X,Y 

        Examples:
            self.compute_forward_kinematics([0.0, 1.57])
        """

        ee_point = np.zeros(2)
        ee_point[0] = round(self.DH_params.a[0]*np.cos(self.DH_params.theta[0]) + self.DH_params.a[1]*np.cos(self.DH_params.theta[0] + self.DH_params.theta[1]), 10)
        ee_point[1] = round(self.DH_params.a[0]*np.sin(self.DH_params.theta[0]) + self.DH_params.a[1]*np.sin(self.DH_params.theta[0] + self.DH_params.theta[1]), 10)

        return ee_point
    
    def compute_inverse_kinematics(self, point, cfg):
        theta_aux     = np.zeros(2)

        # Cosine Theorem [Beta]: eq (1)
        cosT_beta_numerator   = ((self.DH_params.a[0]**2) + (point[0]**2 + point[1]**2) - (self.DH_params.a[1]**2))
        cosT_beta_denumerator = (2*self.DH_params.a[0]*np.sqrt(point[0]**2 + point[1]**2))
        
        # Calculation angle of Theta 1,2 (Inverse trigonometric functions):
        # Rule 1: The range of the argument “x” for arccos function is limited from -1 to 1.
        # −1 ≤ x ≤ 1
        # Rule 2: Output of arccos is limited from 0 to π (radian).
        # 0 ≤ y ≤ π

        # Calculation angle of Theta 1
        if cosT_beta_numerator/cosT_beta_denumerator > 1:
            theta_aux[0] = np.arctan2(point[1], point[0]) 
            print('[INFO] Theta 1 Error: ', point[0], point[1])
        elif cosT_beta_numerator/cosT_beta_denumerator < -1:
            theta_aux[0] = np.arctan2(point[1], point[0]) - np.pi 
            print('[INFO] Theta 1 Error: ', point[0], point[1]) 
        else:
            if cfg == 0:
                theta_aux[0] = np.arctan2(point[1], point[0]) - np.arccos(cosT_beta_numerator/cosT_beta_denumerator)
            elif cfg == 1:
                theta_aux[0] = np.arctan2(point[1], point[0]) + np.arccos(cosT_beta_numerator/cosT_beta_denumerator)
                
        # Cosine Theorem [Alha]: eq (2)
        cosT_alpha_numerator   = (self.DH_params.a[0]**2) + (self.DH_params.a[1]**2) - (point[0]**2 + point[1]**2)
        cosT_alpha_denumerator = (2*(self.DH_params.a[0]*self.DH_params.a[1]))

        # Calculation angle of Theta 2
        if cosT_alpha_numerator/cosT_alpha_denumerator > 1:
            theta_aux[1] = np.pi
            print('[INFO] Theta 2 Error: ', point[0], point[1])
        elif cosT_alpha_numerator/cosT_alpha_denumerator < -1:
            theta_aux[1] = 0.0
            print('[INFO] Theta 2 Error: ', point[0], point[1])
        else:
            if cfg == 0:
                theta_aux[1] = np.pi - np.arccos(cosT_alpha_numerator/cosT_alpha_denumerator)
            elif cfg == 1:
                theta_aux[1] = np.arccos(cosT_alpha_numerator/cosT_alpha_denumerator) - np.pi

        return theta_aux


    def compute_differential_kinematics(self, p_target, theta, accuracy, num_of_iter):
        pass
'''

def gen_test_path_planner_plugin():
    return f'''
from robotica_plugins.path_planners.path_planning_plugin_interface import PathPlannerPluginInterface
from robotica_datatypes.path_datatypes.waypoint import WayPoint

class ExamplePathPlannerPlugin(PathPlannerPluginInterface):
    def __init__(self, *args):
        PathPlannerPluginInterface.__init__(self, *args)

    def planner(self, start, goal):
        if not self.validate_point(goal):
            print("Not a valid goal point")
            return None

        return [WayPoint(start, 0.3), WayPoint(goal, 0.3)]
'''

def gen_test_trajectory_planner_plugin():
    return f'''
import numpy as np
from robotica_plugins.trajectory_planners.trajectory_planning_interface import CartesianTrajectoryPluginInterface

class ExampleTrajPlannerPlugin(CartesianTrajectoryPluginInterface):
    def __init__(self, kinematics):
        CartesianTrajectoryPluginInterface.__init__(self, kinematics)
    
    def cartesian_trajectory_generator(self, wpts):
        """ Cartesian Trajectory Generator
        
        Args:
            wpts ([WayPoint]): List of WayPoint objects

        Returns:
            x[List]: ee x points 
            y[List]: ee y points 
            speeds[List]: speeds at each point 
        """
        num_steps = 15
        x_pts = []
        y_pts = []
        speeds = []

        for i in range(len(wpts) - 1):
            start_pt = wpts[i].point
            target_pt = wpts[i+1].point

            time = np.linspace(0.0, 1.0, num_steps)

            # Linear Bezier Curve
            # p(t) = (1 - t)*p_[0] + t*p_[1], t ∈ [0, 1]
            x = (1 - time) * start_pt[0] + time * target_pt[0]
            y = (1 - time) * start_pt[1] + time * target_pt[1]

            speed = [wpts[i+1].speed] * len(time)

            x_pts.extend(x)
            y_pts.extend(y)
            speeds.extend(speed)

        return x_pts, y_pts, speeds
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
                                    'example_kinematics_plugin.py': gen_test_kinematics_plugin(),
                                }
                            },
                        },
                        'files':{
                            '__init__.py': '',
                            'definition.py':init_definition_file('Kinematics_Plugins'),
                        }
                    },
                    'path_planners':{
                        'dirs':{
                            'plugins':{
                                'dirs':{},
                                'files':{
                                    '__init__.py': '',
                                    'example_path_planner_plugin.py': gen_test_path_planner_plugin(),
                                }
                            },
                        },
                        'files':{
                            '__init__.py': '',
                            'definition.py':init_definition_file('Path_Planners_Plugins'),
                        }
                    },
                    'trajectory_planners':{
                        'dirs':{
                            'plugins':{
                                'dirs':{},
                                'files':{
                                    '__init__.py': '',
                                    'example_traj_planner_plugin.py': gen_test_trajectory_planner_plugin(),
                                }
                            },
                        },
                        'files':{
                            '__init__.py': '',
                            'definition.py':init_definition_file('Trajectory_Planners_Plugins'),
                        }
                    },
                },
                'files':{
                    '__init__.py': '',
                }
            },
            'configs':{
                'dirs':{
                    'robots':{
                        'dirs':{},
                        'files':{
                            'example_robot.yml': gen_example_robot_yml(),
                        }
                    },
                    'worlds':{
                        'dirs':{},
                        'files':{
                            'example_world.yml': gen_example_world_yml(),
                        }
                    },
                    'launch':{
                        'dirs':{},
                        'files':{}
                    },
                },
                'files':{
                    '__init__.py': '',
                }
            },
            'scripts':{
                'dirs':{},
                'files':{
                    '__init__.py': '',
                    'robot_interface.py': robot_interface_file(),
                    'robot_application.py': gen_applcation_script(),
                }
            },
        },
        'files': {
            'README.md': '# My Workspace\n\nThis is a sample workspace.',
            'robotica_ws.txt': f'workspace:={ws_name}',
            '__init__.py': '',
            'setup.py': setup_file,
            'build_plugins.py': build_plugins_script(),
        }
    }

    # Create workspace and setup the environment
    create_ws(ws, ws_name)
    curr_path = os.getcwd()
    ws_path = os.path.join(curr_path, ws_name)
    add_workspace(ws_path)
    os.chdir(ws_path)
    subprocess.run(['pip', 'install', '-e', '.'])
    subprocess.run(['python', 'build_plugins.py'])


if __name__ == "__main__":
    # Define workspace details
    create_robotica_ws()