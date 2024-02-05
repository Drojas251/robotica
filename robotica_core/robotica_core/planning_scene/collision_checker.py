import numpy as np
from robotica_core.simulation.physics import CollisionManager
from robotica_core.utils.yml_parser import SceneParamsLoader
from robotica_core.utils.yml_parser import RobotParamsLoader
from robotica_datatypes.kinematic_datatypes.DH_params import DH_parameters
from robotica_core.kinematics.tftree import TFTree


class CollisionChecker:
    def __init__(self, robot_yml_file, env_file=None):

        param_loader = RobotParamsLoader(robot_yml_file)
        theta, a, d, alpha = param_loader.load_dh_params()
        self.DH_params = DH_parameters(theta, a, d, alpha)

        self.tftree = TFTree(self.DH_params)
        self.collision_checker = CollisionManager(self.tftree.get_link_tfs(), self.tftree.DH_params.a)

        self._load_env_data(env_file)
        self.collision_checker.init_collision_managers()

    def check_collision(self, joint_angles):
        self.tftree.set_joints(joint_angles)
        link_tfs = self.tftree.get_link_tfs()
        self.collision_checker.update_robot_tfs(link_tfs)
        return self.collision_checker.check_collision()

    def _load_env_data(self, env_file):
        #Load Rectangle Objects
        if env_file == None:
            return
        
        scene_params = SceneParamsLoader(env_file)
        rectangle_objs = scene_params.load_rectangles()

        for name, rect_obj in rectangle_objs.items():
            pose_dict = rect_obj["pose"]
            size_dict = rect_obj["size"]

            x = pose_dict["x"]
            y = pose_dict["y"]
            pose = np.array([x, y, 0.0])

            size_x = size_dict["x"]
            size_y = size_dict["y"]

            self.collision_checker.env_collision_objs.add_box_collision_obj(name, size_x, size_y, pose)
