import numpy as np
from robotica_core.simulation.physics import CollisionManager
from robotica_core.simulation.rendering import Visualization
from robotica_core.kinematics.tftree import TFTree
from robotica_core.utils.yml_parser import SceneParamsLoader


class SimCore:
    def __init__(self, robot_model, env_yml_file):
        self._rectangle_objs = {}
        self.scene_params = SceneParamsLoader(env_yml_file)
        self.robot_model = robot_model
        self.tftree = TFTree(self.robot_model.DH_params)
        self.vis_scene = Visualization(self.robot_model, self.tftree)
        self.collision_checker = CollisionManager(self.tftree.get_link_tfs(), self.robot_model.DH_params.a)
        
        # Load Collision Checking Data
        self.collision_checker.add_collision_callback(self._collision_callback)
        self._load_env_data()
        self.collision_checker.init_collision_managers()

        self._env_yml_file = env_yml_file

    def update_tf_tree(self, joint_angles):
        self.tftree.set_joints(joint_angles)

    def render(self):
        self.vis_scene.visualize_robot()

    def reset_env(self, env_yml_file=None):
        self._clear_env_data()
        self._reload_env_params()
        self._load_env_data()

    def collisions_detected(self):
        link_tfs = self.tftree.get_link_tfs()
        self.collision_checker.update_robot_tfs(link_tfs)
        return self.collision_checker.check_collision()

    def _collision_callback(self, env_obj, robot_link):
        self.vis_scene.vis_collision(env_obj)

    def _load_env_data(self):
        #Load Rectangle Objects
        self._rectangle_objs = self.scene_params.load_rectangles()

        for name, rect_obj in self._rectangle_objs.items():
            print(f"adding: {name}")
            pose_dict = rect_obj["pose"]
            size_dict = rect_obj["size"]

            x = pose_dict["x"]
            y = pose_dict["y"]
            pose = np.array([x, y, 0.0])

            size_x = size_dict["x"]
            size_y = size_dict["y"]

            self.collision_checker.env_collision_objs.add_box_collision_obj(name, size_x, size_y, pose)
            self.vis_scene.add_rectangle_obj(name, (x - 0.5*size_x, y - 0.5*size_y), (size_x, size_y))

    def _clear_env_data(self):
        #Revoming Rectangle Objects
        for name, rect_obj in self._rectangle_objs.items():
            print(f"deleting: {name}")
            self.vis_scene.remove_rectangle_obj(name)
            self.collision_checker.remove_env_collision_obj(name)

    def _reload_env_params(self):
        self.scene_params = SceneParamsLoader(self._env_yml_file)



