import numpy as np
from robotica_core.simulation.physics import CollisionManager
from robotica_core.simulation.rendering import Visualization
from robotica_core.kinematics.tftree import TFTree
from robotica_core.utils.yml_parser import SceneParamsLoader




class SimCore:
    def __init__(self, DH_params, env_yml_file):
        self.scene_params = SceneParamsLoader(env_yml_file)
        self.tftree = TFTree(DH_params)
        self.vis_scene = Visualization(DH_params, self.tftree)
        self.collision_checker = CollisionManager(self.tftree.get_link_tfs(), DH_params.a)
        
        # Load Collision Checking Data
        self._load_env_data()
        self.collision_checker.init_collision_managers()

    def update_tf_tree(self, joint_angles):
        self.tftree.set_joints(joint_angles)

    def render(self):
        self.vis_scene.visualize_arm()

    def check_for_collisions(self):
        link_tfs = self.tftree.get_link_tfs()
        self.collision_checker.update_robot_tfs(link_tfs)
        self.collision_checker.check_collision()

    def _load_env_data(self):
        #Load Rectangle Objects
        rectangle_objs = self.scene_params.load_rectangles()

        for rect_obj in rectangle_objs:
            pose_dict = rect_obj["pose"]
            size_dict = rect_obj["size"]

            x = pose_dict["x"]
            y = pose_dict["y"]
            pose = np.array([x, y, 0.0])

            size_x = size_dict["x"]
            size_y = size_dict["y"]

            self.collision_checker.env_collision_objs.add_box_collision_obj(size_x, size_y, pose)
            self.vis_scene.add_rectangle_obj((x - 0.5*size_x, y - 0.5*size_y), (size_x, size_y))


