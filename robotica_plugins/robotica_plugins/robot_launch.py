from robotica_core.kinematics.robot_model import RobotModel
from robotica_core.utils.yml_parser import load_kinematics_class
from robotica_plugins.kinematics.definition import Kinematics_Plugins
from robotica_core.simulation.joint_publisher import publish_joint_data
from robotica_core.trajectory_planning.trajectory import Trajectory


class Robot:
    def __init__(self, robot_yml_file):
        self.robot_model = RobotModel(robot_yml_file)

        kinematics_cls_name = load_kinematics_class(robot_yml_file)
        kinematics_cls = Kinematics_Plugins[kinematics_cls_name]
        self.kinematics = kinematics_cls(self.robot_model)

    def joint_move(self, joint_trajectory):

        cartesian_traj = []
        for joint in joint_trajectory:
            ee_point = self.kinematics.forward_kinematics(joint)
            cartesian_traj.append(ee_point)

        print(cartesian_traj)

        trajectory = Trajectory(joint_trajectory, cartesian_traj)
        publish_joint_data(trajectory)

    def cartesian_move(self, cartesian_trajectory):

        joint_traj = []
        for point in cartesian_trajectory:
            joint_pos = self.kinematics.inverse_kinematics(point, 0)
            joint_traj.append(joint_pos)

        print(joint_traj)

        
        #publish_joint_data(trajectory)