from robotica_core.kinematics.kinematics_base import KinematicsBase
from abc import abstractmethod
import numpy as np



class KinematicPluginInterface(KinematicsBase):
    def __init__(self, robot_model):
        self.DH_params  = robot_model.DH_params
        self.robot_name = robot_model.robot_name
        self.joint_limits = robot_model.joint_limits
        
        KinematicsBase.__init__(self, robot_model)


    @abstractmethod
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
        pass

    @abstractmethod
    def compute_inverse_kinematics(self, point, cfg):
        """
        Description:
            Inverse kinematics is the mathematical process of calculating the variable 
            joint parameters needed to place the end of a kinematic chain.
            Position of End-Effector (x, y) <-> Joint Angles (Theta_1, Theta_2)
            
        Args:
            (1) point [Float Array]: Position (x, y) of the target in meters.
            (2) cfg [INT]: Robot configuration (IK Multiple Solutions).

        Examples:
            self.inverse_kinematics([0.45, 0.10], 0)
        """
        pass

    @abstractmethod
    def compute_differential_kinematics(self, p_target, theta, accuracy, num_of_iter):
        """
        Description:
            The Jacobian matrix method is an incremental method of inverse kinematics 
            (the motion required to move a limb to a certain position may be performed over several frames). 
        Args:
            (1) p_target [Float Array]: Position (x, y) of the target in meters.
            (2) theta [Float Array]: Joint angle of target in radians.
            (3) accuracy [Float]: Accuracy of inverse kinematics calculation.
            (4) num_of_iter [INT]: Number of iterations of the calculation.

        Examples:
            self.differential_kinematics([0.35, 0.15], [0.0, 0.0], 0.0001, 10000)
        """
        pass