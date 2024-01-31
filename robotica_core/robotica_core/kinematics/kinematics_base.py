# Numpy (Array computing Lib.) [pip3 install numpy]
import numpy as np
from abc import abstractmethod

class KinematicsBase():
    def __init__(self, robot_model):
        self.DH_params  = robot_model.DH_params
        self.robot_name = robot_model.robot_name
        self.joint_limits = robot_model.joint_limits

        self.num_joints = len(self.DH_params.theta)

        # Translation Part -> p(x, y)
        self.p = np.zeros(self.num_joints)
        # Joints Rotation -> theta(theta_1, theta_2)
        self.theta = np.zeros(self.num_joints)
        # Jacobian Matrix (For the Jacobian Inverse Kinematics Calculation)
        self.jacobian_matrix = np.array(np.identity(self.num_joints))

        ## PRIVATE ##
        self.__p_target     = None
        self.__theta_target = np.zeros(self.num_joints)

    def compute_forward_kinematics(self):
        pass

    def compute_inverse_kinematics(self, point, cfg):
        pass

    def compute_differential_kinematics(self, p_target, theta, accuracy, num_of_iter):
        pass

    def forward_kinematics(self, joint_angles):
        """
        Description:
            Forward kinematics refers to the use of the kinematic equations of a robot to compute 
            the position of the end-effector from specified values for the joint parameters.
            Joint Angles (Theta_1, Theta_2) <-> Position of End-Effector (x, y)
            
        Args:
            (3) joint_angles [Float Array]: Joint angle of target in degrees or radians, depends on the variable.
            (4) deg [BOOL]: Representation of the input joint angle ('deg', 'rad').

        Examples:
            self.forward_kinematics_test([0.0, 1.57], deg=False)
        """
        assert len(joint_angles) == self.num_joints

        self.DH_params.theta = joint_angles

        # FK Computation
        self.p = self.compute_forward_kinematics()

        #TODO: CHECK AND VALIDATE DATA STRUCTURE OF self.p

        return self.p


    def inverse_kinematics(self, point, cfg):
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
        self.theta = self.compute_inverse_kinematics(point, cfg)
        return self.theta

    def differential_kinematics(self, p_target, theta, accuracy, num_of_iter):
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