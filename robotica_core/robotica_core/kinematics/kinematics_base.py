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
    def display_workspace(self, display_type = 0):
        """
        Description:
            Display the work envelope (workspace) in the environment.

        Args:
            (1) display_type [INT]: Work envelope visualization options (0: Line, 1: Points).

        Examples:
            self.__display_workspace(0)
        """

        # Generate linearly spaced vectors for the each of joints.
        theta_1 = np.linspace(self.ax_wr[0], self.ax_wr[1], 100)
        theta_2 = np.linspace(self.ax_wr[2], self.ax_wr[3], 100)

        # Return coordinate matrices from coordinate vectors.
        [theta_1_mg, theta_2_mg] = np.meshgrid(theta_1, theta_2)

        # Find the points x, y in the workspace using the equations FK.
        x_p = (self.rDH_param.a[0]*np.cos(theta_1_mg) + self.rDH_param.a[1]*np.cos(theta_1_mg + theta_2_mg))
        y_p = (self.rDH_param.a[0]*np.sin(theta_1_mg) + self.rDH_param.a[1]*np.sin(theta_1_mg + theta_2_mg))

        if display_type == 0:
            x_pN = []
            y_pN = []

            # Inner Circle -> Part 1
            x, y = self._generate_workspace_curve([self.ax_wr[1], self.ax_wr[0]], [1, self.ax_wr[2]], -1)
            x_pN.append(x)
            y_pN.append(y)
            self.plt.plot(x_pN[0], y_pN[0], '-', c=[0,1,0,0.5], linewidth=5)
            # Inner Circle -> Part 2
            x, y = self._generate_workspace_curve([self.ax_wr[1], self.ax_wr[0]], [1, self.ax_wr[3]], -1)
            x_pN.append(x)
            y_pN.append(y)
            self.plt.plot(x_pN[1], y_pN[1], '-', c=[0,1,0,0.5], linewidth=5)
            # Outer Curve -> Part 1
            x, y = self._generate_workspace_curve([self.ax_wr[3], 0.0], [0, self.ax_wr[1]], -1)
            x_pN.append(x)
            y_pN.append(y)
            self.plt.plot(x_pN[2], y_pN[2], '-', c=[0,1,0,0.5], linewidth=5)
            # Outer Circle
            x, y = self._generate_workspace_curve([self.ax_wr[1], self.ax_wr[0]], [1, 0.0], -1)
            x_pN.append(x)
            y_pN.append(y)
            self.plt.plot(x_pN[3], y_pN[3], '-', c=[0,1,0,0.5], linewidth=5)
            # Outer Curve -> Part 2
            x, y = self._generate_workspace_curve([0.0, self.ax_wr[2]], [0, self.ax_wr[0]], -1)
            x_pN.append(x)
            y_pN.append(y)
            self.plt.plot(x_pN[4], y_pN[4], '-', c=[0,1,0,0.5], linewidth=5)
            
            self.plt.plot(x_p[0][0], y_p[0][0],'.', label=u"Work Envelop", c=[0,1,0,0.5])

        elif display_type == 1:
            self.plt.plot(x_p, y_p,'o', c=[0,1,0,0.1])
            self.plt.plot(x_p[0][0],y_p[0][0], '.', label=u"Work Envelop", c=[0,1,0,0.5])

    def _generate_workspace_curve(self, limit, fk_param, increment):
        """
        Description:
           Simple function to generate a workspace curve.

        Args:
            (1) limit [Float Array]: Maximum and minimum limit of the curve.
            (2) fk_param [INT, Float Array]: Dynamic parameter for points from the FK calculation and the index of the current parameter.
            (3) increment [INT]: Number of increments and direction.

        Returns:
            (1 - 2) parameter{1}, parameter{2} [Float Array]: Results of path values.
        """

        x = []
        y = []

        for i in range(int(limit[0] * (180/np.pi)), int(limit[1] * (180/np.pi)), increment):
            if fk_param[0] == 0:
                self.forward_kinematics(1, [fk_param[1], i * (np.pi/180)], 'rad')
            elif fk_param[0] == 1:
                self.forward_kinematics(1, [i * (np.pi/180), fk_param[1]], 'rad')

            x.append(self.p[0])
            y.append(self.p[1])

        return x, y