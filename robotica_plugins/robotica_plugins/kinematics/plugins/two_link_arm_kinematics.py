import numpy as np
from robotica_plugins.kinematics.kinematics_plugin_interface import KinematicPluginInterface



class TwoLinkArmKinematics(KinematicPluginInterface):
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