import numpy as np
import copy

class TFTree:
    def __init__(self, DH_params):
        self.DH_params = DH_params
        self.origin  = (0,0)
        self.num_joints = len(self.DH_params.theta)

    def set_joints(self, thetas):
        self.DH_params.theta = thetas

    def get_tree(self):
        tree = []
        tree.append(self.origin)

        for i in range(self.num_joints):
            A_i = self.get_base_transform(i+1)
            tree.append(self._get_translation(A_i))

        return tree

    def get_link_tfs(self):
        # HACK: Getting the tf of the center of each link
        link_tf_list = []

        for j in range(self.num_joints):
            transforms = []
            for i in range(j + 1):
                theta = self.DH_params.theta[i]
                a = self.DH_params.a[i]
                d = self.DH_params.d[i]
                alpha = self.DH_params.alpha[i]

                if i == j:
                    a = a/2
                A_i = self._transform(theta, a, d, alpha)
                transforms.append(A_i)

            A = [[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]]
            for Aj in transforms:
                A = np.matmul(A,Aj)

            link_tf_list.append(A)

        return link_tf_list        

    def _get_translation(self, A):
        return (A[0][3], A[1][3])

    def get_base_transform(self, joint_num):
        transforms = []
        for i in range(joint_num):
            theta = self.DH_params.theta[i]
            a = self.DH_params.a[i]
            d = self.DH_params.d[i]
            alpha = self.DH_params.alpha[i]
            A_i = self._transform(theta, a, d, alpha)
            transforms.append(A_i)

        A = [[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]]
        for Aj in transforms:
            A = np.matmul(A,Aj)
        return A

    def _transform(self, theta, a, d, alpha):
        def c(angle):
            return round(np.cos(angle),2)

        def s(angle):
            return round(np.sin(angle),2)

        A = [
            [c(theta), -s(theta)*c(alpha), s(theta)*s(alpha), a*c(theta)],
            [s(theta), c(theta)*c(alpha), -c(theta)*s(alpha), a*s(theta)],
            [0, s(alpha), c(alpha), d],
            [0, 0, 0, 1],
        ]

        return A
