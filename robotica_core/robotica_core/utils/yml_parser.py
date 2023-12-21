import yaml

class ParseYML:
    def __init__(self, yml_path):
        self.yml_path = yml_path

    def _read_yml_file(self):
        with open(self.yml_path, 'r') as file:
            data = yaml.safe_load(file)
        return data

class RobotParamsLoader(ParseYML):
    def __init__(self, params_yml_path):
        super().__init__(params_yml_path)

    def load_dh_params(self):
        data = self._read_yml_file()
        dh_theta = data['dh_params']['theta']
        dh_a = data['dh_params']['a']
        dh_d = data['dh_params']['d']
        dh_alpha = data['dh_params']['alpha']
        return dh_theta, dh_a, dh_d, dh_alpha

    def load_robot_name(self):
        data = self._read_yml_file()
        return data['robot_name']

    def load_joint_limits(self):
        data = self._read_yml_file()
        joint_limits = data['joint_limits']
        return joint_limits

    def load_kinematics_class(self):
        data = self._read_yml_file()
        return data['kinematics_class_plugin']

    def load_trjectory_planner_class(self):
        data = self._read_yml_file()
        return data['trajectory_planner_class_plugin']
    

class NetworkingParams(ParseYML):
    def __init__(self):
        yml_path = "/home/drojas/robot_arm/robotica/robotica_core/robotica_core/endpoints.yml"
        super().__init__(yml_path)

    def get_pub_sub_info(self, topic_name):
        data = self._read_yml_file()
        pub_sub = data["pub_sub"]
        channel = pub_sub[topic_name]
        return channel["topic"], channel["port"]
    
    def get_serv_req_info(self, serv_name):
        data = self._read_yml_file()
        serv_req = data["service"]
        channel = serv_req[serv_name]
        return channel["port"]

class SceneParamsLoader(ParseYML):
    def __init__(self, params_yml_path):
        super().__init__(params_yml_path)

    def load_rectangles(self):
        data = self._read_yml_file()
        objs = data.get("objects", None)
        if objs == None:
            print("No objects loaded in the scene")
            return None
        
        rectangles = []
        for obj in objs.values():
            obj_type = obj.get("type")
            if obj_type == "rectangle":
                rectangles.append(obj)

        return rectangles



