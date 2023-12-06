import yaml

def load_dh_params(yml_path):
    with open(yml_path, 'r') as file:
        data = yaml.safe_load(file)

    dh_theta = data['dh_params']['theta']
    dh_a = data['dh_params']['a']
    dh_d = data['dh_params']['d']
    dh_alpha = data['dh_params']['alpha']

    return dh_theta, dh_a, dh_d, dh_alpha

def load_robot_name(yml_path):
    with open(yml_path, 'r') as file:
        data = yaml.safe_load(file)
    return data['robot_name']

def load_joint_limits(yml_path):
    with open(yml_path, 'r') as file:
        data = yaml.safe_load(file)
    joint_limits = data['joint_limits']
    return joint_limits

def load_kinematics_class(yml_path):
    with open(yml_path, 'r') as file:
        data = yaml.safe_load(file)
    return data['kinematics_class_plugin']

def load_trjectory_planner_class(yml_path):
    with open(yml_path, 'r') as file:
        data = yaml.safe_load(file)
    return data['trajectory_planner_class_plugin']
