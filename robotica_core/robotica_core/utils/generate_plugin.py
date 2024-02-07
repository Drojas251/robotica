import os
import re
import copy
import tkinter as tk
from tkinter import ttk
from tkinter import filedialog

class PluginGenerator:
    KINPLUGIN = "kinematics"
    PATHPLUGIN = "path_planners"
    TRAJPLUGIN = "trajectory_planners"

    def __init__(self, root):
        self.root = root
        self.plugin_path = None
        self.selected_plugin_type = None

    def run_ui(self):
        # Drop-down menu to select plugin type
        self.dropdown_var = tk.StringVar(self.root)
        dropdown_label = tk.Label(self.root, text="Select plugin type:")
        dropdown_label.grid(row=0, column=0, padx=10, pady=5)
        dropdown = ttk.Combobox(self.root, textvariable=self.dropdown_var)
        dropdown['values'] = (self.KINPLUGIN, self.PATHPLUGIN, self.TRAJPLUGIN)
        dropdown.grid(row=0, column=1, padx=10, pady=5)
        dropdown.bind("<<ComboboxSelected>>", self._select_option)

        # Input for class name
        class_name_label = tk.Label(self.root, text="Plugin Class Name: (e.g. MyPathPlanner)")
        class_name_label.grid(row=1, column=0, padx=10, pady=5)
        self.class_name_input = tk.Entry(self.root)
        self.class_name_input.grid(row=1, column=1, padx=10, pady=5)

        # Select ws directory
        ws_label = tk.Label(self.root, text="Select Robotica WS Dir:")
        ws_label.grid(row=3, column=0, padx=10, pady=5)
        self.select_ws_button = tk.Button(self.root, text="Click to Select Directory", command=self._select_ws_dir)
        self.select_ws_button.grid(row=3, column=1, columnspan=2, pady=10)

        self.dir_path = tk.Label(self.root, text="")
        self.dir_path.grid(row=4, column=1, padx=10, pady=5)
        self.selected_ws_label = tk.Label(self.root, text="Plugin Path")
        self.selected_ws_label.grid(row=4, column=0, padx=10, pady=5)

        # Generate plugin button
        self.generate_button = tk.Button(self.root, text="Generate Plugin", command=self._generate_output)
        self.generate_button.grid(row=5, column=0, columnspan=2, pady=10)

    def _generate_output(self):
        self._generate_plugin()
        self.root.destroy()

    def _select_option(self, event):
        self.selected_plugin_type = self.dropdown_var.get()

    def _find_plugin_dir(self, start_dir, target_dir):
        for root, dirs, files in os.walk(start_dir):
            if target_dir in dirs:
                return os.path.join(root, target_dir)
        return None

    def _select_ws_dir(self):
        directory_path = filedialog.askdirectory()
        if directory_path:
            plugin_type_path = self._find_plugin_dir(directory_path, self.selected_plugin_type)
            self.plugin_path = os.path.join(plugin_type_path, "plugins")
            self.dir_path.config(text=self.plugin_path)

    def _format_file_name(self, text):
        text = text.replace(' ', '_')
        if text.isupper():
            return text.lower()
        elif text.islower():
            return text
        elif re.search(r'[A-Z]', text) and re.search(r'[a-z]', text):
            text = re.sub(r'(?<!^)(?=[A-Z])', '_', text).lower()
            return text
        else:
            return None
    
    def _generate_plugin(self):
        # Get inputs
        class_name = self.class_name_input.get()   
        cls_name = copy.copy(class_name)
        file_name = self._format_file_name(cls_name)
        file_name = file_name + ".py"
        file_path = os.path.join(self.plugin_path, file_name)

        print("Plugin Type:", self.selected_plugin_type)
        print("Class Name:", class_name)
        print(f"{self.selected_plugin_type} plugin path:", file_path)
        
        if self.selected_plugin_type == self.KINPLUGIN:
            plugin_code = self._kinematic_plugin_template(class_name)
        elif self.selected_plugin_type == self.TRAJPLUGIN:
            plugin_code = self._traj_planning_plugin_template(class_name)
        elif self.selected_plugin_type == self.PATHPLUGIN:
            plugin_code = self._path_planning_plugin_template(class_name)
        else:
            print("Invalid plugin type")
            return

        self._generate_python_file(file_path, plugin_code)

    def _generate_python_file(self, plugin_file_name, code):
        with open(plugin_file_name, 'w') as f:
            f.write(code)  

    def _kinematic_plugin_template(self, plugin_class_name):
        return f'''
########################################################
# This is a generated template for a kinematics plugin
# The user must fill in the methods of this plugin class
########################################################

from robotica_plugins.kinematics.kinematics_plugin_interface import KinematicPluginInterface

class {plugin_class_name}(KinematicPluginInterface):
    def __init__(self, robot_model):
        KinematicPluginInterface.__init__(self, robot_model)

        ####################################################
        # The following attributes are available for use
        #
        # Attribute 1: self.DH_params (DH_parameters object)
        #   DH_parameters.theta     list of joint angles
        #   DH_parameters.a         list of link lengths
        #   DH_parameters.d         list of link offsets
        #   DH_parameters.alpha     list of link twists
        #
        #   Example:  
        #       joint_angle_1 = self.DH_params.theta[0]
        #       joint_angle_2 = self.DH_params.theta[1]
        #       link_length_2 = self.DH_params.a[1]
        #
        # Attribute 2: self.joint_limits (list of JointLimits objects)
        #   JointLimits.min_joint_pos       list of minimum joint positions
        #   JointLimits.max_joint_pos       list of maximum joint positions
        #   JointLimits.min_joint_velo      list of minimum joint velocities
        #   JointLimits.max_joint_velo      list of maximum joint velocities
        #
        #   Example:
        #       self.joint_limits[0].min_joint_pos
        #       self.joint_limits[1].max_joint_pos
        #
        ####################################################

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
        raise NotImplementedError("forward kinematics functionality is not implemented yet")


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
        raise NotImplementedError("inverse kinematics functionality is not implemented yet")
'''
    
    def _path_planning_plugin_template(self, plugin_class_name):
        return f'''
########################################################
# This is a generated template for a kinematics plugin
# The user must fill in the methods of this plugin class
########################################################

from robotica_plugins.path_planners.path_planning_plugin_interface import PathPlannerPluginInterface
from robotica_datatypes.path_datatypes.waypoint import WayPoint

class {plugin_class_name}(PathPlannerPluginInterface):
    def __init__(self, *args):
        PathPlannerPluginInterface.__init__(self, *args)

    def planner(self, start, goal):
        """ Define Planner Here

        Planner can be any planner of your choosing. 
        Planner must take in a start position and end position, 
        and return a list of WayPoint's. 

        Args: 
            start (tuple): (x, y)
            goal (tuple): (x, y)

        Return:
            path (List): [Waypoint]

        if path cannot be found
        
        Return:
            path: None
        """
        raise NotImplementedError("inverse kinematics functionality is not implemented yet")
'''
    
    def _traj_planning_plugin_template(self, plugin_class_name):
        return f'''
########################################################
# This is a generated template for a kinematics plugin
# The user must fill in the methods of this plugin class
########################################################

from robotica_plugins.trajectory_planners.trajectory_planning_interface import CartesianTrajectoryPluginInterface

class {plugin_class_name}(CartesianTrajectoryPluginInterface):
    def __init__(self, *args):
        CartesianTrajectoryPluginInterface.__init__(self, *args)

    def cartesian_trajectory_generator(self, wpts):
        """ Cartesian Trajectory Generator
        
        Args:
            wpts ([WayPoint]): List of WayPoint objects

        Returns:
            x[List]: ee x points 
            y[List]: ee y points 
            speeds[List]: speeds at each point 
        """
        raise NotImplementedError("inverse kinematics functionality is not implemented yet")
'''

def generate_plugin():
    # Create the main window
    root = tk.Tk()
    root.title("Input Generator")
    root.geometry("600x300")

    gen = PluginGenerator(root)
    gen.run_ui()

    # Run the Tkinter event loop
    root.mainloop()

if __name__ == "__main__":
    generate_plugin()
