# PYTHON_ARGCOMPLETE_OK

import json
import os
import argparse
import argcomplete

from robotica_core.utils.workspace_manager import find_workspaces

class RoboticaLaunch:
    def __init__(self):
        self.ws_name_list, self.ws_dict = self.map_workspace()
        
    def map_workspace(self):
        active_ws = find_workspaces()
        ws_name_list = []
        ws_dict = {}
        for ws_path in active_ws:
            ws_name = os.path.split(ws_path)[-1]
            ws_name_list.append(ws_name)

            ws_dict[ws_name] = ws_path

        return ws_name_list, ws_dict
    
    def file_completer(self, prefix, parsed_args, **kwargs):
        # Get the selected workspace from parsed arguments
        workspace = parsed_args.workspace

        # Get the list of files in the selected workspace
        workspace_path = self.ws_dict[workspace]
        launch_dir = os.path.join(workspace_path, "configs/launch")
        files = os.listdir(launch_dir)

        # Filter files based on the prefix
        matching_files = [file for file in files if file.startswith(prefix)]

        return matching_files

    
    def get_launch_file(self):
        parser = argparse.ArgumentParser(description='Example script with arguments')

        # Add the "workspace" argument
        parser.add_argument('workspace', choices=self.ws_name_list, help='Choose a workspace')

        # Add the "file" argument with autocompletion based on the files in the selected workspace
        parser.add_argument('file', help='Choose a file in the workspace').completer = self.file_completer

        # Enable autocompletion
        argcomplete.autocomplete(parser)

        # Parse the arguments again
        args = parser.parse_args()

        ws_path = self.ws_dict[args.workspace]
        launch_file = os.path.join(ws_path, "configs/launch", args.file)

        # Open the JSON file
        with open(launch_file, 'r') as json_file:
            # Load the JSON data
            data = json.load(json_file)

        robot_file = data.get("robot", None)
        world_file = data.get("world", None)

        if robot_file is None:
            raise Exception("Robot file not found in the launch file")
        
        robot_file_path = os.path.join(ws_path, "configs/robots",  robot_file)
        
        if world_file is None:
            print("World file not found in the launch file")
            world_file_path = None

        else:
            world_file_path = os.path.join(ws_path, "configs/worlds", world_file)

        return robot_file_path, world_file_path
