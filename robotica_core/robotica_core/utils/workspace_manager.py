import os
from robotica_core.utils.yml_parser import ParseYML

class WorkSpacesManager(ParseYML):
    def __init__(self):
        home_dir = os.path.expanduser("~")
        active_ws_file = os.path.join(home_dir, ".robotica/workspaces/active_ws.yml")
        super().__init__(active_ws_file)

    def get_active_ws(self):
        data = self._read_yml_file()
        return data["workspaces"]
    
    def add_ws(self, ws_path):
        data = self._read_yml_file()
        if ws_path not in data["workspaces"]:
            data["workspaces"].append(ws_path)
            self._write_yml_file(data)
    
    def remove_ws(self, ws_name):
        data = self._read_yml_file()
        if ws_name in data["workspaces"]:
            data["workspaces"].remove(ws_name)
            self._write_yml_file(data)

############################
#Funcitons available to use:
############################    
                    
def find_workspaces():
    ws_manager = WorkSpacesManager()
    active_ws = ws_manager.get_active_ws()

    for ws in active_ws:
        if os.path.exists(ws):
            continue
        else:
            ws_manager.remove_ws(ws)
    return active_ws

def add_workspace(ws_path):
    ws_manager = WorkSpacesManager()
    if os.path.exists(ws_path):
        ws_manager.add_ws(ws_path)
    else:
        print(f"Path {ws_path} does not exist")

