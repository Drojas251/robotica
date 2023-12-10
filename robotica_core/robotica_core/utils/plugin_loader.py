

import os
import ast


class PluginLoader:
    MODULE = "robotica_plugins"
    PLUGIN_DIR = "plugins"
    PLUGIN_DEF = "definition.py"
    def __init__(self, robotica_plugin_dir, group_name):
        """
        Args:
            robotica_plugin_dir (str): Directory name of specific plugin type
            group_name (str): Plugin in group name
        """
        self.plugin_dir_path = os.environ["ROBOTICA_PLUGIN_PATH"]

        self.robotica_plugin_dir = robotica_plugin_dir
        self.group_name = group_name

        self.full_path = self.plugin_dir_path + "/" + robotica_plugin_dir + "/" + self.PLUGIN_DIR
        self.def_file_path = self.plugin_dir_path + "/" + robotica_plugin_dir + "/" + self.PLUGIN_DEF

        self.imports = []
        self.cls_names = []

        self.generate_definition_file()

    def generate_definition_file(self):

        print(f"\nBuilding Plugins in {self.full_path}")
        self._extract_plugin_info()

        file_content = ""
        for class_name, import_statement in zip(self.cls_names, self.imports):
            file_content += f"{class_name}.__name__: {class_name},\n"
        
        # Write the content to a Python file
        with open(f"{self.def_file_path}", "w") as file:
            file.write("\n".join(self.imports))
            file.write("\n\n")
            file.write(self.group_name + " = {\n")
            file.write(file_content)
            file.write("}\n")

        if len(self.cls_names) == 0:
            print("No Plugins Found")
            return

        print("Successfully Built Plugins:")
        for plugin in self.cls_names:
            print(f"    {plugin}")

    def _extract_plugin_info(self):
        for filename in os.listdir(self.full_path):
            if filename.endswith(".py"):
                file_path = os.path.join(self.full_path, filename)
                f_path = filename[:-3]
                
                classes = self._extract_class_names(file_path)
                if classes:
                    for cls in classes:
                        # Check that they inherit from base class
                        impt = f"from {self.MODULE}.{self.robotica_plugin_dir}.{self.PLUGIN_DIR}.{f_path} import {cls}"
                        self.imports.append(impt)
                        self.cls_names.append(cls)

    def _extract_class_names(self, file_path):
        with open(file_path, "r") as file:
            tree = ast.parse(file.read(), filename=file_path)

        class_names = [node.name for node in ast.walk(tree) if isinstance(node, ast.ClassDef)]
        return class_names
    
    