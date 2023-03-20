import rospkg
import os
from rospkg import RosPack as rp
import pathlib
import yaml
import subprocess


param_dirs = ["config", "param", "params", "launch"]

"""
Get current workspace from ROS_PACKAGE_PATH env variable
requires user confirmation
The way to judge where there is a workspace: 
there is an item in ROS_PACKAGE_PATH that contains "src"

return: string of workspace name
"""


def get_workspace():
    ROS_PACKAGE_PATH = os.environ.get('ROS_PACKAGE_PATH')
    if "src" in ROS_PACKAGE_PATH:
        paths = ROS_PACKAGE_PATH.split(":")
        for path in paths:
            if "src" in path:
                ws_name = os.path.basename(os.path.dirname(path))
                while True:
                    confim = input(
                        "Is {} your workspace?(y/n)\n ".format(ws_name)).strip().lower()
                    if confim == "y":
                        return path
                    elif confim == "n":
                        print("Consider re-sourcing your workspace.\n")
                        break
                    else:
                        print("Unrecognizable input.\n")
    else:
        print("No workspace found. Consider re-sourcing your workspace.\n")


"""
Get all the subdirectories in "dirname"
without all the subdirectories start with "."

return: list of all subfolders
"""


def scan_dir(dirname):
    subfolders = [f.path for f in os.scandir(dirname) if f.is_dir()]
    for dirname in list(subfolders):
        if pathlib.PurePath(dirname).name.startswith("."):
            continue
        if "launch" in os.listdir(dirname):
            subfolders.append(dirname)
            continue
        if os.path.isdir(dirname) and len(os.listdir(dirname)) == 0:
            continue
        subfolders.extend(scan_dir(dirname))
    return subfolders


def choice(list, item_name):
    for i, item in enumerate(list):
        print(f'{i}: {item}')
    choice = int(input("\nChoose {} by number: ".format(item_name)))
    selected_item = list[choice]
    return selected_item


"""
Let user choose package to be configured
Get all the packages that can be configured in workspace
by match the result of all ros packages 
and all the subdirctory names of ws_path

return: list of packages
"""


def get_pkg_list(ws_path):
    pkg_list = set()
    dir_list = scan_dir(ws_path)
    dirs = set(os.path.basename(dir) for dir in dir_list)
    for pkg in rp().list():
        if pkg in dirs:
            sub_dirs = {os.path.basename(f.path) for f in os.scandir(
                rp().get_path(pkg)) if f.is_dir()}
            if sub_dirs & set(param_dirs):
                pkg_list.add(pkg)
    result_list = sorted(pkg_list)
    return result_list


"""
If the subdirectory name contains any of the items in the param_dirs list
it is considered to contain param

return: list of param paths
"""


def get_param_list(pkg_path):
    param_list = []
    for param_dir in param_dirs:
        full_dir = os.path.join(pkg_path, param_dir)
        if os.path.isdir(full_dir):
            param_list.extend(param_dir + param_dir.join(f.path.split(param_dir)[1:]) for f in os.scandir(
                full_dir) if f.is_file())
    return param_list


def find_key(dict_obj, target_key):
    """
    在多层字典中查找某个键
    :param dict_obj: 待查找的字典
    :param target_key: 目标键名
    :return: 目标键的值，如果不存在返回 None
    """
    if target_key in dict_obj:
        return dict_obj[target_key]
    for key, value in dict_obj.items():
        if isinstance(value, dict):
            result = find_key(value, target_key)
            print(result)
            if result is not None:
                return result
    return None


def process_yaml(file_path):
    param_name = input("Input param name: ")
    with open(file_path, 'r') as f:
        data = yaml.safe_load(f)
    value = find_key(data, param_name)
    if value:
        print('Value of {}:'.format(param_name), value)    
    value = input("Enter value for '{}': ".format(param_name))
    data[param_name] = value
    with open(file_path, "w") as f:
        yaml.safe_dump(data, f)
        
        
def process_file(file_path):
    # subprocess.run(['nano', file_path])
    with open(file_path, 'w+') as f:
    
        subprocess.call(['nano', file_path])

        # get the edited content
        f.seek(0)
        content = f.read()

        # find the line that was edited
        lines = content.split('\n')
        for i, line in enumerate(lines):
            if 'edited' in line:
                print(f'The user edited line {i+1}: {line}')

if __name__ == '__main__':
    ws_path = get_workspace()
    if (ws_path):
        print(ws_path)
    selected_pkg = choice(get_pkg_list(ws_path), "pkg")
    pkg_path = rp().get_path(selected_pkg)
    param_list = get_param_list(pkg_path)
    param_path = (str(pkg_path) + "/" + choice(param_list, "param"))
    # process_yaml(param_path)
    process_file(param_path)
