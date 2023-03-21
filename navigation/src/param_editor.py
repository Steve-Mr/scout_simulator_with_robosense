import os
from rospkg import RosPack as rp
import pathlib
import subprocess
import difflib
from datetime import datetime
import git


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
    while(1):
        choice = int(input("\nChoose {} by number: ".format(item_name)))
        if choice in range(len(list)):
            selected_item = list[choice]
            break
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


def logger(ws_path, logger_name, content):
    with open(os.path.join(ws_path, logger_name), 'a+') as f:
        f.write(content)

def auto_commit(ws_path):

    repo = git.Repo(ws_path)

    if repo.is_dirty():

        # Get the list of changed files and use it as the commit message
        changed_files = [item.a_path for item in repo.index.diff(None)]
        commit_message = "\n".join(changed_files)

        # Check if there are any changes in the submodules
        for submodule in repo.submodules:

            # Check if the submodule exists and is dirty
            if submodule.module_exists() and submodule.module().is_dirty():

                # Create a Git Repo object for the submodule
                submodule_repo = git.Repo(submodule.abspath)

                # Add all changes to the staging area

                # get the list of modified files in the submodule
                modified_files = [
                    item.a_path for item in submodule_repo.index.diff(None)]
                commit_message += "\n" + "\n".join(modified_files)

                submodule_commit_message = f"Auto commit: {submodule.name}: {', '.join(modified_files)}"
                submodule_repo.git.add(update=True)

                submodule_repo.git.commit(m=submodule_commit_message)

        repo.git.add(update=True)

        # Commit the changes with the list of changed files as the commit message
        repo.git.commit(m=commit_message)

    print("Finished.")


def process_file(file_path):
    with open(file_path, 'r') as f:
        content_before = f.readlines()

    # 调用 nano 编辑文件
    subprocess.call(['nano', file_path])

    # 读取修改后的文件内容
    with open(file_path, 'r') as f:
        content_after = f.readlines()

    return difflib.unified_diff(
        content_before, content_after,
        fromfile=file_path, tofile=file_path, n=0,
        fromfiledate=datetime.now().strftime("%Y-%m-%d %H:%M:%S"), tofiledate=datetime.now().strftime("%Y-%m-%d %H:%M:%S"))


if __name__ == '__main__':
    log_name = "changelog"
    ws_path = get_workspace()
    if (ws_path):
        print(ws_path)
    selected_pkg = choice(get_pkg_list(ws_path), "pkg")
    pkg_path = rp().get_path(selected_pkg)
    param_list = get_param_list(pkg_path)
    param_path = (str(pkg_path) + "/" + choice(param_list, "param"))
    for line in process_file(param_path):
        logger(ws_path, log_name, line)
    logger(ws_path, log_name, "\n\n")
    auto_commit(ws_path)
