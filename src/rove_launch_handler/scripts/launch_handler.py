#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
import signal
import subprocess
import os
from rove_launch_handler.srv import LaunchRequest, LaunchListRequest

launched_files = {}

class LaunchFile: 
    def __init__(self, package, file_name, pid):
        self.package = package
        self.file_name = file_name
        self.pid = pid

class LaunchMsg:
    def __init__(self):
        self.message = ""
        self.is_launched = False
        self.file_name = ""

def launch_file(package, file_name):
    command = f"ros2 launch {package} {file_name}"

    p = subprocess.Popen(command, shell=True, preexec_fn=os.setsid)

    launch_msg = LaunchMsg()
    # Sleep to make sure the launch command has time to fail if there's an error
    time.sleep(1)
    state = p.poll()
    launch_msg.file_name = file_name
    if state is None:
        launch_msg.message = f"{file_name} was launched"
        launched_files[file_name] = LaunchFile(package, file_name, p.pid)
        launch_msg.is_launched = True
    else:
        launch_msg.message = f"{file_name} was not launched"
        launch_msg.is_launched = False
    return launch_msg

def kill_launch_file(file_name):
    launch_msg = LaunchMsg()
    launch_msg.file_name = file_name
    if file_name in launched_files:
        pid = launched_files[file_name].pid
        try:
            os.killpg(os.getpgid(pid), signal.SIGINT)
            del launched_files[file_name]
            launch_msg.message = f"{file_name} was killed"
            launch_msg.is_launched = False
        except ProcessLookupError as e:
            launch_msg.message = f"Failed to kill {file_name}: {str(e)}"
            launch_msg.is_launched = False
    else:
        launch_msg.message = f"{file_name} was not launched"
        launch_msg.is_launched = False
    return launch_msg

def kill_all():
    for file_name in list(launched_files.keys()):
        kill_launch_file(file_name)

class LaunchHandlerService(Node):
    def __init__(self):
        super().__init__('launch_handler_service')
        self.srv_launch = self.create_service(LaunchRequest, 'launchHandler/launchFile', self.launch_callback)
        self.srv_list = self.create_service(LaunchListRequest, 'launchHandler/getAllLaunchedFiles', self.get_launched_files)
        self.get_logger().info("LaunchHandlerService node has been started.")

    def launch_callback(self, request, response):
        package = request.package
        file_name = request.file_name
        if file_name not in launched_files:
            launch_msg = launch_file(package, file_name)
        else:
            launch_msg = kill_launch_file(file_name)
        response.message = launch_msg.message
        response.is_launched = launch_msg.is_launched
        response.file_name = launch_msg.file_name
        return response

    def get_launched_files(self, request, response):
        response.packages = [lf.package for lf in launched_files.values()]
        response.files = list(launched_files.keys())
        return response

def main(args=None):
    rclpy.init(args=args)
    node = LaunchHandlerService()
    rclpy.get_default_context().on_shutdown(kill_all)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
