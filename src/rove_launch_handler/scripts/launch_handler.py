#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
import signal
import subprocess
import os
from rove_launch_handler.srv import LaunchRequest, LaunchListRequest

launchedFiles = dict()

class LaunchFile: 
    def __init__(self, package, fileName, pid):
        self.package = package
        self.fileName = fileName
        self.pid = pid

class LaunchMsg:
    def __init__(self):
        self.message = ""
        self.isLaunched = False
        self.fileName = ""

def launchFile(package, fileName):
    command = "ros2 launch {0} {1}".format(package, fileName)

    p = subprocess.Popen(command, shell=True, preexec_fn=os.setsid)

    launchMsg = LaunchMsg()
    # Sleep to make sure the launch command has time to fail if there's an error
    time.sleep(1)
    state = p.poll()
    launchMsg.fileName = fileName
    if state is None:
        launchMsg.message = fileName + " was launched"
        launchedFiles[fileName] = LaunchFile(package, fileName, p.pid)
        launchMsg.isLaunched = True
    else:
        launchMsg.message = fileName + " was not launched"
        launchMsg.isLaunched = False
    return launchMsg

def killLaunchFile(fileName):
    launchMsg = LaunchMsg()
    launchMsg.fileName = fileName
    if fileName in launchedFiles:
        pid = launchedFiles[fileName].pid
        os.killpg(os.getpgid(pid), signal.SIGINT)
        del launchedFiles[fileName]
        launchMsg.message = fileName + " was killed"
        launchMsg.isLaunched = False
    else:
        launchMsg.message = fileName + " was not launched"
        launchMsg.isLaunched = False
    return launchMsg

def killAll():
    for launchFile in launchedFiles.values():
        killLaunchFile(launchFile.fileName)

class LaunchHandlerService(Node):
    def __init__(self):
        super().__init__('launch_handler_service')
        self.srv_launch = self.create_service(LaunchRequest, 'launchHandler/launchFile', self.launch_callback)
        self.srv_list = self.create_service(LaunchListRequest, 'launchHandler/getAllLaunchedFiles', self.get_launched_files)
        self.get_logger().info("LaunchHandlerService node has been started.")

    def launch_callback(self, request, response):
        package = request.package
        fileName = request.file_name
        # Check if the launch file is already running
        if fileName not in launchedFiles:
            launchMsg = launchFile(package, fileName)
        else:
            launchMsg = killLaunchFile(fileName)
        response.message = launchMsg.message
        response.is_launched = launchMsg.isLaunched
        response.file_name = launchMsg.fileName
        return response

    def get_launched_files(self, request, response):
        # Get array of package names
        packageNames = []
        for launchedFile in launchedFiles.values(): 
            packageNames.append(launchedFile.package)
        
        response.packages = packageNames
        response.files = list(launchedFiles.keys())
        return response

def main(args=None):
    rclpy.init(args=args)
    node = LaunchHandlerService()
    rclpy.get_default_context().on_shutdown(killAll)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
