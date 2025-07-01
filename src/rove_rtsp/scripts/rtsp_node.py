#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import subprocess
import threading
import os
import signal


class RtspNode(Node):
    def __init__(self):
        super().__init__('rtsp_node')
        self.get_logger().info("Launching 'rovertsp'...")

        try:
            self.process = subprocess.Popen(
                ['rovertsp'],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                preexec_fn=os.setsid  # Start the process in a new process group
            )

            self.stdout_thread = threading.Thread(target=self._read_stream, args=(self.process.stdout, "stdout"))
            self.stderr_thread = threading.Thread(target=self._read_stream, args=(self.process.stderr, "stderr"))
            self.stdout_thread.start()
            self.stderr_thread.start()

        except Exception as e:
            self.get_logger().error(f"Failed to launch 'rovertsp': {e}")
            self.process = None

    def _read_stream(self, stream, label):
        for line in iter(stream.readline, ''):
            if line:
                self.get_logger().info(f"[rovertsp/{label}]: {line.strip()}")
        stream.close()

    def destroy_node(self):
        self.get_logger().info("Shutting down 'rovertsp'...")
        if self.process and self.process.poll() is None:
            try:
                os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
                self.process.wait(timeout=5)
            except Exception as e:
                self.get_logger().error(f"Error during shutdown: {e}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RtspNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
