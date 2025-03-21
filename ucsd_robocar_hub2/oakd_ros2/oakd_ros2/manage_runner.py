import rclpy
from rclpy.node import Node
import subprocess
import os

class ManagePyRunner(Node):
    def __init__(self):
        super().__init__('manage_py_runner')
        self.get_logger().info("Starting manage.py execution...")

        # Path to manage.py (mounted inside Docker)
        manage_script = "/home/jetson/projects/mycar/manage.py"  # CHANGE THIS to the correct path

        # Command to run it using the correct Python environment
        command = f"/home/jetson/projects/envs/donkey/bin/python3 {manage_script}"

        try:
            process = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

            # Read and print output in real-time
            for line in process.stdout:
                self.get_logger().info(line.decode().strip())

            # Wait for process to complete
            process.wait()

            if process.returncode == 0:
                self.get_logger().info("manage.py executed successfully.")
            else:
                self.get_logger().error(f"Error running manage.py: {process.stderr.read().decode().strip()}")

        except Exception as e:
            self.get_logger().error(f"Failed to execute manage.py: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = ManagePyRunner()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down manage.py runner...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
