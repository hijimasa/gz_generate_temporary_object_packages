import sys
import os
import time
import re
import numpy as np
import queue
import rclpy
import subprocess
import threading
from ros_gz_interfaces.srv import SpawnEntity
from geometry_msgs.msg import PoseStamped

from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory
import xacro

REMAIN_TIME_SEC = 5.0

def euler_from_quaternion(quaternion):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw
    
def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion (w in last place)
    quat = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0] * 4
    q[0] = cy * cp * cr + sy * sp * sr
    q[1] = cy * cp * sr - sy * sp * cr
    q[2] = sy * cp * sr + cy * sp * cr
    q[3] = sy * cp * cr - cy * sp * sr

    return q

flying_disc_description_path = os.path.join(
    get_package_share_directory('flying_disc_description'))

flying_disc_xacro_file = os.path.join(flying_disc_description_path,
                          'urdf',
                          'flying_disc.urdf.xacro')

# xacroをロード
flying_disc_doc = xacro.process_file(flying_disc_xacro_file, mappings={'use_sim' : 'true'})
# xacroを展開してURDFを生成
flying_disc_desc = flying_disc_doc.toprettyxml(indent='  ')

class DiscSpawner(Node):

    def __init__(self):
        super().__init__('spawner')
        self.subscription = self.create_subscription(
            PoseStamped,
            'spawn_disc_pose',
            self.spawn_callback,
            10)
        self.subscription  # prevent unused variable warning
        
        self.disc_queue = queue.Queue()
        self.thread_processing = False

        self.start_time = time.time()     
        self.disc_remain_time = -1
        self.disc_id = -1
        self.destroied = True

        self.time_period = 1.0

        # Node.create_timer(timer_period_sec, callback)に引数を渡してタイマーを作成
        self.tmr = self.create_timer(self.time_period, self.timer_callback)

    def spawn_callback(self, msg):
        if not self.thread_processing:
            threading.Thread(target=self.spawn_func, args=(msg,)).start()
        
    def spawn_func(self, msg):
        self.thread_processing = True
        completed_process = subprocess.run(["ign", "model", "-m", msg.header.frame_id, "-p"
                ], encoding='utf-8', stdout = subprocess.PIPE)
        result_list = completed_process.stdout.split("\n")
        if len(result_list) < 8:
            self.thread_processing = False
            return
        position = re.findall("(?<=\[).+?(?=\])", result_list[6])[0].split()
        orientation = re.findall("(?<=\[).+?(?=\])", result_list[7])[0].split()
        (roll, pitch, yaw) = euler_from_quaternion(msg.pose.orientation)
        disc_number = int((time.time()-self.start_time)*100)
        subprocess.run(["ros2", "run", "ros_gz_sim", "create",
                "-world", "empty",
                "-string", str(flying_disc_desc),
                "-name", "disc" + str(disc_number),
                "-x", str(float(position[0])+msg.pose.position.x),
                "-y", str(float(position[1])+msg.pose.position.y),
                "-z", str(float(position[2])+msg.pose.position.z),
                "-R", str(float(orientation[0])+roll),
                "-P", str(float(orientation[1])+pitch),
                "-Y", str(float(orientation[2])+yaw),
                "-allow_renaming", "false"], stdout = subprocess.PIPE)
        completed_process = subprocess.run(["ign", "model", "-m", "disc" + str(disc_number),
                ], encoding='utf-8', stdout = subprocess.PIPE)
        result_list = completed_process.stdout.split("\n")
        if len(result_list) >= 4:
            disc_id = int(re.findall("(?<=\[).+?(?=\])", result_list[3])[0])
            self.disc_queue.put((time.time()+REMAIN_TIME_SEC, disc_id))
        time.sleep(1)
        self.thread_processing = False

    def timer_callback(self):
        if time.time()-self.disc_remain_time > 0.0:
            if not self.destroied:
                subprocess.run(["ros2", "run", "ros_gz_sim_extra", "remove",
                        "-world", "empty",
                        "-id", str(self.disc_id)], stdout = subprocess.PIPE)
                self.destroied = True
            if not self.disc_queue.empty():
                (self.disc_remain_time, self.disc_id) = self.disc_queue.get()
                self.destroied = False
        
def main(args=None):
    rclpy.init(args=args)
    
    disc_spawner = DiscSpawner()

    rclpy.spin(disc_spawner)

    disc_spawner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
