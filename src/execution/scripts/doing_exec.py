#!/usr/bin/env python3
import rclpy
import json
import asyncio
from typing import List
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from rclpy.time import Time
import tf2_ros
import math
from typing import Optional
from task_list.exec_list import ExecListNode

class DoingExec(Node):
    def __init__(self,exec_list_node: ExecListNode):
        super().__init__('doing_exec')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.control_pub = self.create_publisher(String, 'control', 10)
        self.robot_site_pub = self.create_publisher(String, 'robot_site', 10)
        self.exec_list = exec_list_node
        self.pose_queue = asyncio.Queue()
        self.last_stamp = None
        self.moving = False
        self.target = None
        self.turning = False
        self.last_yaw = None
        self.timer = self.create_timer(0.1, self.tf_update)
    def doing_exec(self, task_list: List[str]):
        func = getattr(, None)
    async def move(self, x, y):
        self.get_logger().info(f"Start move to {x}, {y}")
        self.target = (x, y)
        self.moving = True
        while True:
            current_x, current_y = await self.pose_queue.get()
            dx = x - current_x
            dy = y - current_y
            dist = math.sqrt(dx * dx + dy * dy)
            self.get_logger().info(f"移动偏差值: {dist}")
            msg = String()
            msg.data = json.dumps({
                "topic_name": "cmd_vel",
                "data": [dx,dy,self.last_yaw],
            })
            self.control_pub.publish(msg)
            if dist < 0.1:
                self.get_logger().info("结束移动")
                break
        self.moving = False
    async def turn(self, yaw):
        self.get_logger().info(f"Start turn to {yaw}")
        self.target = yaw
        self.last_yaw = yaw
        self.turning = True
        while True:
            current_yaw = await self.pose_queue.get()
            dyaw = yaw - current_yaw
            self.get_logger().info(f"旋转偏差值: {dyaw}")
            msg = String()
            msg.data = json.dumps({
                "topic_name": "cmd_vel",
                "data": [0, 0, dyaw],
            })
            self.control_pub.publish(msg)
            if abs(dyaw) < 0.1:
                self.get_logger().info("结束转动")
                break
        self.turning = False
    async def others(self, topic ,arg):
        self.get_logger().info(f"Start {topic}")
        msg = String()
        msg.data = json.dumps({
            "topic_name": topic,
            "data": arg
        })
        self.control_pub.publish(msg)
    def tf_update(self):#这一部分采用了轮询，可能会出现更新幅度频率不稳定的情况，以后再改
        tf_msg = self.read_base_to_map_tf()
        if tf_msg is None:
            return 
        if self.last_stamp == tf_msg.header.stamp:
            return
        self.last_stamp = tf_msg.header.stamp
        x = tf_msg.transform.translation.x
        y = tf_msg.transform.translation.y
        yaw = self.quaternion_to_yaw(tf_msg.transform.rotation)
        # 只在 move/turn运行时才推送
        if self.moving:
            if self.pose_queue.empty():
                asyncio.create_task(self.pose_queue.put((x, y)))
        if self.turning:
            if self.pose_queue.empty():
                asyncio.create_task(self.pose_queue.put(yaw))
        self.robot_site_pub.publish(String(data=json.dumps([x, y, yaw])))
    def quaternion_to_yaw(self, q):
        x = q.x
        y = q.y
        z = q.z
        w = q.w
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw
    def read_base_to_map_tf(self) -> Optional[TransformStamped]:
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                "map",
                "base_link",
                rclpy.time.Time()
            )
            return tf_msg
        except tf2_ros.TransformException:
            return None

def main(args=None):
    rclpy.init(args=args)
    node = DoingExec()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
def main():
    rclpy.init()
    node = DoingExec()
    loop = asyncio.get_event_loop()
    async def ros_spin():
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            await asyncio.sleep(0)

    async def main_task():
        await node.move(2.0, 3.0)

    loop.create_task(ros_spin())
    loop.create_task(main_task())

    loop.run_forever()

if __name__ == '__main__':
    main()
'''我希望通过检测tf树某个节点的更新情况，将新的数据更新到asynico队列里面，然后在move函数中提取这个队列中的新数据，通过与move预先输入的目标数据作比较来判断树否完成move任务'''