#!/usr/bin/env python3

from os import path
from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from pymoveit2 import MoveIt2
from pymoveit2.robots import ur5

DEFAULT_EXAMPLE_MESH = path.join(
    path.dirname(path.realpath(__file__)), "assets", "rack1.stl"
)


def main():
    rclpy.init()

    # Create node for this example
    node = Node("ex_task")

    # Declare parameter for joint positions
    node.declare_parameter(
        "filepath1",
        path.join(path.dirname(path.realpath(__file__)), "assets", "rack1.stl"),
    )
    node.declare_parameter(
        "filepath3",
        path.join(path.dirname(path.realpath(__file__)), "assets", "rack3.stl"),
    )
    node.declare_parameter(
        "action",
        "add",
    )
    node.declare_parameter("position1", [0.55, 0.06, -0.58])
    node.declare_parameter("quat_xyzw1", [0.0, 0.0, 0.0, 1.0])
    node.declare_parameter("position3", [0.25, -0.64, -0.58])
    node.declare_parameter("quat_xyzw3", [0.0, 0.0, -0.707, 0.707])

    # Create callback group that allows execution of callbacks in parallel without restrictions
    callback_group = ReentrantCallbackGroup()

    # Create MoveIt 2 interface
    moveit2 = MoveIt2(
        node=node,
        joint_names=ur5.joint_names(),
        base_link_name=ur5.base_link_name(),
        end_effector_name=ur5.end_effector_name(),
        group_name=ur5.MOVE_GROUP_ARM,
        callback_group=callback_group,
    )

    # Spin the node in background thread(s)
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()

    # Get parameters
    action = node.get_parameter("action").get_parameter_value().string_value
    filepath1 = node.get_parameter("filepath1").get_parameter_value().string_value
    position1 = node.get_parameter("position1").get_parameter_value().double_array_value
    quat_xyzw1 = node.get_parameter("quat_xyzw1").get_parameter_value().double_array_value
    filepath3 = node.get_parameter("filepath3").get_parameter_value().string_value
    position3 = node.get_parameter("position3").get_parameter_value().double_array_value
    quat_xyzw3 = node.get_parameter("quat_xyzw3").get_parameter_value().double_array_value

    # Determine ID of the collision mesh
    mesh_id1 = path.basename(filepath1).split(".")[0]
    mesh_id3 = path.basename(filepath3).split(".")[0]

    if "add" == action:
        # Add collision mesh
        i=0
        for i in range(3):
            moveit2.add_collision_mesh(
                filepath="~/eYantra/src/pymoveit2/examples/assets/rack1.stl", id=mesh_id1, position=position1, quat_xyzw=quat_xyzw1, frame_id=ur5.base_link_name()
            )
            i=i+1
        i=0
        for i in range(3):
            moveit2.add_collision_mesh(
                filepath="~/eYantra/src/pymoveit2/examples/assets/rack2.stl", id=mesh_id3, position=position3, quat_xyzw=quat_xyzw3, frame_id=ur5.base_link_name()
            )
            i=i+1    

    # Get parameters
    cartesian = False

    P1p = [0.35, 0.1, 0.68]
    P1q = [0.50, 0.50, 0.50, 0.50]

    P2p = [0.194, -0.43, 0.701]
    P2q = [0.50, -0.50, 0.50, 0.50]

    Dp = [-0.37, 0.12, 0.397]
    Dq = [-0.50, 0.50, 0.50, -0.50]

    # Move to pose
    # node.get_logger().info(
    #     f"Moving to {{position: {list(position)}, quat_xyzw: {list(quat_xyzw)}}}"
    # )
    moveit2.move_to_pose(position=P1p, quat_xyzw=P1q, cartesian=cartesian)
    moveit2.wait_until_executed()
    moveit2.move_to_pose(position=Dp, quat_xyzw=Dq, cartesian=cartesian)
    moveit2.wait_until_executed()
    moveit2.move_to_pose(position=P2p, quat_xyzw=P2q, cartesian=cartesian)
    moveit2.wait_until_executed()
    moveit2.move_to_pose(position=Dp, quat_xyzw=Dq, cartesian=cartesian)
    moveit2.wait_until_executed()

    
    rclpy.shutdown()
    exit(0)


if __name__ == "__main__":
    main()