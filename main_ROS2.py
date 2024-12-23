from __future__ import division
import random
import numpy as np
import math
import pybullet as p
import sim_update
import threading
import time
import rclpy
from rclpy.node import Node
from dsr_msgs2.srv import MoveJoint

MAX_ITERS = 10000
delta_q = 0.1  # Step size
k_att = 1.5   # Coefficient for attractive force
k_rep = 1.2   # Coefficient for repulsive force
d0 = 0.1      # Distance threshold for repulsive force


class NodeROS(Node):
    def __init__(self):
        super().__init__('robot_trajectory_controller')
        self.client = self.create_client(MoveJoint, '/dsr01/motion/move_joint')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.req = MoveJoint.Request()

    def send_trajectory(self, trajectory):
        for joint_angles in trajectory:
            joint_angles_in_degrees = [math.degrees(angle) for angle in joint_angles]  # Chuyển đổi radian sang độ
            self.req.pos = joint_angles_in_degrees
            self.req.vel = 1000.0  # Tốc độ di chuyển
            self.req.acc = 1000.0  # Gia tốc
            self.req.time = 0.0  # Thời gian thực hiện lệnh
            self.req.radius = 0.0  # Bán kính chuyển động tròn nếu cần
            self.req.mode = 0  # Chế độ điều khiển
            self.req.blend_type = 1
            self.req.sync_type = 0

            future = self.client.call_async(self.req)
            rclpy.spin_until_future_complete(self, future)

            if future.result().success:
                self.get_logger().info('Successfully moved to: %s' % joint_angles_in_degrees)
            else:
                self.get_logger().error('Failed to move to: %s' % joint_angles_in_degrees)
            time.sleep(1.0)


class NodeSim:
    def __init__(self, joint_positions, parent=None):
        self.joint_positions = joint_positions
        self.parent = parent


def visualize_path(q_1, q_2, env, color=[0, 1, 0]):
    env.set_joint_positions(q_1)
    point_1 = list(p.getLinkState(env.robot_body_id, 6)[0])
    point_1[2] -= 0.15
    
    env.set_joint_positions(q_2)
    point_2 = list(p.getLinkState(env.robot_body_id, 6)[0])
    point_2[2] -= 0.15

    p.addUserDebugLine(point_1, point_2,color, 1.5)

def dynamic_rrt_star(env, q_init, q_goal, MAX_ITERS, delta_q, steer_goal_p, distance=0.1):
    V, E = [NodeSim(q_init)], []
    path, found = [], False

    for i in range(MAX_ITERS):
        q_rand = semi_random_sample(steer_goal_p, q_goal)
        q_nearest = nearest([node.joint_positions for node in V], q_rand)
        q_new = steer(q_nearest, q_rand, delta_q)

        # Giới hạn bước di chuyển sau khi cộng thêm lực
        if get_euclidean_distance(q_new, q_nearest) > delta_q:
            q_new = steer(q_nearest, q_new, delta_q)

        if not env.check_collision(q_new, distance=0.18):
            q_new_node = NodeSim(q_new)
            q_nearest_node = next(node for node in V if node.joint_positions == q_nearest)
            q_new_node.parent = q_nearest_node

            if q_new_node not in V:
                V.append(q_new_node)
            if (q_nearest_node, q_new_node) not in E:
                E.append((q_nearest_node, q_new_node))
                visualize_path(q_nearest, q_new, env)
            if get_euclidean_distance(q_goal, q_new) < delta_q:
                q_goal_node = NodeSim(q_goal, q_new_node)
                V.append(q_goal_node)
                E.append((q_new_node, q_goal_node))
                visualize_path(q_new, q_goal, env)
                found = True
                break

    if found:
        current_node = q_goal_node
        path.append(current_node.joint_positions)
        while current_node.parent is not None:
            current_node = current_node.parent
            path.append(current_node.joint_positions)
        path.reverse()
        return path
    else:
        return None


def semi_random_sample(steer_goal_p, q_goal):
    prob = random.random()
    if prob < steer_goal_p:
        return q_goal
    else:
        q_rand = [random.uniform(-np.pi, np.pi) for _ in range(len(q_goal))]
    return q_rand


def get_euclidean_distance(q1, q2):
    return math.sqrt(sum((q2[i] - q1[i]) ** 2 for i in range(len(q1))))


def nearest(V, q_rand):
    distance = float("inf")
    q_nearest = None
    for v in V:
        if get_euclidean_distance(q_rand, v) < distance:
            q_nearest = v
            distance = get_euclidean_distance(q_rand, v)
    return q_nearest


def steer(q_nearest, q_rand, delta_q):
    if get_euclidean_distance(q_rand, q_nearest) <= delta_q:
        return q_rand
    else:
        q_hat = [(q_rand[i] - q_nearest[i]) / get_euclidean_distance(q_rand, q_nearest) for i in range(len(q_rand))]
        q_new = [q_nearest[i] + q_hat[i] * delta_q for i in range(len(q_hat))]
    return q_new

def get_grasp_position_angle(object_id):
    position, grasp_angle = np.zeros((3, 1)), 0
    position, orientation = p.getBasePositionAndOrientation(object_id)
    grasp_angle = p.getEulerFromQuaternion(orientation)[2]
    return position, grasp_angle

def run_dynamic_rrt_star():
    rclpy.init(args=None)
    ros_node = NodeROS()

    env.load_gripper()
    passed = 0
    for _ in range(10):
        object_id = env._objects_body_ids[0]
        position, grasp_angle = get_grasp_position_angle(object_id)
        grasp_success = env.execute_grasp(position, grasp_angle)

        if grasp_success:
            path_conf = dynamic_rrt_star(env, env.robot_home_joint_config,
                                     env.robot_goal_joint_config, MAX_ITERS, delta_q, 0.5)
            
            if path_conf:
                print("\nĐường đi tìm được (các góc khớp):")
                print("[")
                for joint_state in path_conf:
                    joint_degrees = [round(math.degrees(angle), 2) for angle in joint_state]
                    print(f"    {joint_degrees},")
                print("]")
                
                print("\nTọa độ quỹ đạo thực tế của end-effector:")
                print("[")
                for joint_state in path_conf:
                    env.set_joint_positions(joint_state)
                    end_pos = list(p.getLinkState(env.robot_body_id, env.robot_end_effector_link_index)[0])
                    end_pos[2] -= 0.15  # Trừ 0.15 ở trục Z
                    end_pos_rounded = [round(coord, 3) for coord in end_pos]
                    print(f"    {end_pos_rounded},")
                print("]")

                # Sau khi có đầy đủ đường đi, bắt đầu thực thi
                print("\nBắt đầu thực thi đường đi...")
                
                # Di chuyển trong môi trường mô phỏng
                env.set_joint_positions(env.robot_home_joint_config)
                markers = []
                for joint_state in path_conf:
                    env.move_joints(joint_state, speed=0.05)
                    link_state = p.getLinkState(env.robot_body_id, env.robot_end_effector_link_index)
                    markers.append(sim_update.SphereMarker(link_state[0], radius=0.02))
                
                # Gửi đường đi cho ROS
                print("Gửi đường đi tới ROS...")
                ros_node.send_trajectory(path_conf)

                print("Đường đi hoàn thành. Thả vật...")
                env.open_gripper()
                env.step_simulation(num_steps=5)
                env.close_gripper()

                # Đường đi ngược lại
                path_conf1 = path_conf[::-1]
                if path_conf1:
                    print("\nBắt đầu đường đi ngược lại...")
                    print("\nVị trí end-effector đường về:")
                    for joint_state in path_conf1:
                        env.move_joints(joint_state, speed=0.05)
                        end_pos = list(p.getLinkState(env.robot_body_id, env.robot_end_effector_link_index)[0])
                        end_pos[2] -= 0.15  # Trừ 0.15 ở trục Z
                        print([round(pos, 3) for pos in end_pos])
                        markers.append(sim_update.SphereMarker(link_state[0], radius=0.02))
                    
                    print("Gửi đường về tới ROS...")
                    ros_node.send_trajectory(path_conf1)

                markers = None
            p.removeAllUserDebugItems()
            
        env.robot_go_home()
        object_pos, _ = p.getBasePositionAndOrientation(object_id)
        if object_pos[0] >= -0.8 and object_pos[0] <= -0.2 and \
                object_pos[1] >= -0.3 and object_pos[1] <= 0.3 and \
                object_pos[2] <= 0.2:
            passed += 1
        env.reset_objects()
    
    print(f"\nSố lần thực hiện thành công: {passed}/10")
    rclpy.shutdown()


if __name__ == "__main__":
    random.seed(1)
    object_shapes = ["assets/objects/cube.urdf"]
    env = sim_update.PyBulletSim(object_shapes=object_shapes)
    run_dynamic_rrt_star()
