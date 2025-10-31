#! /usr/bin/env python3
#-*- coding: utf-8 -*-
import tf
import sys
import rospy
import moveit_commander
from control_msgs.msg import GripperCommandActionGoal
from geometry_msgs.msg import PoseStamped, Pose
from yolov8_ros_msgs.msg import BoundingBoxes
from message_filters import ApproximateTimeSynchronizer, Subscriber
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse
from grasp_demo.srv import FindNearestModel
import random

class MoveRobot():
    def __init__(self):

        # 初始化 planning group
        self.robot = moveit_commander.robot.RobotCommander()
        self.arm_group = moveit_commander.move_group.MoveGroupCommander(
            "arm")

        self.gripper_group = moveit_commander.move_group.MoveGroupCommander(
            "gripper")
        
        # 设置机械手臂的速度和加速度
        self.arm_group.set_max_acceleration_scaling_factor(1)
        self.arm_group.set_max_velocity_scaling_factor(1)

        # 物体的位置
        self.Obj_pose = PoseStamped()
        self.Obj_pose.pose.position.x = 0
        self.find_enable = False
        self.Obj_class = ''

        # 添加这一行 👇
        self.next_place = 1  # 下一次要放到 place1

        # 物体吸附接口

        self.attach_srv = rospy.ServiceProxy('/link_attacher_node/attach',
                                             Attach)
        self.attach_srv.wait_for_service()

        self.detach_srv = rospy.ServiceProxy('/link_attacher_node/detach',
                                             Attach)
        self.detach_srv.wait_for_service()

        # 获取最近的模型
        self.find_nearest_model = rospy.ServiceProxy('find_nearest_model', FindNearestModel)

        self.obj_pose_sub = Subscriber(
            "/objection_position_pose", Pose)
        
        self.yolo_sub = Subscriber("/yolov8/BoundingBoxes", BoundingBoxes)

		# Sync Subscribers
        self.ats = ApproximateTimeSynchronizer(
			[
				self.obj_pose_sub,
				self.yolo_sub
			],
			queue_size=5,
			slop=1,
			allow_headerless=True
		)

        self.ats.registerCallback(self.msg_filter_callback)

    def msg_filter_callback(self, msg, yolo_msg):
        # 1) 只有在 enable 状态下才处理
        if not self.find_enable:
            return

        # 2) 如果没有检测到任何 bounding box，就直接返回
        if not yolo_msg.bounding_boxes:
            rospy.logwarn("msg_filter_callback: no bounding boxes, skip")
            return

        # 3) 安全地取第一个 box
        self.Obj_pose.pose = msg
        self.Obj_class = yolo_msg.bounding_boxes[0].Class

        # 4) 一旦拿到有效位置，就关闭 find_enable
        if self.Obj_pose.pose.position.x != 0:
            self.find_enable = False

    def stop(self):
        moveit_commander.roscpp_initializer.roscpp_shutdown()

    def find_model(self, x, y, z):

        resp = self.find_nearest_model(x, y, z)
        return resp.model_name
    
    def gazeboAttach(self, obj):

        rospy.loginfo("Attaching gripper and object")
        req = AttachRequest()

        req.model_name_1 = "robot"
        req.link_name_1 = "link6"

        req.model_name_2 = obj
        req.link_name_2 = "base_link"

        self.attach_srv.call(req)

    def gazeboDetach(self, obj):

        rospy.loginfo("Detaching gripper and object")
        req = AttachRequest()
        req.model_name_1 = "robot"
        req.link_name_1 = "link6"

        req.model_name_2 = obj
        req.link_name_2 = "base_link"

        self.detach_srv.call(req)

    def gripper_move(self, width):

        cj = self.gripper_group.get_current_joint_values()

        cj[0] = -width  # 设置夹爪的宽度
        cj[1] = -width  # 设置夹爪的宽度

        self.gripper_group.set_joint_value_target(cj)
        self.gripper_group.go(wait=True)

    def plan_cartesian_path(self, pose):
        """
        笛卡尔路径规划

        Parameters:
        pose - 目标pose

        Returns:
        None
        """
        waypoints = [pose]

        # 设置机器臂当前的状态作为运动初始状态
        self.arm_group.set_start_state_to_current_state()

        fraction = 0.0   # 路径规划覆盖率
        maxtries = 100   # 最大尝试规划次数
        attempts = 0     # 已经尝试规划次数

        # 尝试规划一条笛卡尔空间下的路径，依次通过所有路点
        while fraction < 1.0 and attempts < maxtries:
            (plan, fraction) = self.arm_group.compute_cartesian_path(
                waypoints,  # waypoint poses，路点列表
                0.01,       # eef_step，终端步进值
                False       # avoid_collisions，避障规划
            )

            # 尝试次数累加
            attempts += 1

            # 打印运动规划进程
            if attempts % 10 == 0:
                rospy.loginfo("Still trying after " + str(attempts) + " attempts...")

        # 如果路径规划成功（覆盖率100%）,则开始控制机械臂运动
        if fraction > 0.9:
            rospy.loginfo("Path computed successfully. Moving the arm.")
            self.arm_group.execute(plan, wait=True)
            rospy.loginfo("Path execution complete.")
        # 如果路径规划失败，则打印失败信息
        else:
            rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(attempts) + " attempts.")

    def goSP(self):

        self.arm_group.set_joint_value_target([-1.42600646019347, -0.6654795904953197, -2.0600720730880955, -1.9857785438875304, 1.5713970317717658, -1.426795764006920])
        self.arm_group.go(wait=True)

    def grasp_obj(self):

        # 去到物体上方位置
        print(self.Obj_pose)
        current_pose = self.arm_group.get_current_pose()
        self.Obj_pose.pose.orientation = current_pose.pose.orientation

        self.Obj_pose.pose.position.z += 0.35
        self.plan_cartesian_path(self.Obj_pose.pose)

        self.Obj_pose.pose.position.z -= 0.09
        self.plan_cartesian_path(self.Obj_pose.pose)

        cp = self.arm_group.get_current_pose().pose
        self.model_name = self.find_model(cp.position.x, cp.position.y, cp.position.z)

        print(self.model_name)

        self.gripper_move(0.021)
        self.gazeboAttach(self.model_name)

    def move_obj(self, place):

        self.Obj_pose.pose.position.z += 0.1
        self.plan_cartesian_path(self.Obj_pose.pose)

        if place == 0:
            self.arm_group.set_joint_value_target([-2.293984716022825, -1.5648051051230158, -1.6090742485541867, -3.008545077645789, 0.8493561487349961, -0.05446061968624249])
            self.arm_group.go()

            self.arm_group.set_joint_value_target([-0.2366192138065646, -1.6647092498036438, 1.6630617121944766, -3.0694460917878734, -2.9044654189365504, -3.0608461287977935])
            self.arm_group.go()

            self.arm_group.set_joint_value_target([-1.2227173902871424, -1.309783061561677, 1.2979998332758687, -3.1120954808221652, -1.9191373952501927, -3.1233565660272635])
            self.arm_group.go()
        
        if place == 1:

            self.arm_group.set_joint_value_target([1.6719146976286305, -1.6285861244232684, 1.9387121451250877, 2.04236311261619, -0.04184029047063298, -2.3557749240282932])
            self.arm_group.go()

            self.arm_group.set_joint_value_target([2.93711090982176, -1.1417405635809716, 1.117150632690401, 3.1351327345344835, -1.295371369402555, -3.1360148044744918])
            self.arm_group.go()

        self.gazeboDetach(self.model_name)

        if place == 0:

            self.arm_group.set_joint_value_target([-0.2366192138065646, -1.6647092498036438, 1.6630617121944766, -3.0694460917878734, -2.9044654189365504, -3.0608461287977935])
            self.arm_group.go()
    
            self.arm_group.set_joint_value_target([-2.293984716022825, -1.5648051051230158, -1.6090742485541867, -3.008545077645789, 0.8493561487349961, -0.05446061968624249])
            self.arm_group.go()

        if place == 1:

            self.arm_group.set_joint_value_target([1.6719146976286305, -1.6285861244232684, 1.9387121451250877, 2.04236311261619, -0.04184029047063298, -2.3557749240282932])
            self.arm_group.go()

    def main_loop(self):
        try:
            while not rospy.is_shutdown():
                rospy.loginfo(f"准备抓取物体，下一个放置位置: place{self.next_place}")

                # 1. 回到预抓取位置
                self.goSP()
                self.gripper_move(0.03)  # 张开夹爪

                # 2. 开启物体识别
                self.find_enable = True
                rospy.loginfo("等待物体检测结果...")
                rospy.sleep(3)  # 给 YOLO 和定位足够时间（可调）

                # 3. 检查是否成功获取物体位置
                if self.find_enable == True:  # 说明没收到有效数据，find_enable 没被设为 False
                    rospy.logwarn("未检测到物体，结束任务。")
                    break  # 退出循环

                # 4. 抓取物体
                self.grasp_obj()

                # 5. 放置到指定位置（0 或 1）
                self.move_obj(self.next_place)

                # 6. 更新下一次放置位置（0 和 1 交替）
                self.next_place = 1 - self.next_place  # 1→0, 0→1

                # 7. 重置物体位置标志，准备下一次检测
                self.Obj_pose.pose.position.x = 0

                rospy.sleep(0.5)  # 小延迟，避免干扰

        except Exception as e:
            rospy.logerr("主循环出错: " + str(e))
   
def main():
    rospy.init_node('grasp_demo', anonymous=True)
    rospy.loginfo('Start Grasp Demo')
    moverobot = MoveRobot()
    while(not rospy.is_shutdown()):
        moverobot.main_loop()


if __name__ == "__main__":

    main()
