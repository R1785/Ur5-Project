#! /usr/bin/env python

import rospy
import sys
import copy

import tf2_ros
import tf2_msgs.msg
import moveit_commander

import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
from pkg_vb_sim.srv import *

class CartesianPath:

    # Constructor
    def __init__(self):

        rospy.init_node('node_eg5_waypoints', anonymous=True)

        self._planning_group = "ur5_1_planning_group"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self._exectute_trajectory_client = actionlib.SimpleActionClient(
            'execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()

        rospy.loginfo('\033[94m' + " >>> Init done." + '\033[0m')
        self._tfBuffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._tfBuffer)
        self.tfx = 0
        self.box_name = "box"
#............................................................................................
    def vaccum_gripper(self, value):
        ''' To initiate the vaccum gripper once the end effector is near the box'''
        rospy.wait_for_service("/eyrc/vb/ur5_1/activate_vacuum_gripper")
        vaccum = rospy.ServiceProxy("/eyrc/vb/ur5_1/activate_vacuum_gripper", vacuumGripper)
        v_accum = vaccum(value)
        return v_accum
#................................................................................................
    def conveyor_belt(self,power):
        ''' It is used to control conveyor belt speed '''
        rospy.wait_for_service("/eyrc/vb/conveyor/set_power")
        conve_yor = rospy.ServiceProxy("/eyrc/vb/conveyor/set_power", conveyorBeltPowerMsg)
        conveyor= conve_yor(power)
        return conveyor
#..................................................................................................
    def ee_cartesian_translation(self, trans_x, trans_y, trans_z):
        ''' It is used for cartesian translation'''
        # 1. Create a empty list to hold waypoints
        waypoints = []

        # 2. Add Current Pose to the list of waypoints
        waypoints.append(self._group.get_current_pose().pose)

        # 3. Create a New waypoint
        wpose = geometry_msgs.msg.Pose()
        wpose.position.x = waypoints[0].position.x + (trans_x)
        wpose.position.y = waypoints[0].position.y + (trans_y)
        wpose.position.z = waypoints[0].position.z + (trans_z)
    #    print("position x",wpose.position.x)
        # This to keep EE parallel to Ground Plane
        wpose.orientation.x = -0.5
        wpose.orientation.y = -0.5
        wpose.orientation.z = 0.5
        wpose.orientation.w = 0.5


        # 4. Add the new waypoint to the list of waypoints
        waypoints.append(copy.deepcopy(wpose))


        # 5. Compute Cartesian Path connecting the waypoints in the list of waypoints
        (plan, fraction) = self._group.compute_cartesian_path(
            waypoints,   # waypoints to follow
            0.01,        # Step Size, distance between two adjacent computed waypoints will be 1 cm
            0.0)         # Jump Threshold
        rospy.loginfo("Path computed successfully. Moving the arm.")

        # The reason for deleting the first two waypoints from the computed Cartisian Path can be found here,
        # https://answers.ros.org/question/253004/moveit-problem-error-trajectory-message-contains-waypoints-that-are-not-strictly-increasing-in-time/?answer=257488#post-id-257488
        num_pts = len(plan.joint_trajectory.points)
        if (num_pts >= 3):
            del plan.joint_trajectory.points[0]
            del plan.joint_trajectory.points[1]

        # 6. Make the arm follow the Computed Cartesian Path
        self._group.execute(plan)
#.......................................................................

    def func_tf_print(self, arg_frame_1, arg_frame_2):
        ''' It is used to fetch tf of respective reference frame and target frame'''
        try:
            self.trans = self._tfBuffer.lookup_transform(arg_frame_1, arg_frame_2, rospy.Time())
            self.tfx = self.trans.transform.translation.x
            self.tfy = self.trans.transform.translation.y
            self.tfz = self.trans.transform.translation.z

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("TF error")
#........................................................................................................
    def go_to_pose(self, arg_pose):

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        self._group.set_pose_target(arg_pose)
        flag_plan = self._group.go(wait=True)  # wait=False for Async Move

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        if (flag_plan == True):
            rospy.loginfo('\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found." + '\033[0m')

        return flag_plan
#.....................................................................................
    def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
        '''Method to ensure that the updates are made'''

        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
      # Test if the box is in attached objects
            attached_objects = self._scene.get_attached_objects([self.box_name])
            is_attached = len(attached_objects.keys()) > 0

            is_known = self.box_name in self._scene.get_known_object_names()


            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            rospy.sleep(0.1)
            seconds = rospy.get_time()
#............................................................................................
    def set_joint_angles(self, arg_list_joint_angles):
        ''' To set Joint angles'''
        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        self._group.set_joint_value_target(arg_list_joint_angles)
        self._group.plan()
        flag_plan = self._group.go(wait=True)

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        if flag_plan == True:
            rospy.loginfo(
                '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

        return flag_plan
#.......................................................................................

    def attach_box(self, timeout=4):

        ''' method to attach the box to the end effector'''
        #box_name = self.box_name
        robot = self._robot
        scene = self._scene
        eef_link = self._eef_link



        grasping_group = 'ur5_1_planning_group'
        touch_links = robot.get_link_names(group=grasping_group)
        scene.attach_box(eef_link, self.box_name, touch_links=touch_links)

        return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)
#........................................................................................

    def add_box(self, x,y,z, timeout=4):
        ''' Method to add box to the planning scene'''

        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "world"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.z = z
        box_pose.pose.position.x = x
        box_pose.pose.position.y = y
        self._scene.add_box(self.box_name, box_pose, size=(0.15, 0.15, 0.15))

        self.box_name = "box"
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)
#.............................................................................
    def detach_box(self, timeout=4):
        ''' Method to detach box '''
        #box_name = self.box_name
        scene = self._scene
        eef_link = self._eef_link

        scene.remove_attached_object(eef_link, name=self.box_name)

        return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)
#.................................................................................................

    def remove_box(self, timeout=4):
        ''' method to remove box from the planning scene'''
        self._scene.remove_world_object(self.box_name)
        return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)
#.................................................................................................

    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class CartesianPath Deleted." + '\033[0m')

#....................................................................................................
def main():
    rospy.sleep(32)

    ur5 = CartesianPath()

    box_length = 0.15               # Length of the Package
    vacuum_gripper_width = 0.115    # Vacuum Gripper Width
    delta = vacuum_gripper_width + (box_length/2)+1  # 0.19



    lst_joint_angles_4 = [1.6590395859861982,
                         -3.1093074647177525,
                         -0.6103533600857993,
                         -0.992773068443646,
                          1.5702635365874809,
                          1.6583834569470097]

    lst_joint_angles_1 = [-0.06181728152852184,
                          -0.9320059933442044,
                           1.9684910488519431,
                          -2.606440782278974,
                          -1.5708083177647998,
                           3.079279914134135]

    lst_joint_angles_3 = [1.2257217506093383,
                          -0.9153297873004362,
                           1.7397218480674201,
                          -2.3944687656501094,
                          -1.5707469309093014,
                           -1.914974671479392]

    lst_joint_angles_2 = [3.0047207373146243,
                          -0.6992509644602434,
                           1.0175326195302157,
                          -1.889117482710196,
                          -1.5712690706727672,
                          -0.13627485126061956]

    ur5_2_home_pose = geometry_msgs.msg.Pose()
    ur5_2_home_pose.position.x = -0.8
    ur5_2_home_pose.position.y = 0
    ur5_2_home_pose.position.z = 1 + vacuum_gripper_width + (box_length/2)
    # This to keep EE parallel to Ground Plane
    ur5_2_home_pose.orientation.x = -0.5
    ur5_2_home_pose.orientation.y = -0.5
    ur5_2_home_pose.orientation.z = 0.5
    ur5_2_home_pose.orientation.w = 0.5

    ur5_2_home_pose1 = geometry_msgs.msg.Pose()
    ur5_2_home_pose1.position.x = 0.576872
    ur5_2_home_pose1.position.y =-0.909899
    ur5_2_home_pose1.position.z = 0.999034
    reference_frame = "world"
    target_frame = "ur5_wrist_3_link"
    target_frame2 = "logical_camera_2_packagen1_frame"
    target_frame3= "logical_camera_2_packagen2_frame"
    target_frame4= "logical_camera_2_packagen3_frame"
    while not rospy.is_shutdown():
        # 1. Go to Home Position

        #ur5.go_to_pose(ur5_2_home_pose)
        ur5.set_joint_angles(lst_joint_angles_2)
        ur5.func_tf_print(target_frame, target_frame2)
        ax=ur5.tfx
        by=ur5.tfy
        mz=ur5.tfz

         # 2. Translate EE by 0.5m  in x
        rospy.loginfo('\033[94m' + "Translating EE by 0.5m in x from current position." + '\033[0m')
        ur5.ee_cartesian_translation(-mz, ax, 0)

#............................................................................................
        ur5.func_tf_print(reference_frame, target_frame2)
        ax=ur5.tfx
        by=ur5.tfy
        mz=ur5.tfz

        ur5.add_box(ax,by,mz)
        ur5.attach_box()
        ur5.vaccum_gripper(True)

        ur5.set_joint_angles(lst_joint_angles_3)

        ur5.detach_box()
        ur5.vaccum_gripper(False)

#.....Finish Finish Finsih 1...........................................................................................
        #ur5.go_to_pose(ur5_2_home_pose)
        ur5.set_joint_angles(lst_joint_angles_2)

        ur5.func_tf_print(target_frame, target_frame3)


        ax=ur5.tfx
        by=ur5.tfy
        mz=ur5.tfz


         # 2. Translate EE by 0.5m  in x
        rospy.loginfo('\033[94m' + "Translating EE by 0.5m in x from current position." + '\033[0m')
        ur5.ee_cartesian_translation(-mz, ax, 0)

#..........................................................................................................
        ur5.func_tf_print(reference_frame, target_frame2)
        ax=ur5.tfx
        by=ur5.tfy
        mz=ur5.tfz

        ur5.add_box(ax,by,mz)
        ur5.attach_box()
        ur5.vaccum_gripper(True)

        ur5.ee_cartesian_translation(0,0,0.1)
#...................................................................................................
        ur5.set_joint_angles(lst_joint_angles_1)
        ur5.detach_box()
        ur5.vaccum_gripper(False)


#..............Finish Finish2.............................................................................
        #ur5.go_to_pose(ur5_2_home_pose)
        ur5.set_joint_angles(lst_joint_angles_2)
        ur5.func_tf_print(target_frame, target_frame4)

        ax=ur5.tfx
        by=ur5.tfy
        mz=ur5.tfz


         # 2. Translate EE by 0.5m  in x
        rospy.loginfo('\033[94m' + "Translating EE by 0.5m in x from current position." + '\033[0m')
        ur5.ee_cartesian_translation(-mz, ax, 0)
#............................................................................................

        ur5.func_tf_print(reference_frame, target_frame4)
        ax=ur5.tfx
        by=ur5.tfy
        mz=ur5.tfz
        ur5.add_box(ax,by,mz)
        ur5.vaccum_gripper(True)
        ur5.attach_box()

        ur5.set_joint_angles(lst_joint_angles_4)
        ur5.detach_box()
        ur5.vaccum_gripper(False)
        ur5.set_joint_angles(lst_joint_angles_2)

        break




#........................................................................................

    del ur5


if __name__ == '__main__':

    main()
