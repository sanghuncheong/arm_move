
#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley, Mike Lautman

## BEGIN_SUB_TUTORIAL imports
##
## To use the Python MoveIt! interfaces, we will import the `moveit_commander`_ namespace.
## This namespace provides us with a `MoveGroupCommander`_ class, a `PlanningSceneInterface`_ class,
## and a `RobotCommander`_ class. More on these below. We also import `rospy`_ and some messages that we will use:
##
import time
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg
from arm_move.msg._arm_move_msg import arm_move_msg
from arm_move.msg._box_info_msg import box_info_msg

from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

## END_SUB_TUTORIAL


def all_close(goal, actual, tolerance):
    """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
    all_equal = True
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

    return True


class MoveGroupPythonIntefaceTutorial(object):
    """MoveGroupPythonIntefaceTutorial"""

    def __init__(self):
        super(MoveGroupPythonIntefaceTutorial, self).__init__()

        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        self.group_name = 'r_arm' # this is just for the initialization
        move_group = moveit_commander.MoveGroupCommander(self.group_name)
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)

        display_trajectory2vrep_publisher = rospy.Publisher('/r_arm/joint_set',
                                                            sensor_msgs.msg.JointState,
                                                            queue_size=20)

        planning_frame = move_group.get_planning_frame()
        # eef_link = move_group.get_end_effector_link()
        eef_link = 'Body_RF2_c3'
        print "end eef link list\n", eef_link
        group_names = robot.get_group_names()

        # Misc variables
        self.box_name = ''
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.display_trajectory2vrep_publisher = display_trajectory2vrep_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    def go_to_pose_goal(self):

        move_group = self.move_group
        pose_goal = geometry_msgs.msg.Pose()

        pose_goal.position.x = self.choosed_position.x
        pose_goal.position.y = self.choosed_position.y
        pose_goal.position.z = self.choosed_position.z
        pose_goal.orientation.x = self.choosed_orientation.x
        pose_goal.orientation.y = self.choosed_orientation.y
        pose_goal.orientation.z = self.choosed_orientation.z
        pose_goal.orientation.w = self.choosed_orientation.w

        # pose_goal.position.x = 0.3
        # pose_goal.position.y = -0.2
        # pose_goal.position.z = 0.41
        # pose_goal.orientation.x = -0.5
        # pose_goal.orientation.y = -0.5
        # pose_goal.orientation.z = 0.5
        # pose_goal.orientation.w = 0.5

        # move_group.set_planner_id('SPARStwo')
        # move_group.set_planner_id('RRTstar')
        # move_group.set_planner_id('BiTRRT')

        move_group.set_num_planning_attempts(10000)
        move_group.set_planning_time(5)
        move_group.set_goal_position_tolerance(0.01)
        move_group.set_goal_orientation_tolerance(0.01)


        move_group.set_pose_target(pose_goal)

        print "goal pose:", pose_goal
        ## Now, we call the planner to compute the plan and execute it.
        plan = move_group.go(wait=True)
        time.sleep(2)
        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        move_group.clear_pose_targets()

        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)

    def pickup(self, upCM, scale=1):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_cartesian_path
        ##
        ## Cartesian Paths
        ## ^^^^^^^^^^^^^^^
        ## You can plan a Cartesian path directly by specifying a list of waypoints
        ## for the end-effector to go through. If executing  interactively in a
        ## Python shell, set scale = 1.0.
        ##
        waypoints = []

        wpose = move_group.get_current_pose().pose
        wpose.position.z -= -0.05  # First move up (z)
        # wpose.position.z -= scale * 0.1  # First move up (z)
        waypoints.append(copy.deepcopy(wpose))


        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0,
        # ignoring the check for infeasible jumps in joint space, which is sufficient
        # for this tutorial.
        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints,  # waypoints to follow
            0.01,  # eef_step
            0.0)  # jump_threshold
        # print "joint names \n", plan.joint_trajectory.joint_names
        # print "msg type \n", type(plan.joint_trajectory.points)
        # print "trajectory length", len(plan.joint_trajectory.points)
        # print "msg type \n", type(plan.joint_trajectory.points[0])
        # print "msg type \n", type(plan.joint_trajectory.points[0].positions)
        # print plan.joint_trajectory.points[0]
        # print plan.joint_trajectory.points[0].positions

        # display_trajectory2vrep_publisher = self.display_trajectory2vrep_publisher
        # vrep_move = sensor_msgs.msg.JointState()
        # for i in range(len(plan.joint_trajectory.points)):
        #     print i, "th path!!"
        #     vrep_move.name = plan.joint_trajectory.joint_names
        #     vrep_move.position = plan.joint_trajectory.points[i].positions
        #     display_trajectory2vrep_publisher.publish(vrep_move)
        #     time.sleep(0.1)
        # print "this is the planned path \n", plan
        # Note: We are just planning, not asking move_group to actually move the robot yet:

        move_group.execute(plan, wait=True)
        return plan, fraction

        ## END_SUB_TUTORIAL

    def display_trajectory(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher

        ## BEGIN_SUB_TUTORIAL display_trajectory
        ##
        ## Displaying a Trajectory
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
        ## group.plan() method does this automatically so this is not that useful
        ## here (it just displays the same trajectory again):
        ##
        ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
        ## We populate the trajectory_start with our current robot state to copy over
        ## any AttachedCollisionObjects and add our plan to the trajectory.
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        display_trajectory_publisher.publish(display_trajectory);

        ## END_SUB_TUTORIAL

    def execute_plan(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL execute_plan
        ##
        ## Executing a Plan
        ## ^^^^^^^^^^^^^^^^
        ## Use execute if you would like the robot to follow
        ## the plan that has already been computed:
        move_group.execute(plan, wait=True)

        ## **Note:** The robot's current joint state must be within some tolerance of the
        ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
        ## END_SUB_TUTORIAL

    def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL wait_for_scene_update
        ##
        ## Ensuring Collision Updates Are Receieved
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## If the Python node dies before publishing a collision object update message, the message
        ## could get lost and the box will not appear. To ensure that the updates are
        ## made, we wait until we see the changes reflected in the
        ## ``get_attached_objects()`` and ``get_known_object_names()`` lists.
        ## For the purpose of this tutorial, we call this function after adding,
        ## removing, attaching or detaching an object in the planning scene. We then wait
        ## until the updates have been made or ``timeout`` seconds have passed
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = box_name in scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False
        ## END_SUB_TUTORIAL

    def add_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL add_box
        ##
        ## Adding Objects to the Planning Scene
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## First, we will create a box in the planning scene at the location of the left finger:
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "/base_footprint"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.z = 0.07  # slightly above the end effector
        box_name = "box"
        scene.add_box(box_name, box_pose, size=(0.1, 0.1, 0.1))

        ## END_SUB_TUTORIAL
        # Copy local variables back to class variables. In practice, you should use the class
        # variables directly unless you have a good reason not to.
        self.box_name = box_name
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    def add_box2(self, box_name, box_position, box_orientation, box_scale, timeout=4):

        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "/base_footprint"

        box_pose.pose.position.x = box_position.x
        box_pose.pose.position.y = box_position.y
        box_pose.pose.position.z = box_position.z
        box_pose.pose.orientation.x = box_orientation.x
        box_pose.pose.orientation.y = box_orientation.y
        box_pose.pose.orientation.z = box_orientation.z
        box_pose.pose.orientation.w = box_orientation.w

        box_scale = (box_scale.x,box_scale.y,box_scale.z)

        self.scene.add_box(box_name, box_pose, box_scale)

        ## END_SUB_TUTORIAL
        # Copy local variables back to class variables. In practice, you should use the class
        # variables directly unless you have a good reason not to.
        self.box_name = box_name
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    def attach_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        robot = self.robot
        scene = self.scene
        eef_link = self.eef_link
        group_names = self.group_names

        ## BEGIN_SUB_TUTORIAL attach_object
        ##
        ## Attaching Objects to the Robot
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## Next, we will attach the box to the Panda wrist. Manipulating objects requires the
        ## robot be able to touch them without the planning scene reporting the contact as a
        ## collision. By adding link names to the ``touch_links`` array, we are telling the
        ## planning scene to ignore collisions between those links and the box. For the Panda
        ## robot, we set ``grasping_group = 'hand'``. If you are using a different robot,
        ## you should change this value to the name of your end effector group name.
        grasping_group = 'hand'
        touch_links = robot.get_link_names(group=grasping_group)
        scene.attach_box(eef_link, box_name, touch_links=touch_links)
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)


    def attach_box2(self, box_name, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        # box_name = self.box_name
        robot = self.robot
        scene = self.scene
        eef_link = self.eef_link
        group_names = self.group_names

        ## BEGIN_SUB_TUTORIAL attach_object
        ##
        ## Attaching Objects to the Robot
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## Next, we will attach the box to the Panda wrist. Manipulating objects requires the
        ## robot be able to touch them without the planning scene reporting the contact as a
        ## collision. By adding link names to the ``touch_links`` array, we are telling the
        ## planning scene to ignore collisions between those links and the box. For the Panda
        ## robot, we set ``grasping_group = 'hand'``. If you are using a different robot,
        ## you should change this value to the name of your end effector group name.
        grasping_group = 'r_hand'
        touch_links = robot.get_link_names(group=grasping_group)
        print "touch links list\n",touch_links
        scene.attach_box(eef_link, box_name, touch_links=touch_links)
        ## END_SUB_TUTORIAL
        # time.sleep(1)
        # self.pickup(0.05)
        # We wait for the planning scene to update.
        return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)


    def detach_box2(self, box_name, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        # box_name = self.box_name
        scene = self.scene
        eef_link = self.eef_link

        ## BEGIN_SUB_TUTORIAL detach_object
        ##
        ## Detaching Objects from the Robot
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## We can also detach and remove the object from the planning scene:
        scene.remove_attached_object(eef_link, name=box_name)
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)

    def detach_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene
        eef_link = self.eef_link

        ## BEGIN_SUB_TUTORIAL detach_object
        ##
        ## Detaching Objects from the Robot
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## We can also detach and remove the object from the planning scene:
        scene.remove_attached_object(eef_link, name=box_name)
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)

    def remove_box2(self, box_name, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        # box_name = self.box_name

        scene = moveit_commander.PlanningSceneInterface()
        self.scene = scene
        self.box_name = box_name
        ## BEGIN_SUB_TUTORIAL remove_object
        ##
        ## Removing Objects from the Planning Scene
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## We can remove the box from the world.
        scene.remove_world_object(box_name)

        ## **Note:** The object must be detached before we can remove it from the world
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)

    def remove_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL remove_object
        ##
        ## Removing Objects from the Planning Scene
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## We can remove the box from the world.
        scene.remove_world_object(box_name)

        ## **Note:** The object must be detached before we can remove it from the world
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)

    def setjoint2(self):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        # move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_to_joint_state
        ##
        ## Planning to a Joint Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^^
        ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_ so the first
        ## thing we want to do is move it to a slightly better configuration.
        # We can get the joint values from the group and adjust some of the values:
        self.group_name = 'l_arm'  # this is just for the initialization
        move_group = moveit_commander.MoveGroupCommander(self.group_name)
        joint_goal = move_group.get_current_joint_values()

        joint_goal[0] = 0.60
        joint_goal[1] = +0.3
        joint_goal[2] = -0.054
        joint_goal[3] = -2.25
        joint_goal[4] = -1.59
        joint_goal[5] = -0.3
        joint_goal[6] = 0.01

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        move_group.go(joint_goal, wait=True)

        self.group_name = 'r_arm' # this is just for the initialization
        move_group = moveit_commander.MoveGroupCommander(self.group_name)
        joint_goal = move_group.get_current_joint_values()

        joint_goal[0] = 0.60
        joint_goal[1] = -0.3
        joint_goal[2] = -0.054
        joint_goal[3] = -2.25
        joint_goal[4] = -1.59
        joint_goal[5] = 0.3
        joint_goal[6] = 0.01

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        move_group.stop()



        ## END_SUB_TUTORIAL

        # For testing:
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)


def setjoint(data):
  try:
    print "setting start if 1, data:", data
    if data.data == '1':
        print "joint set start"
        tutorial.setjoint2()

  except 1:
    print "done"


def main_arm(data):
  try:
    print "move arm:", data.arm_name[0]
    print "goal position\n", data.goal_position
    print "goal orientation\n", data.goal_orientation

    tutorial.group_name = data.arm_name[0]
    tutorial.choosed_position = data.goal_position
    tutorial.choosed_orientation = data.goal_orientation

    tutorial.go_to_pose_goal()

    cur_pose = tutorial.move_group.get_current_pose().pose
    print "current pose:", cur_pose
  except 1:
    print "done"


def att_box(data):
  try:
    print "attach box:", data.object_name[0]

    print "position\n",    data.object_position
    print "orientation\n", data.object_orientation
    print "scale\n",       data.object_scale

    box_name = data.object_name[0]
    box_position = data.object_position
    box_orientation= data.object_orientation
    box_scale = data.object_scale

    tutorial.attach_box2(box_name)

  except 1:
    print "done"


def det_box(data):
  try:
    print "dettach box:", data.object_name[0]

    box_name = data.object_name[0]

    tutorial.detach_box2(box_name)

  except 1:
    print "done"

def add_box(data):
  try:
    print "add box:", data.object_name[0]

    print "position\n",    data.object_position
    print "orientation\n", data.object_orientation
    print "scale\n",       data.object_scale

    box_name = data.object_name[0]
    box_position = data.object_position
    box_orientation= data.object_orientation
    box_scale = data.object_scale

    tutorial.add_box2(box_name, box_position, box_orientation, box_scale)

  except 1:
    print "done"


def del_box(data):
  try:
    print "delete box:", data.object_name[0]

    box_name = data.object_name[0]
    tutorial.remove_box2(box_name)

  except 1:
    print "done"


def listener():

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('moveit_arm_controller', anonymous=True)
    rospy.Subscriber('arm_goalPose', arm_move_msg, main_arm)
    rospy.Subscriber('arm_initJoint', String, setjoint)
    rospy.Subscriber('add_box_info', box_info_msg, add_box)
    rospy.Subscriber('del_box_info', box_info_msg, del_box)
    rospy.Subscriber('att_box_info', box_info_msg, att_box)
    rospy.Subscriber('det_box_info', box_info_msg, det_box)


    rospy.spin()

if __name__ == '__main__':
    print "------------------------------"
    print "Arm trajectory NODE starts!!!!"
    print "------------------------------"
    print "Press Ctrl-D to exit at any time"
    tutorial = MoveGroupPythonIntefaceTutorial()
    object_list = []
    listener()

    print "added objects", object_list
    for i in range(len(object_list)):
        print "erase object", i
        tutorial.remove_box2(object_list[i])
    print "end node!!"


    ## BEGIN_TUTORIAL
    ## .. _moveit_commander:
    ##    http://docs.ros.org/melodic/api/moveit_commander/html/namespacemoveit__commander.html
    ##
    ## .. _MoveGroupCommander:
    ##    http://docs.ros.org/melodic/api/moveit_commander/html/classmoveit__commander_1_1move__group_1_1MoveGroupCommander.html
    ##
    ## .. _RobotCommander:
    ##    http://docs.ros.org/melodic/api/moveit_commander/html/classmoveit__commander_1_1robot_1_1RobotCommander.html
    ##
    ## .. _PlanningSceneInterface:
    ##    http://docs.ros.org/melodic/api/moveit_commander/html/classmoveit__commander_1_1planning__scene__interface_1_1PlanningSceneInterface.html
    ##
    ## .. _DisplayTrajectory:
    ##    http://docs.ros.org/melodic/api/moveit_msgs/html/msg/DisplayTrajectory.html
    ##
    ## .. _RobotTrajectory:
    ##    http://docs.ros.org/melodic/api/moveit_msgs/html/msg/RobotTrajectory.html
    ##
    ## .. _rospy:
    ##    http://docs.ros.org/melodic/api/rospy/html/
    ## CALL_SUB_TUTORIAL imports
    ## CALL_SUB_TUTORIAL setup
    ## CALL_SUB_TUTORIAL basic_info
    ## CALL_SUB_TUTORIAL plan_to_joint_state
    ## CALL_SUB_TUTORIAL plan_to_pose
    ## CALL_SUB_TUTORIAL plan_cartesian_path
    ## CALL_SUB_TUTORIAL display_trajectory
    ## CALL_SUB_TUTORIAL execute_plan
    ## CALL_SUB_TUTORIAL add_box
    ## CALL_SUB_TUTORIAL wait_for_scene_update
    ## CALL_SUB_TUTORIAL attach_object
    ## CALL_SUB_TUTORIAL detach_object
    ## CALL_SUB_TUTORIAL remove_object
    ## END_TUTORIAL