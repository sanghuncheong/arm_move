#!/usr/bin/env python
def move_goalpose_client(arm_name, hand_name, start_state, goal_pos, goal_ori, obj, planner_name, n_attempt, c_time, n_repeat, n_maxstep):
    import rospy
    from arm_move.srv._arm_move_srv import *

    rospy.wait_for_service('move_goalpose_srv')
    try:
        f_check_srv = rospy.ServiceProxy('move_goalpose_srv', arm_move_srv)
        # Grasp the object if obj
        if obj:
            att_box_client(hand_name, obj)
        pub_msg = arm_move_srvRequest()
        pub_msg.arm_name.append(arm_name)
        pub_msg.goal_position.x = goal_pos[0]
        pub_msg.goal_position.y = goal_pos[1]
        pub_msg.goal_position.z = goal_pos[2]
        pub_msg.goal_orientation.x = goal_ori[0]
        pub_msg.goal_orientation.y = goal_ori[1]
        pub_msg.goal_orientation.z = goal_ori[2]
        pub_msg.goal_orientation.w = goal_ori[3]
        pub_msg.planner_name = planner_name
        pub_msg.n_attempt = n_attempt
        pub_msg.c_time = c_time
        pub_msg.n_repeat = n_repeat
        pub_msg.start_state = start_state
        pub_msg.n_maxstep   = n_maxstep
        resp_f_check = f_check_srv(pub_msg)
        if resp_f_check.feasibility == 1:
            if len(resp_f_check.r_trj.joint_trajectory.points) > 0:
                # print "Plan is found with", len(resp_f_check.r_trj.joint_trajectory.points), "steps"
                return [1, resp_f_check.r_trj]
            else:
                # print "No plan found"
                return [0, resp_f_check.r_trj]
        else:
            # print "No plan found"
            return [0, resp_f_check.r_trj]
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def feasible_check_client(arm_name, goal_pos, goal_ori, n_maxstep):
    import rospy
    from arm_move.srv._arm_move_srv import *
    rospy.wait_for_service('feasibile_check_srv')
    try:
        f_check_srv = rospy.ServiceProxy('feasibile_check_srv', arm_move_srv)
        pub_msg = arm_move_srvRequest()
        pub_msg.arm_name.append(arm_name)

        pub_msg.goal_position.x = goal_pos[0]
        pub_msg.goal_position.y = goal_pos[1]
        pub_msg.goal_position.z = goal_pos[2]
        pub_msg.goal_orientation.x = goal_ori[0]
        pub_msg.goal_orientation.y = goal_ori[1]
        pub_msg.goal_orientation.z = goal_ori[2]
        pub_msg.goal_orientation.w = goal_ori[3]

        pub_msg.n_maxstep = n_maxstep

        resp_f_check = f_check_srv(pub_msg)

        if resp_f_check.feasibility == 1:
            if len(resp_f_check.r_trj.joint_trajectory.points) > 0:
                print "Plan is found with", len(resp_f_check.r_trj.joint_trajectory.points), "steps"
                return 1
            else:
                print "No plan found"
                return 0
        else:
            print "No plan found"
            return 0
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def add_box_client(box_name, box_xyz, box_xyzw, box_wdh, box_color):
    import rospy
    from arm_move.srv._box_info_srv import *
    rospy.wait_for_service('add_box_srv')
    try:
        # print "\tCLF) add box client", box_name
        add_box_srv = rospy.ServiceProxy('add_box_srv', box_info_srv)

        pub_msg = box_info_srvRequest()
        pub_msg.object_name.append(box_name)
        pub_msg.object_color.append(box_color)

        pub_msg.object_position.x = box_xyz[0]
        pub_msg.object_position.y = box_xyz[1]
        pub_msg.object_position.z = box_xyz[2]
        pub_msg.object_orientation.x = box_xyzw[0]
        pub_msg.object_orientation.y = box_xyzw[1]
        pub_msg.object_orientation.z = box_xyzw[2]
        pub_msg.object_orientation.w = box_xyzw[3]
        pub_msg.object_scale.x = box_wdh[0]
        pub_msg.object_scale.y = box_wdh[1]
        pub_msg.object_scale.z = box_wdh[2]

        resp1 = add_box_srv(pub_msg)

    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def del_box_client(box_name):
    import rospy
    from arm_move.srv._box_info_srv import *

    rospy.wait_for_service('del_box_srv')
    try:
        del_box_srv = rospy.ServiceProxy('del_box_srv', box_info_srv)
        # pub_msg = box_info_msg()
        pub_msg = box_info_srvRequest()
        pub_msg.object_name.append(box_name)

        resp1 = del_box_srv(pub_msg)

    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def att_box_client(hand_name, box_name):
    import rospy
    from arm_move.srv._att_hand_box_srv import *
    print "\tCLF) hand, box name:", hand_name, box_name
    rospy.wait_for_service('att_box_srv')
    try:
        att_box_srv = rospy.ServiceProxy('att_box_srv', att_hand_box_srv)

        pub_msg = att_hand_box_srvRequest()
        pub_msg.object_name.append(box_name)
        pub_msg.hand_name.append(hand_name)

        resp1 = att_box_srv(pub_msg)

    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def add_mesh_client(mesh_name, mesh_xyz, mesh_xyzw, mesh_wdh):
    import rospy
    from arm_move.srv._mesh_info_srv import *
    rospy.wait_for_service('add_mesh_srv')
    try:
        add_mesh_srv = rospy.ServiceProxy('add_mesh_srv', mesh_info_srv)

        pub_msg = mesh_info_srvRequest()
        pub_msg.object_name.append(mesh_name)
        pub_msg.object_color.append('')

        pub_msg.object_position.x = mesh_xyz[0]
        pub_msg.object_position.y = mesh_xyz[1]
        pub_msg.object_position.z = mesh_xyz[2]
        pub_msg.object_orientation.x = mesh_xyzw[0]
        pub_msg.object_orientation.y = mesh_xyzw[1]
        pub_msg.object_orientation.z = mesh_xyzw[2]
        pub_msg.object_orientation.w = mesh_xyzw[3]
        pub_msg.object_scale.x = mesh_wdh[0]
        pub_msg.object_scale.y = mesh_wdh[1]
        pub_msg.object_scale.z = mesh_wdh[2]

        pub_msg.file_name = mesh_name

        resp1 = add_mesh_srv(pub_msg)

    except rospy.ServiceException, e:
        print "Service call failed: %s" % e        


def det_box_client(box_name, box_xyz, box_xyzw, box_wdh, box_color):
    import rospy
    from arm_move.srv._box_info_srv import *

    rospy.wait_for_service('det_box_srv')
    try:
        det_box_srv = rospy.ServiceProxy('det_box_srv', box_info_srv)

        pub_msg = box_info_srvRequest()
        pub_msg.object_name.append(box_name)
        pub_msg.object_color.append(box_color)

        pub_msg.object_position.x = box_xyz[0]
        pub_msg.object_position.y = box_xyz[1]
        pub_msg.object_position.z = box_xyz[2]
        pub_msg.object_orientation.x = box_xyzw[0]
        pub_msg.object_orientation.y = box_xyzw[1]
        pub_msg.object_orientation.z = box_xyzw[2]
        pub_msg.object_orientation.w = box_xyzw[3]
        pub_msg.object_scale.x = box_wdh[0]
        pub_msg.object_scale.y = box_wdh[1]
        pub_msg.object_scale.z = box_wdh[2]

        resp1 = det_box_srv(pub_msg)
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def draw_box_client(box_name, box_xyz, box_xyzw, box_wdh, box_color):
    import rospy
    from arm_move.srv._box_info_srv import *

    rospy.wait_for_service('draw_box_srv')
    try:
        draw_box_srv = rospy.ServiceProxy('draw_box_srv', box_info_srv)
        # pub_msg = box_info_msg()
        pub_msg = box_info_srvRequest()
        # pub_srv.header = 0
        pub_msg.object_name.append(box_name)
        pub_msg.object_color.append(box_color)

        pub_msg.object_position.x = box_xyz[0]
        pub_msg.object_position.y = box_xyz[1]
        pub_msg.object_position.z = box_xyz[2]
        pub_msg.object_orientation.x = box_xyzw[0]
        pub_msg.object_orientation.y = box_xyzw[1]
        pub_msg.object_orientation.z = box_xyzw[2]
        pub_msg.object_orientation.w = box_xyzw[3]
        pub_msg.object_scale.x = box_wdh[0]
        pub_msg.object_scale.y = box_wdh[1]
        pub_msg.object_scale.z = box_wdh[2]

        resp1 = draw_box_srv(pub_msg)

    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def move_joints_client_deg(arm_name, jointGoal):
    import rospy
    from arm_move.srv._arm_goalJoint_srv import *

    rospy.wait_for_service('arm_goalJoint_srv')
    try:
        m_joints_srv = rospy.ServiceProxy('arm_goalJoint_srv', arm_goalJoint_srv)
        # pub_msg = box_info_msg()
        pub_msg = arm_goalJoint_srvRequest()
        pub_msg.goalPose.name = [arm_name]
        radGoal = np.deg2rad(jointGoal)
        pub_msg.goalPose.position = radGoal

        resp1 = m_joints_srv(pub_msg)

    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def move_joints_client_rad(arm_name, jointGoal):
    import rospy
    from arm_move.srv._arm_goalJoint_srv import *

    rospy.wait_for_service('arm_goalJoint_srv')
    try:
        m_joints_srv = rospy.ServiceProxy('arm_goalJoint_srv', arm_goalJoint_srv)
        # pub_msg = box_info_msg()
        pub_msg = arm_goalJoint_srvRequest()
        pub_msg.arm_name = [arm_name]
        # radGoal = np.deg2rad(jointGoal)
        pub_msg.goalPose.position = jointGoal
        #print"go to", jointGoal
        resp1 = m_joints_srv(pub_msg)

    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def panda_gripper_open():
    import rospy
    from arm_move.srv._work_start_srv import *
    rospy.wait_for_service('panda_gripper_open_srv')
    try:
        i_joints_srv = rospy.ServiceProxy('panda_gripper_open_srv', work_start_srv)
        # pub_msg = box_info_msg()
        pub_msg = work_start_srvRequest()
        pub_msg.w_start = 1

        resp1 = i_joints_srv(pub_msg)

        print "response", resp1
        if resp1.w_flag == 1:
            print "work done"
            return resp1.w_flag
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def panda_gripper_close():
    import rospy
    from arm_move.srv._work_start_srv import *
    rospy.wait_for_service('panda_gripper_close_srv')
    try:
        i_joints_srv = rospy.ServiceProxy('panda_gripper_close_srv', work_start_srv)
        # pub_msg = box_info_msg()
        pub_msg = work_start_srvRequest()
        pub_msg.w_start =1

        resp1 = i_joints_srv(pub_msg)

        print "response", resp1
        if resp1.w_flag == 1:
            print "work done"
            return resp1.w_flag
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def remove_all():
    import rospy
    from arm_move.srv._work_start_srv import *
    rospy.wait_for_service('remove_all_srv')
    try:
        i_joints_srv = rospy.ServiceProxy('remove_all_srv', work_start_srv)
        # pub_msg = box_info_msg()
        pub_msg = work_start_srvRequest()
        pub_msg.w_start =1

        resp1 = i_joints_srv(pub_msg)

        # print "response", resp1
        if resp1.w_flag == 1:
            # print "work done"
            return resp1.w_flag
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


# if __name__ == '__main__':
#     print "Requesting for adding a box in moveIT"
#     add_box_client()


if __name__ == "__main__":
    print "This file has list of custom made functions"
    remove_all()