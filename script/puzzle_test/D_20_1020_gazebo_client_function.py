import os
import rospy
#from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.msg import *
from gazebo_msgs.srv import *
from geometry_msgs.msg import Pose

HOME_PATH     = os.path.expanduser('~')
MESH_DIR_PATH = HOME_PATH+"/.gazebo/models/gazebo_meshes/"



def spawn_gazebo_model(model_name, model_pose) : 
    
    initial_pose = Pose()
    initial_pose.position.x = model_pose[0]
    initial_pose.position.y = model_pose[1]
    initial_pose.position.z = model_pose[2]
  
    f = open( MESH_DIR_PATH + model_name + '/model.sdf','r')
    sdff = f.read()

    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    spawn_model_prox = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    spawn_model_prox(model_name, sdff, "", initial_pose, "world")


def delete_gazebo_model(model_name) : 
    
    rospy.wait_for_service('/gazebo/delete_model')
    
    try:
        delete_model_prox = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        
        pub_msg = DeleteModelRequest() 
        pub_msg.model_name = model_name
        
        resp = delete_model_prox(pub_msg) 

    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def change_gazebo_model_position(model_name, model_pose) :

    set_model_prox = rospy.wait_for_service('/gazebo/set_model_state') 

    state_msg = ModelState()
    state_msg.model_name = model_name
    state_msg.pose.position.x = model_pose[0]
    state_msg.pose.position.y = model_pose[1]
    state_msg.pose.position.z = model_pose[2]
    state_msg.pose.orientation.x = 0
    state_msg.pose.orientation.y = 0
    state_msg.pose.orientation.z = 0
    state_msg.pose.orientation.w = 1
   
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        resp = set_state( state_msg ) 

    except rospy.ServiceException, e:
        print "change_gazebo_model_position Service call failed: %s" % e



def attach_object_on_gazebo_gripper(model_name_1, link_name_1, model_name_2, link_name_2):
    import rospy
    from gazebo_ros_link_attacher.srv import *
    
    rospy.wait_for_service('/link_attacher_node/attach')

    try:
        attach_object_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
       
        pub_msg = AttachRequest()
        pub_msg.model_name_1 = model_name_1
        pub_msg.link_name_1 = link_name_1
        
        pub_msg.model_name_2 = model_name_2
        pub_msg.link_name_2  = link_name_2
        
        resp1 = attach_object_srv(pub_msg)
    
    except rospy.ServiceException, e:
        print "attach_object_on_gazebo_gripper Service call failed: %s" % e


def detach_object_on_gazebo_gripper(model_name_1, link_name_1, model_name_2, link_name_2):
    import rospy
    from gazebo_ros_link_attacher.srv import *
    
    rospy.wait_for_service('/link_attacher_node/detach')

    try:
        detach_object_srv = rospy.ServiceProxy('/link_attacher_node/detach', Attach)
       
        pub_msg = AttachRequest()
        pub_msg.model_name_1 = model_name_1
        pub_msg.link_name_1 = link_name_1
        
        pub_msg.model_name_2 = model_name_2
        pub_msg.link_name_2  = link_name_2

        resp1 = detach_object_srv(pub_msg)

    except rospy.ServiceException, e:
        print "detach_object_on_gazebo_gripper Service call failed: %s" % e    