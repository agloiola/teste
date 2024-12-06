#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Bool
from actionlib_msgs.msg import GoalStatus

def movebase_client():
    
    rospy.init_node('movebase_client')
    
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    
    goal.target_pose.pose.position.x = 0.5
    goal.target_pose.pose.position.y = 0.3
    goal.target_pose.pose.orientation.w = 1.0

    client.send_goal(goal)  
    client.wait_for_result()
    
    rospy.sleep(1)
    
    if client.get_state() == GoalStatus.SUCCEEDED:
        rospy.loginfo("Robô moveu para a posição desejada")

        rospy.loginfo("Sinal de chegada publicado!")
    else:
        rospy.loginfo("Erro")
  
    rospy.loginfo("Esperando o objeto ser posicionado")
    
	
    rospy.sleep(2)
    goal.target_pose.pose.position.x = 4.0
    goal.target_pose.pose.position.y = 0.0
    goal.target_pose.pose.orientation.w = 1.0

    client.send_goal(goal)  
    client.wait_for_result()
    
    if client.get_state() == GoalStatus.SUCCEEDED:
        rospy.loginfo("Robô carregou o objeto")
    else:
        rospy.loginfo("Erro ao carregar o objeto")
        
    
	
if __name__ == "__main__":
    movebase_client()

