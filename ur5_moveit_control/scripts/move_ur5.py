import sys
import rospy
import moveit_commander
import math
'''
class MoveRobotNode():
    """MoveRobotNode"""

    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_robot_node", anonymous=True)

        # Inicialización de MoveIt
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()

        group_name = "manipulator"
        self.move_group = moveit_commander.MoveGroupCommander(group_name)

    def go_to_joint_state(self):
        move_group = self.move_group

        # Definir tau como 2π
        tau = 2 * math.pi

        # Obtener el estado actual de las articulaciones y modificarlo
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = -tau / 8
        joint_goal[2] = 0
        joint_goal[3] = -tau / 4
        joint_goal[4] = 0
        joint_goal[5] = tau / 6  # 1/6 de una vuelta

        # Enviar el objetivo de articulaciones al robot
        success = move_group.go(joint_goal, wait=True)

        # Detener cualquier movimiento residual
        move_group.stop()

        # Limpiar los objetivos de pose
        move_group.clear_pose_targets()

        # Verificar si el movimiento fue exitoso
        if success:
            rospy.loginfo("Movement succeeded")
        else:
            rospy.logwarn("Movement failed")


if __name__ == "__main__":
    robot_control = MoveRobotNode()
    robot_control.go_to_joint_state()
'''

class MoveRobotNode():
    """MoveRobotNode"""

    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_robot_node", anonymous=True)

        # Inicialización de MoveIt
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()

        group_name = "manipulator"
        self.move_group = moveit_commander.MoveGroupCommander(group_name)

    def go_to_joint_state(self):
        move_group = self.move_group

        # Definir tau como 2π
        tau = 2 * math.pi

        # Obtener el estado actual de las articulaciones y modificarlo
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = -math.pi/2
        joint_goal[2] = 0
        joint_goal[3] = 0
        joint_goal[4] = 0
        joint_goal[5] = 0  # 1/6 de una vuelta

        # Enviar el objetivo de articulaciones al robot
        success = move_group.go(joint_goal, wait=True)

        # Detener cualquier movimiento residual
        move_group.stop()

        # Limpiar los objetivos de pose
        move_group.clear_pose_targets()

        # Verificar si el movimiento fue exitoso
        if success:
            rospy.loginfo("Movement succeeded")
        else:
            rospy.logwarn("Movement failed")


if __name__ == "__main__":
    robot_control = MoveRobotNode()
    robot_control.go_to_joint_state()

'''
#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
from math import pi
from geometry_msgs.msg import Pose
import moveit_msgs.msg

def cartesian_path():
    # Initialize moveit_commander and rospy nodes
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('ur5_cartesian_path', anonymous=True)

    # Instantiate a RobotCommander object (interface to robot's kinematic model)
    robot = moveit_commander.RobotCommander()

    # Instantiate a MoveGroupCommander object (interface to the manipulator group)
    group_name = "manipulator"  # Default planning group name for UR5
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # Get the current pose of the end-effector
    start_pose = move_group.get_current_pose().pose

    # Define waypoints for the Cartesian path
    waypoints = []

    # Start from the current pose
    wpose = copy.deepcopy(start_pose)

    # First waypoint: move 0.1 m along the X-axis
    wpose.position.x += 0.1
    waypoints.append(copy.deepcopy(wpose))

    # Second waypoint: move 0.1 m along the Y-axis
    wpose.position.y += 0.1
    waypoints.append(copy.deepcopy(wpose))

    # Third waypoint: move 0.1 m along the Z-axis
    wpose.position.z += 0.1
    waypoints.append(copy.deepcopy(wpose))

    # Compute the Cartesian path connecting the waypoints
    (plan, fraction) = move_group.compute_cartesian_path(
                        waypoints,          # Waypoints to follow
                        eef_step=0.01      # End-effector step size
                        ) # Jump threshold

    # Execute the planned path
    move_group.execute(plan, wait=True)

    # Shut down MoveIt cleanly
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        cartesian_path()
    except rospy.ROSInterruptException:
        pass
'''