import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class RobotMoveBaseClient:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        self.pose_subscriber = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.pose_cb)
        self.current_pose = None
        self.state = 'IDLE'
        self.feedback_counter = 0  # Ajouter un compteur de feedback

    def feedback_cb(self, feedback):
        self.feedback_counter += 1
        if self.feedback_counter % 10 == 0:  # Remplacer 10 par le nombre de messages de feedback que vous voulez ignorer
            rospy.loginfo('Robot is currently at position: {}'.format(feedback.base_position.pose))

    
    def pose_cb(self, msg):
        # Cette fonction est appelée lorsque le robot envoie une nouvelle position
        self.current_pose = msg.pose.pose

    def done_cb(self, status, result):
        # Cette fonction est appelée lorsque le serveur d'action a terminé l'action
        if status == actionlib.GoalStatus.SUCCEEDED and self.current_pose is not None:
            rospy.loginfo('Robot arrived at position x=%f, y=%f' % (self.current_pose.position.x, self.current_pose.position.y))
            self.state = 'ARRIVED'
        else:
            rospy.loginfo('The robot failed to reach the position')
            self.state = 'FAILED'

    def move_to_goal(self, x_goal, y_goal):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x_goal
        goal.target_pose.pose.position.y = y_goal
        goal.target_pose.pose.orientation.w = 1.0

        self.client.send_goal(goal, done_cb=self.done_cb, feedback_cb=self.feedback_cb)

        self.state = 'MOVING'

        self.client.wait_for_result()

    def get_state(self):
        return self.state

if __name__ == '__main__':
    rospy.init_node('move_base_client_py')
    robot_move_base_client = RobotMoveBaseClient()
    try:
        while True:
            x_goal = float(input("Enter the x coordinate of the new goal: "))
            y_goal = float(input("Enter the y coordinate of the new goal: "))
            robot_move_base_client.move_to_goal(x_goal, y_goal)
            while robot_move_base_client.get_state() == 'MOVING':
                rospy.sleep(1)  # Attendez que le robot atteigne l'objectif ou échoue.
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
