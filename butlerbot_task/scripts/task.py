#!/usr/bin/env python3

from time import sleep
from threading import Thread
import sqlite3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.duration import Duration

from butlerbot_interfaces.action import FoodDelivery
from butlerbot_interfaces.srv import Confirm
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# Function to retrieve goal data by name
def get_goal_by_name(name):
    conn = sqlite3.connect('/home/devuser/workspace/src/butlerbot_task/data/goals.db')
    cursor = conn.cursor()

    cursor.execute('SELECT * FROM goals WHERE name=?', (name,))
    row = cursor.fetchone()

    conn.close()

    if row:
        goal = {
            "name": row[1],
            "position": {"x": row[2], "y": row[3], "z": row[4]},
            "orientation": {"x": row[5], "y": row[6], "z": row[7], "w": row[8]}
        }
        return goal
    else:
        return None


class FoodDeliveryActionServer(Node):
    def __init__(self):
        super().__init__('food_delivery_action_server')
        self._action_server = ActionServer(
            self,
            FoodDelivery,
            'food_delivery',
            self.execute_callback
        )

        # Initialize navigation client
        self._navigate_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.feedback = FoodDelivery.Feedback()
        self.result = FoodDelivery.Result()
        self.get_logger().info("Food delivery action server is ready")

    def execute_callback(self, goal_handle):
        self.get_logger().info("Starting food delivery...")
        tables = goal_handle.request.location
        self.cancellation = False

        # Fetch locations
        kitchen_location = get_goal_by_name("kitchen")
        home_location = get_goal_by_name("home")

        if not (kitchen_location and home_location):
            self.get_logger().error("Failed to retrieve one or more locations.")
            self.result.success = False
            goal_handle.abort()
            return self.result

        # Navigate to kitchen
        self.feedback.current_status = "Heading to the kitchen"
        goal_handle.publish_feedback(self.feedback)
        if not self.navigate_to_pose(kitchen_location):
            self.get_logger().error("Failed to navigate to kitchen")
            self.result.success = False
            goal_handle.abort()
            return self.result

        # Wait for external confirmation after reaching kitchen with timeout
        self.feedback.current_status = "Waiting for confirmation at kitchen"
        goal_handle.publish_feedback(self.feedback)
        if not self.wait_for_confirmation('kitchen', timeout=5):
            self.get_logger().info("Timeout reached at kitchen, returning home")
            self.navigate_to_pose(home_location)
            self.result.success = False
            goal_handle.abort()
            return self.result
        
        for table_number in tables:
            # get table location
            table_location = get_goal_by_name(table_number)
            
            if not table_location:
                self.get_logger().error(f"Failed to retrieve location of {table_number}")
                self.result.success = False
                goal_handle.abort()
                return self.result

            # Proceed to deliver food to table
            self.feedback.current_status = f"Delivering food to {table_number}"
            goal_handle.publish_feedback(self.feedback)
            if not self.navigate_to_pose(table_location):
                self.get_logger().error(f"Failed to navigate to {table_number}")
                self.navigate_to_pose(home_location)
                self.result.success = False
                goal_handle.abort()
                return self.result
            
            # Wait for external confirmation after reaching table with timeout
            self.feedback.current_status = f"Waiting for confirmation at {table_number}"
            goal_handle.publish_feedback(self.feedback)
            if not self.wait_for_confirmation(table_number, timeout=5):
                self.get_logger().info("No confirmation at table, moving to next table")
    
          
        if self.cancellation == True:  
            # Wait for external confirmation after reaching table with timeout
            self.feedback.current_status = f"Moving to kitchen"
            goal_handle.publish_feedback(self.feedback)
            if not self.navigate_to_pose(kitchen_location):
                self.get_logger().error("Failed to navigate back to kitchen")
                self.navigate_to_pose(home_location)
                self.result.success = False
                goal_handle.abort()
                return self.result

        # Finally, return to home
        self.feedback.current_status = "Returning to home"
        goal_handle.publish_feedback(self.feedback)
        if not self.navigate_to_pose(home_location):
            self.get_logger().error("Failed to return to home")
            self.result.success = False
            goal_handle.abort()
            return self.result

        self.feedback.current_status = "Food delivered successfully"
        goal_handle.publish_feedback(self.feedback)
        self.result.success = True
        goal_handle.succeed()
        return self.result

    def wait_for_confirmation(self, location, timeout=30):
        confirmation_received = False

        def get_user_input():
            nonlocal confirmation_received
            confirmation = input(f'Waiting for confirmation at {location}: (y/n)')
            if confirmation.lower() == 'y':
                confirmation_received = True
                self.get_logger().info("Confirmed")

        # Start a thread to handle user input
        input_thread = Thread(target=get_user_input)
        input_thread.start()

        # Check for timeout
        start_time = self.get_clock().now()
        while (self.get_clock().now() - start_time) < Duration(seconds=timeout):
            if confirmation_received:
                return True
            sleep(0.1)  # Small delay to avoid busy waiting

        self.cancellation = True
        self.get_logger().error("Order cancelled or Confirmation timed out.")
        return False
    

    def navigate_to_pose(self, location):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'

        goal_msg.pose.pose.position.x = float(location['position']['x'])
        goal_msg.pose.pose.position.y = float(location['position']['y'])
        goal_msg.pose.pose.position.z = float(location['position']['z'])
        goal_msg.pose.pose.orientation.x = float(location['orientation']['x'])
        goal_msg.pose.pose.orientation.y = float(location['orientation']['y'])
        goal_msg.pose.pose.orientation.z = float(location['orientation']['z'])
        goal_msg.pose.pose.orientation.w = float(location['orientation']['w'])

        self._navigate_client.wait_for_server()

        future = self._navigate_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Navigation goal was not accepted.")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result()
        if result.status != 4:  # SUCCEEDED
            self.get_logger().error("Navigation failed.")
            return False

        return True


def main(args=None):
    rclpy.init(args=args)
    action_server = FoodDeliveryActionServer()
    rclpy.spin(action_server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
