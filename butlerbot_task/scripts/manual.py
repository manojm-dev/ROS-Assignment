#!/usr/bin/env python3

import time
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from butlerbot_interfaces.action import FoodDelivery  # Import custom action from your package
from nav2_msgs.action import NavigateToPose  # Import NavigateToPose action
from nav_msgs.msg import Odometry
import sqlite3

# Function to retrieve goal data by name
def get_goal_by_name(name):
    conn = sqlite3.connect('/home/devuser/workspace/src/butlerbot_task/data/goals.db')
    cursor = conn.cursor()
    
    cursor.execute('SELECT * FROM goals WHERE name=?', (name,))
    row = cursor.fetchone()
    
    conn.close()
    
    if row:
        goal = {
            "name": row[1],  # row[1] is the goal name
            "position": {"x": row[2], "y": row[3], "z": row[4]},  # row[2], row[3], row[4] are position values
            "orientation": {"x": row[5], "y": row[6], "z": row[7], "w": row[8]}  # row[5], row[6], row[7], row[8] are orientation values
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
            'food_delivery',  # Action server name
            self.execute_callback
        )
        
        # Odometry subscriber
        self.current_position = {'x': 0.0, 'y': 0.0}
        self.tolerance = 0.1  # Tolerance for goal check
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',  # Replace with your odometry topic
            self.odom_callback,
            10
        )
        
        self._navigate_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Rename variables to avoid confusion
        self.delivery_feedback = FoodDelivery.Feedback()
        self.navigation_feedback = FoodDelivery.Feedback()
        self.delivery_result = FoodDelivery.Result()
        
        self.delivery_feedback.current_status = f"Idle at home"
        self.get_logger().info("Food delivery action server is ready and is in idle condition")
    
    def odom_callback(self, msg):
        """Callback to update current position using odometry data."""
        self.current_position['x'] = msg.pose.pose.position.x
        self.current_position['y'] = msg.pose.pose.position.y

    def execute_callback(self, goal_handle):
        self.get_logger().info('Beginning food delivery job...')
        old_status = 0

        # Getting goal
        table_number = goal_handle.request.location
        
        # Fetch kitchen location from the database
        kitchen_location = get_goal_by_name("kitchen")
        if not kitchen_location:
            self.get_logger().error("Failed to retrieve kitchen location.")
            self.delivery_result.success = False
            goal_handle.abort(self.delivery_result)
            return self.delivery_result
        
        # Fetch home location from the database
        home_location = get_goal_by_name("home")
        if not home_location:
            self.get_logger().error(f"Failed to retrieve home location")
            self.delivery_result.success = False
            goal_handle.abort(self.delivery_result)
            return self.delivery_result

        # Fetch table location from the database
        table_location = get_goal_by_name(table_number)
        if not table_location:
            self.get_logger().error(f"Failed to retrieve location for {table_number}")
            self.delivery_result.success = False
            goal_handle.abort(self.delivery_result)
            return self.delivery_result

        # Step 1: Navigate to the kitchen
        self.navigate_and_verify(kitchen_location, "Heading to the kitchen", goal_handle)

        # Step 2: Navigate to the table
        self.navigate_and_verify(table_location, f"Delivering food to {table_number}", goal_handle)

        # Step 3: Return to home
        self.navigate_and_verify(home_location, "Returning to home", goal_handle)

        # Task completed
        goal_handle.succeed()
        self.delivery_result.success = True
        self.delivery_feedback.current_status = "Food delivered successfully"
        return self.delivery_result

    def navigate_and_verify(self, location, status_message, goal_handle):
        """Navigate to a specified location and verify using odometry."""
        self.delivery_feedback.current_status = status_message
        goal_handle.publish_feedback(self.delivery_feedback)

        if not self.navigate_to_pose(location):
            self.get_logger().error(f"Failed to reach location: {status_message}")
            self.delivery_result.success = False
            return

    def navigate_to_pose(self, location):
        """Navigate to the specified location and verify using odometry."""
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
        result = future.result()

        if result.status == 4:  # Check for success status
            if self.verify_position(location) == True:
                return

        
    def verify_position(self, location):
        """Verify the robot's position using odometry data."""
        goal_x = float(location['position']['x'])
        goal_y = float(location['position']['y'])
        distance_to_goal = math.sqrt(
            (goal_x - self.current_position['x']) ** 2 +
            (goal_y - self.current_position['y']) ** 2
        )
        self.get_logger().info(f"Distance to goal: {distance_to_goal:.2f} m")
        return distance_to_goal <= self.tolerance


def main(args=None):
    rclpy.init(args=args)
    action_server = FoodDeliveryActionServer()
    rclpy.spin(action_server)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
