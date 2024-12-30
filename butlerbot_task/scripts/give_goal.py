#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from butlerbot_interfaces.action import FoodDelivery  # Ensure this matches the actual action definition


class FoodDeliveryActionClient(Node):
    def __init__(self):
        super().__init__('food_delivery_action_client')
        self._action_client = ActionClient(self, FoodDelivery, 'food_delivery')

    def send_goal(self, location):
        self.get_logger().info(f"Sending goal to deliver food to {location}")

        # Set the location in the goal message
        goal_msg = FoodDelivery.Goal()
        goal_msg.location = location

        # Wait for the action server to be available
        self._action_client.wait_for_server()

        # Send the goal asynchronously and define the callbacks
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        # Request the result asynchronously
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        # Update the feedback with the current status
        current_status = feedback_msg.feedback.current_status
        self.get_logger().info(f"Feedback: {current_status}")

    def get_result_callback(self, future):
        # Access the result object returned by the action server
        result = future.result().result
        if result.success:
            self.get_logger().info("Delivery was successful!")
        else:
            self.get_logger().info("Delivery failed.")
        rclpy.shutdown()

    def send_goals_for_tables(self, tables):
        self.send_goal(tables)
        rclpy.spin(self)


def main(args=None):
    rclpy.init(args=args)
    
    # Ensure at least one table argument is provided
    if len(sys.argv) < 2:
        print("Usage: ros2 run butlerbot_task give_goal.py <table1> [<table2> ... <tableN>]")
        print("Options: table1, table2, table3")
        sys.exit(1)
    
    tables = sys.argv[1:]
    valid_tables = ['table1', 'table2', 'table3']
    invalid_tables = []
    for table in tables:
        if table not in valid_tables:
            invalid_tables.append(table)
    
    if invalid_tables:
        print(f"Invalid table(s) {', '.join(invalid_tables)}. Valid options: {', '.join(valid_tables)}")
        sys.exit(1)
    
    action_client = FoodDeliveryActionClient()

    # Wait for the action server to be available
    while not action_client._action_client.wait_for_server(timeout_sec=1.0):
        action_client.get_logger().info('Waiting for action server to be available...')

    # Send goals for the valid tables
    action_client.send_goals_for_tables(tables)

    rclpy.shutdown()


if __name__ == '__main__':
    main()