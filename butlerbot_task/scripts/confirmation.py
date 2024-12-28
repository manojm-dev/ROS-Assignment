from rclpy.node import Node
from std_srvs.srv import SetBool
import time

class FoodPlacementConfirmationServer(Node):
    def __init__(self):
        super().__init__('food_placement_confirmation_server')

        # Create a service server to handle confirmation requests
        self.srv = self.create_service(SetBool, 'placed_the_food_confirmation', self.handle_food_confirmation)

    def handle_food_confirmation(self, request, response):
        # Log the received confirmation request
        self.get_logger().info(f"Received confirmation request: {request.data}")

        # Wait for user input before confirming the food placement
        self.get_logger().info("Waiting for user input to confirm food placement...")

        # Simulate waiting for user confirmation in the console
        user_input = input("Has the food been placed? (yes/no): ").strip().lower()

        # Respond based on user input
        if user_input == 'yes':
            self.get_logger().info("Food placement confirmed.")
            response.success = True
        elif user_input == 'no':
            self.get_logger().error("Food placement not confirmed.")
            response.success = False
        else:
            self.get_logger().error("Invalid input. Please enter 'yes' or 'no'.")
            response.success = False

        return response

def main(args=None):
    rclpy.init(args=args)

    food_placement_confirmation_server = FoodPlacementConfirmationServer()

    rclpy.spin(food_placement_confirmation_server)

    food_placement_confirmation_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
