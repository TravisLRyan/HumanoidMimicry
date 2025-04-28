import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # Change this to match your topic type
from http.server import SimpleHTTPRequestHandler, HTTPServer
import threading
import asyncio
import websockets

# Global variable to store the latest data
latest_data = {}

class WebServerNode(Node):
    def __init__(self):
        super().__init__("web_server_node")

        # Declare and get the port parameter
        self.declare_parameter("port", 8080)
        self.port = self.get_parameter("port").value

        # Start the HTTP server to serve the webpage
        self.server_thread = threading.Thread(target=self.start_http_server, daemon=True)
        self.server_thread.start()

        # Start the WebSocket server
        self.websocket_thread = threading.Thread(target=self.start_websocket_server, daemon=True)
        self.websocket_thread.start()

        # ROS 2 Subscribers (add more as needed)
        self.subscription = self.create_subscription(
            String,  # Change this to your message type
            "robot_status",  # Change to your topic name
            self.topic_callback,
            10
        )

        self.get_logger().info(f"Web server running at http://localhost:{self.port}")

    def topic_callback(self, msg):
        """Update latest data when a new message is received."""
        global latest_data
        latest_data["robot_status"] = msg.data  # Store latest message
        self.get_logger().info(f"Received: {msg.data}")

    def start_http_server(self):
        """Starts an HTTP server to serve static files."""
        handler = SimpleHTTPRequestHandler
        httpd = HTTPServer(("0.0.0.0", self.port), handler)
        self.get_logger().info(f"Serving HTTP on port {self.port}...")
        httpd.serve_forever()

    def start_websocket_server(self):
        """Starts a WebSocket server to send ROS data to the webpage."""
        async def websocket_handler(websocket, path):
            while True:
                await websocket.send(str(latest_data))  # Send the latest topic data
                await asyncio.sleep(1)  # Send data every second

        asyncio.set_event_loop(asyncio.new_event_loop())
        start_server = websockets.serve(websocket_handler, "0.0.0.0", 8765)
        asyncio.get_event_loop().run_until_complete(start_server)
        asyncio.get_event_loop().run_forever()


def main(args=None):
    rclpy.init(args=args)
    node = WebServerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

