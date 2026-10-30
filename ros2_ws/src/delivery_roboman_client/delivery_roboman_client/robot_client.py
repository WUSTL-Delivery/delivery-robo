#!/usr/bin/env python3
"""
ROS2 WebSocket Client for Robot Delivery System
Connects to the authoritative server and handles order matching
"""

import rclpy
from rclpy.node import Node
import websocket
import json
import threading
import time
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

class DeliveryRobotClient(Node):
    def __init__(self):
        super().__init__('delivery_roboman_client')
        
        # Declare parameters
        self.declare_parameter('robot_id', 'robot_001')
        self.declare_parameter('ws_url', 'ws://localhost:8080/ws')
        
        self.robot_id = self.get_parameter('robot_id').value
        self.ws_url = self.get_parameter('ws_url').value
        
        # WebSocket connection
        self.ws = None
        self.connected = False
        self.status = "offline"  # offline, online, ready, delivery
        
        # Publishers
        self.status_pub = self.create_publisher(String, 'robot/status', 10)
        self.order_pub = self.create_publisher(String, 'robot/current_order', 10)
        
        # Subscribers
        self.delivery_complete_sub = self.create_subscription(
            String,
            'robot/delivery_complete',
            self.delivery_complete_callback,
            10
        )
        
        # Timer for status updates
        # self.timer = self.create_timer(1.0, self.timer_callback)
        
        # Connect to WebSocket
        self.connect_websocket()
        
        self.get_logger().info(f'Robot Client initialized: {self.robot_id}')
        self.get_logger().info(f'Connecting to: {self.ws_url}')

    def connect_websocket(self):
        """Connect to the WebSocket server"""
        try:
            self.ws = websocket.WebSocketApp(
                self.ws_url,
                on_open=self.on_open,
                on_message=self.on_message,
                on_error=self.on_error,
                on_close=self.on_close
            )
            self.get_logger().info(f'Robot Client initialized: {self.robot_id}')

            
            # Run WebSocket in a separate thread
            ws_thread = threading.Thread(target=self.ws.run_forever)
            ws_thread.daemon = True
            ws_thread.start()
            
            self.get_logger().info('WebSocket connection initiated')
            
        except Exception as e:
            self.get_logger().error(f'Failed to connect to WebSocket: {e}')

    def on_open(self, ws):
        """Called when WebSocket connection is established"""
        self.connected = True
        self.get_logger().info('WebSocket connected!')
        
        # Send online status
        self.send_status_update("online")

    def on_message(self, ws, message):
        """Handle incoming WebSocket messages"""
        try:
            data = json.loads(message)
            self.get_logger().info(f'Received message: {data}')
            
            # Handle robot match (order assignment)
            if 'RobotID' in data and 'OrderID' in data:
                self.handle_order_match(data)
            
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse message: {e}')
        except Exception as e:
            self.get_logger().error(f'Error handling message: {e}')

    def on_error(self, ws, error):
        """Handle WebSocket errors"""
        self.get_logger().error(f'WebSocket error: {error}')
        self.connected = False

    def on_close(self, ws, close_status_code, close_msg):
        """Called when WebSocket connection is closed"""
        self.get_logger().warn(f'WebSocket closed: {close_status_code} - {close_msg}')
        self.connected = False
        self.status = "offline"
        
        # Attempt to reconnect after 5 seconds
        self.get_logger().info('Attempting to reconnect in 5 seconds...')
        time.sleep(5)
        self.connect_websocket()

    def send_status_update(self, status):
        """Send robot status update to server"""
        if not self.connected:
            self.get_logger().warn('Cannot send status update: not connected')
            return
        
        self.status = status
        
        message = {
            "type": "update",
            "payload": {
                "status": status,
                "robot_id": self.robot_id
            }
        }
        
        try:
            self.ws.send(json.dumps(message))
            self.get_logger().info(f'Sent status update: {status}')
        except Exception as e:
            self.get_logger().error(f'Failed to send status update: {e}')

    def handle_order_match(self, match_data):
        """Handle order assignment from server"""
        robot_id = match_data.get('robot_id')
        order_id = match_data.get('order_id')
        
        if robot_id != self.robot_id:
            self.get_logger().warn(f'Received match for different robot: {robot_id}')
            return
        
        self.get_logger().info(f'🤖 Matched with Order #{order_id}!')
        
        # Update status
        self.status = "delivery"
        
        # Publish order to ROS topic
        order_msg = String()
        order_msg.data = json.dumps({
            'order_id': order_id,
            'robot_id': robot_id,
            'timestamp': time.time()
        })
        self.order_pub.publish(order_msg)
        
        # TODO: Start navigation to pickup location
        # self.navigate_to_pickup(order_id)

    def delivery_complete_callback(self, msg):
        """Called when delivery is complete"""
        try:
            data = json.loads(msg.data)
            order_id = data.get('order_id')
            
            self.get_logger().info(f'✅ Delivery complete for Order #{order_id}')
            
            # Send ready status to get next order
            self.send_status_update("ready")
            
        except Exception as e:
            self.get_logger().error(f'Error handling delivery complete: {e}')

    def timer_callback(self):
        """Periodic status update"""
        # Publish current status
        status_msg = String()
        status_msg.data = json.dumps({
            'robot_id': self.robot_id,
            'status': self.status,
            'connected': self.connected,
            'timestamp': time.time()
        })
        self.status_pub.publish(status_msg)
        
        # If online but not matched, send ready status periodically
        if self.connected and self.status == "online":
            # Could send "ready" status here if you want the robot to be available
            # self.send_status_update("ready")
            pass

    def shutdown(self):
        """Cleanup on shutdown"""
        self.get_logger().info('Shutting down robot client...')
        if self.connected:
            self.send_status_update("shutdown")
            time.sleep(0.5)  # Give time for message to send
            self.ws.close()


def main(args=None):
    rclpy.init(args=args)
    
    node = DeliveryRobotClient()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt detected')
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()