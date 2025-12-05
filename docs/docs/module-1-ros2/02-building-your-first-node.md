---
sidebar_position: 2
---

# Chapter 2: Building Your First Node

In the previous chapter, you were introduced to the fundamental concepts of ROS 2. Now, it's time to get your hands dirty and build your first functional ROS 2 node. This chapter will guide you through creating a simple "publisher" node that broadcasts a message on a topic.

## The Publisher/Subscriber Model

Before we dive into the code, let's revisit the publisher/subscriber model, often called "pub/sub." This is the primary method of communication in ROS 2.

*   A **publisher** is a node that sends out data on a specific topic.
*   A **subscriber** is a node that receives data by listening to that topic.

Any number of publishers can send data to a topic, and any number of subscribers can listen. This decouples the nodes from each other, creating a highly modular and flexible system. A node doesn't need to know which node is sending the data or which node is receiving it; it only needs to know the topic name and the message type.

## Creating a Publisher Node

We will create a Python node that publishes a "Hello, World!" message with a counter to a topic named `chatter`.

Here is the complete code for our publisher node:

```python
# Import the rclpy library for ROS 2 Python development
import rclpy
# Import the Node class to create a ROS 2 node
from rclpy.node import Node
# Import the String message type from the std_msgs package
from std_msgs.msg import String

class SimplePublisher(Node):
    """
    A simple ROS 2 publisher node that publishes a string message.
    """
    def __init__(self):
        # Initialize the Node with the name 'simple_publisher'
        super().__init__('simple_publisher')
        # Create a publisher.
        # Arguments:
        # 1. Message type (String)
        # 2. Topic name ('chatter')
        # 3. QoS profile (queue size of 10)
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        # Create a timer that will call the timer_callback function every 0.5 seconds
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # A counter for our messages
        self.i = 0

    def timer_callback(self):
        # Create a new String message
        msg = String()
        # Set the message data
        msg.data = f'Hello World: {self.i}'
        # Publish the message
        self.publisher_.publish(msg)
        # Log the published message to the console
        self.get_logger().info(f'Publishing: "{msg.data}"')
        # Increment the counter
        self.i += 1

def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)
    # Create an instance of our publisher node
    simple_publisher = SimplePublisher()
    # "Spin" the node to keep it running and processing callbacks
    try:
        rclpy.spin(simple_publisher)
    except KeyboardInterrupt:
        # This block will be executed when you press Ctrl+C
        pass
    finally:
        # Cleanly destroy the node and shut down rclpy
        simple_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Breaking Down the Code

1.  **Imports:** We import `rclpy` and the `Node` class, as well as the `String` message type from `std_msgs.msg`.

2.  **`SimplePublisher` Class:** Our node is defined as a class that inherits from `Node`.

3.  **`__init__` Method:**
    *   `super().__init__('simple_publisher')`: We initialize the parent `Node` class with the name of our node.
    *   `self.create_publisher(String, 'chatter', 10)`: This is the core of our publisher. We create a publisher that sends `String` messages on the `chatter` topic. The `10` is the Quality of Service (QoS) queue size, which we will discuss in a later chapter.
    *   `self.create_timer(...)`: We create a timer that calls `self.timer_callback` every 0.5 seconds. This is a common way to create a node that performs a periodic action.

4.  **`timer_callback` Method:**
    *   This function is executed every time the timer fires.
    *   `msg = String()`: We create an instance of the `String` message.
    *   `msg.data = ...`: We set the `data` field of the message.
    *   `self.publisher_.publish(msg)`: We publish the message to the `chatter` topic.
    *   `self.get_logger().info(...)`: We log the published message to the console for debugging.

5.  **`main` Function:**
    *   `rclpy.init()`: Initializes the ROS 2 Python client library.
    *   `simple_publisher = SimplePublisher()`: Creates an instance of our node.
    *   `rclpy.spin(simple_publisher)`: This function keeps the node running, processing any events or callbacks (like our timer). The node will continue to run until the `spin` function is interrupted, for example, by pressing `Ctrl+C` in the terminal.
    *   `destroy_node()` and `shutdown()`: These functions are called to cleanly shut down the node and the `rclpy` library.

## Running Your Publisher Node

1.  Save the code above as `simple_publisher.py`.
2.  Open a new terminal and make sure your ROS 2 environment is sourced.
3.  Run the node with the command:
    ```bash
    python simple_publisher.py
    ```
    You should see the output `Publishing: "Hello World: 0"`, `Publishing: "Hello World: 1"`, and so on, printed every half-second.

Congratulations! You have successfully created and run your first ROS 2 publisher node. However, it's publishing into a void. In the next chapter, we will create a subscriber node to receive these messages.
