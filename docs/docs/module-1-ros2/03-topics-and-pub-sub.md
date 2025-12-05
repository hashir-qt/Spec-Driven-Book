---
sidebar_position: 3
---

# Chapter 3: Topics and the Publisher/Subscriber Model

In the last chapter, you created a publisher node that broadcasts messages to a topic. While useful, it's like shouting into an empty room. To make our system do something meaningful, we need another node to listen to those messages. This chapter will introduce the other half of the equation: the subscriber node.

## Creating a Subscriber Node

A subscriber node listens to a specific topic and processes the messages it receives. We will create a subscriber that listens to the `chatter` topic and prints the "Hello, World!" messages from our publisher node to the console.

Here is the complete code for our subscriber node:

```python
# Import the rclpy library for ROS 2 Python development
import rclpy
# Import the Node class to create a ROS 2 node
from rclpy.node import Node
# Import the String message type from the std_msgs package
from std_msgs.msg import String

class SimpleSubscriber(Node):
    """
    A simple ROS 2 subscriber node that listens to a string message.
    """
    def __init__(self):
        # Initialize the Node with the name 'simple_subscriber'
        super().__init__('simple_subscriber')
        # Create a subscriber.
        # Arguments:
        # 1. Message type (String)
        # 2. Topic name ('chatter')
        # 3. Callback function (self.listener_callback)
        # 4. QoS profile (queue size of 10)
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)
        # The following line is not strictly necessary, but it prevents
        # a "used-before-assignment" warning in some linters.
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # This function is called every time a message is received.
        # The received message is passed as an argument.
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)
    # Create an instance of our subscriber node
    simple_subscriber = SimpleSubscriber()
    # "Spin" the node to keep it running and processing callbacks
    try:
        rclpy.spin(simple_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanly destroy the node and shut down rclpy
        simple_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Breaking Down the Code

1.  **Imports:** Same as our publisher, we import `rclpy`, `Node`, and the `String` message type.

2.  **`SimpleSubscriber` Class:** Our subscriber node class, inheriting from `Node`.

3.  **`__init__` Method:**
    *   `super().__init__('simple_subscriber')`: We initialize the parent `Node` with our node's name.
    *   `self.create_subscription(...)`: This is where the magic happens. We create a subscription to the `chatter` topic.
        *   The message type `String` must match the publisher's message type.
        *   The topic name `chatter` must match the publisher's topic name.
        *   `self.listener_callback` is the function that will be called every time a message is received on the `chatter` topic.
        *   `10` is the QoS queue size.

4.  **`listener_callback` Method:**
    *   This function is our event handler for incoming messages.
    *   The `msg` parameter is the `String` message object that was received.
    *   `self.get_logger().info(...)`: We log the `data` field of the received message to the console.

5.  **`main` Function:** The `main` function is identical to our publisher's. It initializes `rclpy`, creates the node, and spins it to keep it running.

## Running the Publisher and Subscriber

To see the pub/sub model in action, you need to run both the publisher and subscriber nodes at the same time.

1.  **Save the code:** Save the subscriber code above as `simple_subscriber.py`. Make sure it is in the same directory as your `simple_publisher.py` from the previous chapter.

2.  **Open two terminals:** You will need two separate terminals. In both terminals, make sure your ROS 2 environment is sourced.

3.  **Run the publisher:** In the first terminal, run the publisher node:
    ```bash
    python simple_publisher.py
    ```
    You should see it start publishing "Hello, World!" messages.

4.  **Run the subscriber:** In the second terminal, run the subscriber node:
    ```bash
    python simple_subscriber.py
    ```
    Almost instantly, you should see the subscriber start printing the messages it hears from the publisher: `I heard: "Hello World: ..."`

You have now created a complete, albeit simple, robotic application! This fundamental pub/sub pattern is used to build almost every ROS 2 system, from simple robots to complex autonomous vehicles.

## What's Next?

You've learned how to create nodes that communicate using topics. In the next chapter, we'll explore another communication pattern: services. You'll learn how to create a service server and a client to perform request/reply interactions.
