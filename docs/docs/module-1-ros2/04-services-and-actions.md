--- 
sidebar_position: 4
---

# Chapter 4: Services and Actions

So far, we've explored topics, which are great for continuous data streams. But what if you need a two-way conversation? What if you need to call a function on another node and get a result back? This is where **services** and **actions** come in.

## Services: The Request/Reply Model

A service is a request/reply communication pattern. It's defined by a pair of messages: one for the request and one for the response. A node can advertise a service (becoming a **service server**), and another node can call that service (becoming a **service client**).

This is ideal for tasks that have a clear beginning and end, like "capture an image," "calculate a path," or "add two numbers."

### Creating a Service Server

Let's create a service that adds two integers. First, we need a service definition. ROS 2 comes with a standard service definition for this purpose: `example_interfaces/srv/AddTwoInts`. It looks like this:

```
int64 a
int64 b
---
int64 sum
```
The three dashes (`---`) separate the request from the response. The request has two integers, `a` and `b`, and the response has one integer, `sum`.

Here's the code for our service server:

```python
# Import the rclpy library
import rclpy
# Import the Node class
from rclpy.node import Node
# Import the AddTwoInts service type
from example_interfaces.srv import AddTwoInts

class SimpleServiceServer(Node):
    """
    A simple ROS 2 service server that adds two integers.
    """
    def __init__(self):
        super().__init__('simple_service_server')
        # Create a service.
        # Arguments:
        # 1. Service type (AddTwoInts)
        # 2. Service name ('add_two_ints')
        # 3. Callback function (self.add_two_ints_callback)
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        # This function is called every time the service is called.
        # The request and response objects are passed as arguments.
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request\na: {request.a} b: {request.b}')
        self.get_logger().info(f'Sending back response: {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    simple_service_server = SimpleServiceServer()
    try:
        rclpy.spin(simple_service_server)
    except KeyboardInterrupt:
        pass
    finally:
        simple_service_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Creating a Service Client

Now, let's create a client to call our service. The client will send two numbers to the server and wait for the sum.

```python
import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class SimpleServiceClient(Node):
    """
    A simple ROS 2 service client that calls the add_two_ints service.
    """
    def __init__(self):
        super().__init__('simple_service_client')
        # Create a client.
        # Arguments:
        # 1. Service type (AddTwoInts)
        # 2. Service name ('add_two_ints')
        self.client = self.create_client(AddTwoInts, 'add_two_ints')
        # Wait until a service is available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        # Call the service asynchronously
        self.future = self.client.call_async(self.req)
        # Wait for the future to be complete
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    # Check for the correct number of command-line arguments
    if len(sys.argv) != 3:
        print("Usage: python simple_service_client.py <int1> <int2>")
        return

    simple_service_client = SimpleServiceClient()
    response = simple_service_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    simple_service_client.get_logger().info(
        f'Result of add_two_ints: for {sys.argv[1]} + {sys.argv[2]} = {response.sum}')
    simple_service_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Running the Service and Client

1.  **Save the files:** Save the server code as `simple_service_server.py` and the client code as `simple_service_client.py`.

2.  **Open two terminals** and source your ROS 2 environment in both.

3.  **Run the server:** In the first terminal, run the service server:
    ```bash
    python simple_service_server.py
    ```

4.  **Run the client:** In the second terminal, run the service client with two numbers as arguments:
    ```bash
    python simple_service_client.py 5 10
    ```
    The client will send `5` and `10` to the server, which will add them and send back the result. The client will then print `Result of add_two_ints: for 5 + 10 = 15`.

## Actions: For Long-Running Tasks

What if a task takes a long time to complete? With a service, the client would be blocked, waiting for the result. **Actions** solve this problem by providing a more complex communication pattern for long-running, asynchronous tasks.

An action has three parts:
1.  **A goal:** The request to start the task.
2.  **Feedback:** Regular updates on the task's progress.
3.  **A result:** The final outcome of the task.

Think of an action like ordering a pizza.
*   **Goal:** You tell the pizza shop what you want.
*   **Feedback:** You can track the pizza's progress (baking, out for delivery).
*   **Result:** The pizza arrives at your door.

You can also cancel the goal (e.g., if you decide you want a different pizza).

Actions are more complex to implement than services, but they are essential for tasks like navigation, where a robot might take several minutes to reach a destination and you want to receive updates along the way.

We will explore actions in more detail in later chapters when we work with more complex robotic behaviors.

## What You've Learned

In this module, you've learned the fundamental building blocks of ROS 2:
*   **Nodes:** The basic units of computation.
*   **Topics:** For one-way, many-to-many communication.
*   **Services:** For two-way, request/reply communication.
*   **Actions:** For long-running tasks with feedback.

With these concepts, you can build a wide variety of robotic applications.

## What's Next?

In the next module, we will move from the abstract world of nodes and topics to the virtual world of simulation. You will learn how to use **Gazebo**, a powerful robotics simulator, to create and test your ROS 2 applications in a virtual environment before deploying them on a real robot.
