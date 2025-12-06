from agents import Agent, Runner, function_tool, OpenAIChatCompletionsModel
from agents.run import RunConfig
from app.config.agents_config import AGENTS_CONFIG
from app.config.settings import get_settings
from openai import AsyncOpenAI
import logging

logger = logging.getLogger(__name__)


# Function tools for the code helper agent
@function_tool
def generate_code_template(template_type: str) -> str:
    """
    Generates a code template for common ROS 2 patterns.
    Use when the user needs to start a new component.
    
    Args:
        template_type: Type of template (e.g., "publisher", "subscriber", "service", "action", "launch")
    """
    templates = {
        "publisher": '''#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    node = MinimalPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
''',
        "subscriber": '''#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = MinimalSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
''',
        "service": '''#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(
            AddTwoInts, 
            'add_two_ints', 
            self.add_callback)

    def add_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Request: {request.a} + {request.b} = {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = MinimalService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
''',
        "launch": '''from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_package',
            executable='my_node',
            name='my_node',
            output='screen',
            parameters=[{
                'param_name': 'param_value'
            }]
        ),
    ])
'''
    }
    
    template_lower = template_type.lower()
    for key, template in templates.items():
        if key in template_lower:
            return f"```python\n{template}\n```"
    
    return "Template not found. Available templates: publisher, subscriber, service, launch"


@function_tool
def explain_common_error(error_message: str) -> str:
    """
    Provides explanation and solution for common ROS 2 and robotics errors.
    Use when the user shares an error message.
    
    Args:
        error_message: The error message the user is seeing
    """
    error_solutions = {
        "ModuleNotFoundError": "This usually means a Python package isn't installed or the workspace isn't sourced. Try: `source install/setup.bash` and `pip install <package>`",
        "could not find package": "The package isn't built or CMakeLists.txt has issues. Try: `colcon build --packages-select <package>` and source the workspace.",
        "no such file or directory": "File path is wrong or file doesn't exist. Check your file paths and make sure the file was created.",
        "topic not found": "Publisher might not be running, or topic name has a typo. Use `ros2 topic list` to see active topics.",
        "service not available": "Service server isn't running. Start the service node first, or check if it crashed.",
        "transform exception": "TF tree is broken. Use `ros2 run tf2_tools view_frames` to visualize the tree and find the break.",
        "gazebo error": "Common Gazebo issues: Check GAZEBO_MODEL_PATH, ensure meshes exist, verify URDF/SDF syntax.",
    }
    
    error_lower = error_message.lower()
    for key, solution in error_solutions.items():
        if key.lower() in error_lower:
            return f"**{key}**: {solution}"
    
    return "Error not recognized. Please share the full error message and traceback for better assistance."


class CodeHelperAgent:
    def __init__(self):
        self.config = AGENTS_CONFIG["code_helper"]
        self.settings = get_settings()
        
        # Create OpenAI client for Gemini
        self.client = AsyncOpenAI(
            api_key=self.settings.GEMINI_API_KEY,
            base_url=self.settings.GEMINI_BASE_URL
        )

        # Explicitly use OpenAIChatCompletionsModel
        self.model = OpenAIChatCompletionsModel(
            model=self.settings.GEMINI_MODEL,
            openai_client=self.client
        )
        
        # Create agent with function tools
        self.agent = Agent(
            name=self.config["name"],
            model=self.model,
            instructions=self.config["system_prompt"],
            tools=[generate_code_template, explain_common_error]
        )

        self.run_config = RunConfig(
            model=self.model,
            tracing_disabled=True
        )
    
    async def process_query(
        self,
        query: str,
        rag_context: str,
        sources: list[dict]
    ) -> dict:
        """
        Process query using Code Helper persona.
        
        Args:
            query: User's question
            rag_context: Retrieved chapter excerpts from RAG
            sources: List of source metadata
        
        Returns:
            dict with answer, sources, agent info
        """
        try:
            user_message = f"""Context from book chapters:

{rag_context}

---

Question: {query}

Please provide a code example or help debug based on the provided context.
Use your tools when helpful:
- Use generate_code_template for starter code patterns
- Use explain_common_error when the user shares an error
Always format code in ```python blocks and cite source chapters."""

            result = await Runner.run(
                self.agent,
                user_message,
                run_config=self.run_config
            )
            
            answer = result.final_output
            
            logger.info(f"Code Helper Agent generated response: {len(answer)} chars")
            
            return {
                "answer": answer,
                "sources": sources,
                "agent": self.config["name"],
                "agent_type": "code_helper"
            }
            
        except Exception as e:
            logger.error(f"Code Helper Agent error: {e}")
            raise