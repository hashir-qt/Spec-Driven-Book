from agents import Agent, Runner, function_tool, OpenAIChatCompletionsModel
from agents.run import RunConfig
from app.config.agents_config import AGENTS_CONFIG
from app.config.settings import get_settings
from openai import AsyncOpenAI
import logging

logger = logging.getLogger(__name__)


# Function tools for the troubleshooting agent
@function_tool
def get_diagnostic_commands(category: str) -> str:
    """
    Returns useful diagnostic commands for debugging robotics issues.
    Use to give users specific commands to run for diagnosis.
    
    Args:
        category: Category of issue (e.g., "ros2", "gazebo", "network", "gpu", "system")
    """
    commands = {
        "ros2": """
**ROS 2 Diagnostic Commands:**
```bash
# Check if ROS 2 is sourced
echo $ROS_DISTRO

# List all nodes
ros2 node list

# List all topics
ros2 topic list

# Echo a topic
ros2 topic echo /topic_name

# Check topic info
ros2 topic info /topic_name

# List services
ros2 service list

# View TF tree
ros2 run tf2_tools view_frames
```
""",
        "gazebo": """
**Gazebo Diagnostic Commands:**
```bash
# Check Gazebo installation
gazebo --version

# List model paths
echo $GAZEBO_MODEL_PATH

# Run Gazebo with verbose logging
gazebo --verbose

# Check for graphics issues
glxinfo | grep "OpenGL renderer"
```
""",
        "network": """
**Network Diagnostic Commands:**
```bash
# Check ROS 2 network settings
echo $ROS_DOMAIN_ID

# List network interfaces
ip addr

# Check if DDS is working
ros2 daemon status
ros2 daemon stop
ros2 daemon start
```
""",
        "gpu": """
**GPU Diagnostic Commands:**
```bash
# Check NVIDIA driver
nvidia-smi

# Check CUDA version
nvcc --version

# Check OpenGL
glxinfo | grep "OpenGL"

# For Isaac Sim
echo $ISAACSIM_PATH
```
""",
        "system": """
**System Diagnostic Commands:**
```bash
# Check Ubuntu version
lsb_release -a

# Check Python version
python3 --version

# Check available memory
free -h

# Check disk space
df -h

# Check running processes
htop
```
"""
    }
    
    category_lower = category.lower()
    for key, cmds in commands.items():
        if key in category_lower:
            return cmds
    
    return "Category not recognized. Available categories: ros2, gazebo, network, gpu, system"


@function_tool
def get_common_fixes(issue_type: str) -> str:
    """
    Returns step-by-step fixes for common robotics issues.
    Use when you've identified the likely cause of an issue.
    
    Args:
        issue_type: Type of issue (e.g., "workspace", "permissions", "dependencies", "sourcing")
    """
    fixes = {
        "workspace": """
**Workspace Issues - Step-by-Step Fix:**
1. Navigate to your workspace root: `cd ~/your_ws`
2. Remove old build artifacts: `rm -rf build/ install/ log/`
3. Rebuild: `colcon build --symlink-install`
4. Source the workspace: `source install/setup.bash`
5. Add to bashrc: `echo "source ~/your_ws/install/setup.bash" >> ~/.bashrc`
""",
        "permissions": """
**Permission Issues - Step-by-Step Fix:**
1. Add user to dialout group (for serial ports): `sudo usermod -aG dialout $USER`
2. Add user to video group (for cameras): `sudo usermod -aG video $USER`
3. Log out and log back in for changes to take effect
4. Verify groups: `groups`
""",
        "dependencies": """
**Dependency Issues - Step-by-Step Fix:**
1. Update package lists: `sudo apt update`
2. Install ROS 2 dependencies: `rosdep install --from-paths src --ignore-src -r -y`
3. Install Python deps: `pip install -r requirements.txt`
4. If rosdep not initialized: 
   ```
   sudo rosdep init
   rosdep update
   ```
""",
        "sourcing": """
**Sourcing Issues - Step-by-Step Fix:**
1. Source ROS 2 first: `source /opt/ros/humble/setup.bash`
2. Source your workspace second: `source ~/your_ws/install/setup.bash`
3. Add both to ~/.bashrc:
   ```bash
   echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
   echo "source ~/your_ws/install/setup.bash" >> ~/.bashrc
   ```
4. Reload bashrc: `source ~/.bashrc`
"""
    }
    
    issue_lower = issue_type.lower()
    for key, fix in fixes.items():
        if key in issue_lower:
            return fix
    
    return "Issue type not recognized. Available fixes: workspace, permissions, dependencies, sourcing"


class TroubleshootingAgent:
    def __init__(self):
        self.config = AGENTS_CONFIG["troubleshoot"]
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
            tools=[get_diagnostic_commands, get_common_fixes]
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
        Process query using Troubleshooting persona.
        
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

Issue: {query}

Please help diagnose and resolve this issue with step-by-step solutions.
Use your tools when helpful:
- Use get_diagnostic_commands to provide debugging commands
- Use get_common_fixes for step-by-step solutions
Ask clarifying questions if necessary, and always cite source chapters."""

            result = await Runner.run(
                self.agent,
                user_message,
                run_config=self.run_config
            )
            
            answer = result.final_output
            
            logger.info(f"Troubleshooting Agent generated response: {len(answer)} chars")
            
            return {
                "answer": answer,
                "sources": sources,
                "agent": self.config["name"],
                "agent_type": "troubleshoot"
            }
            
        except Exception as e:
            logger.error(f"Troubleshooting Agent error: {e}")
            raise