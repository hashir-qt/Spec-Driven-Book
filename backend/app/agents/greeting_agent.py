from agents import Agent, Runner, function_tool, OpenAIChatCompletionsModel
from agents.run import RunConfig
from app.config.agents_config import AGENTS_CONFIG
from app.config.settings import get_settings
from openai import AsyncOpenAI
import logging

logger = logging.getLogger(__name__)


# Function tools for the greeting agent
@function_tool
def get_book_overview() -> str:
    """
    Returns an overview of the Physical AI Humanoid Robotics book.
    Use this when the user asks about what the book covers or its contents.
    """
    return """
    The Physical AI Humanoid Robotics book covers:
    
    📚 **Module 1: ROS 2 Fundamentals**
    - Introduction to Robot Operating System 2
    - Nodes, Topics, Services, and Actions
    - Building your first ROS 2 packages
    
    📚 **Module 2: Gazebo Simulation**
    - Setting up simulation environments
    - Robot modeling with URDF/SDF
    - Sensors and actuators simulation
    
    📚 **Module 3: NVIDIA Isaac Platform**
    - Isaac Sim for photorealistic simulation
    - Isaac ROS for accelerated perception
    - Training AI models for robotics
    
    📚 **Module 4: Humanoid Robotics**
    - Bipedal locomotion and balance
    - Human-robot interaction
    - Full-body motion planning
    
    """


@function_tool
def get_chapter_recommendations(interest_area: str) -> str:
    """
    Recommends specific chapters based on the user's area of interest.
    
    Args:
        interest_area: The topic the user is interested in (e.g., "simulation", "ros2", "isaac", "locomotion")
    """
    recommendations = {
        "ros2": "Start with Module 1: ROS 2 Fundamentals - covers nodes, topics, services, and building packages.",
        "simulation": "Check out Module 2: Gazebo Simulation - learn URDF modeling and sensor simulation.",
        "isaac": "Explore Module 3: NVIDIA Isaac Platform - for GPU-accelerated robotics and AI training.",
        "locomotion": "Module 4: Humanoid Robotics focuses on bipedal walking and balance control.",
        "perception": "Module 3 (Isaac ROS) and Module 5 (Physical AI) cover perception and sensor fusion.",
        "ai": "Module 5: Physical AI Integration - end-to-end learning and sim-to-real transfer.",
    }
    
    interest_lower = interest_area.lower()
    for key, recommendation in recommendations.items():
        if key in interest_lower:
            return recommendation
    
    return "Based on your interest, I'd recommend starting with Module 1 for fundamentals, then exploring the modules that match your goals."


@function_tool
def get_prerequisites() -> str:
    """
    Returns the prerequisites needed to follow the book content.
    Use when users ask about what they need to know before starting.
    """
    return """
    **Prerequisites for this book:**
    
    🐍 **Programming**: Python proficiency (intermediate level)
    🐧 **Linux**: Basic command line familiarity (Ubuntu recommended)
    🔧 **Optional but helpful**:
       - C++ basics (for some ROS 2 components)
       - Linear algebra fundamentals
       - Basic machine learning concepts
    
    💻 **System Requirements**:
       - Ubuntu 22.04 LTS (or WSL2 on Windows)
       - 16GB RAM minimum (32GB recommended)
       - NVIDIA GPU for Isaac modules (RTX 2070 or better)
    """


class GreetingAgent:
    def __init__(self):
        self.config = AGENTS_CONFIG["greeting"]
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
            tools=[get_book_overview, get_chapter_recommendations, get_prerequisites]
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
        Process greeting or general book queries.
        
        Args:
            query: User's message
            rag_context: Retrieved chapter excerpts (may not be used for greetings)
            sources: List of source metadata
        
        Returns:
            dict with answer, sources, agent info
        """
        try:
            # For greetings, we may not need RAG context
            # But we include it in case the user is asking about the book
            user_message = f"""User message: {query}

{"Book context (if relevant):" + chr(10) + rag_context if rag_context else "No specific book context retrieved for this query."}

Respond warmly and helpfully. If the user is greeting you or asking about the book, use your tools to provide helpful information."""

            result = await Runner.run(
                self.agent,
                user_message,
                run_config=self.run_config
            )
            
            answer = result.final_output
            
            logger.info(f"Greeting Agent generated response: {len(answer)} chars")
            
            return {
                "answer": answer,
                "sources": sources,
                "agent": self.config["name"],
                "agent_type": "greeting"
            }
            
        except Exception as e:
            logger.error(f"Greeting Agent error: {e}")
            raise
