from agents import Agent, Runner, function_tool, OpenAIChatCompletionsModel
from agents.run import RunConfig
from app.config.agents_config import AGENTS_CONFIG
from app.config.settings import get_settings
from openai import AsyncOpenAI
import logging

logger = logging.getLogger(__name__)


# Function tools for the concept explainer agent
@function_tool
def get_related_concepts(concept: str) -> str:
    """
    Returns related concepts that the user might want to explore next.
    Use this to suggest learning paths after explaining a concept.
    
    Args:
        concept: The concept that was just explained
    """
    concept_map = {
        "ros2": "Related concepts: Nodes, Topics, Services, Actions, Parameters, Launch files, tf2 transforms",
        "node": "Related concepts: Publishers, Subscribers, Executors, Callbacks, Lifecycle nodes",
        "topic": "Related concepts: Messages, QoS policies, Publishers, Subscribers, Bag recording",
        "gazebo": "Related concepts: URDF, SDF, Plugins, Sensors, World files, Physics engines",
        "urdf": "Related concepts: Links, Joints, Collision meshes, Visual meshes, Xacro macros",
        "isaac": "Related concepts: Isaac Sim, Isaac ROS, Omniverse, USD format, PhysX, CUDA",
        "simulation": "Related concepts: Physics engines, Sensor simulation, Domain randomization, Sim-to-real",
        "perception": "Related concepts: Computer vision, Sensor fusion, Object detection, SLAM, Point clouds",
        "locomotion": "Related concepts: Gait planning, Balance control, Inverse kinematics, Motion planning",
        "tf": "Related concepts: Coordinate frames, Transform trees, Static transforms, tf2 broadcaster/listener",
    }
    
    for key, related in concept_map.items():
        if key in concept.lower():
            return related
    
    return "Related concepts: Check the book's module overview for related topics in this area."


@function_tool
def generate_concept_diagram_description(concept: str) -> str:
    """
    Generates a text-based diagram description to help visualize a concept.
    Use this when a visual representation would help understanding.
    
    Args:
        concept: The concept to diagram
    """
    diagrams = {
        "ros2 communication": """
```
┌──────────────┐         Topic: /sensor_data         ┌──────────────┐
│   Publisher  │  ────────────────────────────────▶  │  Subscriber  │
│    Node A    │     (Messages flow one-way)         │    Node B    │
└──────────────┘                                     └──────────────┘

┌──────────────┐    Request      ┌──────────────┐
│    Client    │  ────────────▶  │   Service    │
│    Node A    │  ◀────────────  │    Server    │
└──────────────┘    Response     │    Node B    │
                                 └──────────────┘
```
""",
        "tf transform": """
```
                 world
                   │
                   ▼
              base_link
              /       \\
             ▼         ▼
        left_wheel   right_wheel
             
Each arrow represents a transform (position + rotation)
Child frames are defined relative to parent frames
```
""",
        "node lifecycle": """
```
┌───────────────┐    configure()    ┌──────────────┐
│  Unconfigured │  ──────────────▶  │  Configuring │
└───────────────┘                   └──────────────┘
                                           │
                                           ▼
┌───────────────┐    activate()     ┌──────────────┐
│    Active     │  ◀──────────────  │   Inactive   │
└───────────────┘                   └──────────────┘
```
"""
    }
    
    for key, diagram in diagrams.items():
        if key in concept.lower():
            return diagram
    
    return "No specific diagram available for this concept. Consider creating a flowchart or block diagram to visualize the relationships."


class ConceptExplainerAgent:
    def __init__(self):
        self.config = AGENTS_CONFIG["concept_explainer"]
        self.settings = get_settings()
        
        # Create OpenAI client for Gemini
        self.client = AsyncOpenAI(
            api_key=self.settings.GEMINI_API_KEY,
            base_url=self.settings.GEMINI_BASE_URL
        )
        
        # Explicitly use OpenAIChatCompletionsModel to work with Gemini via OpenAI compat
        self.model = OpenAIChatCompletionsModel(
            model=self.settings.GEMINI_MODEL,
            openai_client=self.client
        )
        
        # Create agent with function tools
        self.agent = Agent(
            name=self.config["name"],
            model=self.model,
            instructions=self.config["system_prompt"],
            tools=[get_related_concepts, generate_concept_diagram_description]
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
        Process query using Concept Explainer persona.
        
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

Please explain this concept clearly using analogies and the provided context. 
Use your tools when helpful:
- Use get_related_concepts to suggest what to learn next
- Use generate_concept_diagram_description to add visual aids
Always cite the source chapters."""

            result = await Runner.run(
                self.agent,
                user_message,
                run_config=self.run_config
            )
            
            answer = result.final_output
            
            logger.info(f"Concept Agent generated response: {len(answer)} chars")
            
            return {
                "answer": answer,
                "sources": sources,
                "agent": self.config["name"],
                "agent_type": "concept_explainer"
            }
            
        except Exception as e:
            logger.error(f"Concept Agent error: {e}")
            raise