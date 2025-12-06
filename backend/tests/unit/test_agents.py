import pytest
from unittest.mock import AsyncMock, patch
from app.agents.concept_agent import ConceptExplainerAgent
from app.agents.code_agent import CodeHelperAgent
from app.agents.troubleshoot_agent import TroubleshootingAgent
from app.config.llm_client import setup_gemini_client

# Ensure Gemini client is set up for agents during test collection
setup_gemini_client()

@pytest.fixture
def mock_agent_run():
    with patch('agents.Runner.run', new_callable=AsyncMock) as mock_run:
        yield mock_run

@pytest.mark.asyncio
async def test_concept_explainer_agent(mock_agent_run):
    agent = ConceptExplainerAgent()
    
    mock_agent_run.return_value.final_output = "This is a concept explanation about ROS 2. Source: Module 1."
    
    response = await agent.process_query(
        query="What is ROS 2?",
        rag_context="ROS 2 is a set of software libraries for robot applications.",
        sources=[{"chapter_id": "module-1-ros2/01-introduction", "chapter_title": "Intro to ROS 2", "url": "#", "relevance_score": 0.9}]
    )
    
    assert "ROS 2" in response["answer"]
    assert response["agent_type"] == "concept_explainer"
    mock_agent_run.assert_awaited_once()

@pytest.mark.asyncio
async def test_code_helper_agent(mock_agent_run):
    agent = CodeHelperAgent()
    
    mock_agent_run.return_value.final_output = "```python\nprint(\'Hello ROS 2\')\n``` Source: Module 1 Code."
    
    response = await agent.process_query(
        query="Show me Python code for ROS 2 hello world.",
        rag_context="Python script to create a simple ROS 2 node.",
        sources=[{"chapter_id": "module-1-ros2/02-building-node", "chapter_title": "Building Your First Node", "url": "#", "relevance_score": 0.8}]
    )
    
    assert "```python" in response["answer"]
    assert response["agent_type"] == "code_helper"
    mock_agent_run.assert_awaited_once()

@pytest.mark.asyncio
async def test_troubleshooting_agent(mock_agent_run):
    agent = TroubleshootingAgent()
    
    mock_agent_run.return_value.final_output = "Here are steps to debug: 1. Check logs. Source: Troubleshooting Guide."
    
    response = await agent.process_query(
        query="My ROS 2 node is not starting, help!",
        rag_context="Common ROS 2 node startup issues.",
        sources=[{"chapter_id": "module-1-ros2/troubleshooting", "chapter_title": "ROS 2 Troubleshooting", "url": "#", "relevance_score": 0.75}]
    )
    
    assert "steps to debug" in response["answer"]
    assert response["agent_type"] == "troubleshoot"
    mock_agent_run.assert_awaited_once()

