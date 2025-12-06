import pytest
from app.agents.orchestrator import AgentOrchestrator

def test_concept_query_routing():
    orchestrator = AgentOrchestrator()
    agent, confidence = orchestrator.classify_query("What is ROS 2?")
    assert confidence >= 0.7

def test_code_query_routing():
    orchestrator = AgentOrchestrator()
    agent, confidence = orchestrator.classify_query("Show me code for a publisher")
    assert agent == "code_helper"
    assert confidence >= 0.75

def test_troubleshoot_query_routing():
    orchestrator = AgentOrchestrator()
    agent, confidence = orchestrator.classify_query("Error: node not found")
    assert agent == "troubleshoot"
    assert confidence >= 0.8

def test_low_confidence_fallback():
    orchestrator = AgentOrchestrator()
    # A query unlikely to match any specific agent with high confidence
    agent, confidence = orchestrator.classify_query("Tell me a story")
    assert agent == orchestrator.routing_config["default_agent"]
    assert confidence < orchestrator.routing_config["min_confidence"]

def test_empty_query_fallback():
    orchestrator = AgentOrchestrator()
    agent, confidence = orchestrator.classify_query("")
    assert agent == orchestrator.routing_config["default_agent"]
    assert confidence == 0.0
