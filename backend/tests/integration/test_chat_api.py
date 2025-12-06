import pytest
from unittest.mock import AsyncMock, patch
from fastapi.testclient import TestClient
from app.main import app
from app.db.models.chat_history import ChatHistory
from app.db.session import get_db, SessionLocal
from app.routes.chat import get_rag_service_dependency # Import the dependency function
import sys
sys.path.append('../../') # Add project root to sys.path

@pytest.fixture(name="client", scope="module")
def client_fixture():
    # Use TestClient directly with the app, which will use the configured get_db
    with TestClient(app) as client:
        yield client

@pytest.fixture(name="db_session", scope="function")
def db_session_fixture():
    # Provide a separate session for each test for isolation
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()

@pytest.fixture
def mock_agent_run():
    with patch('agents.Runner.run', new_callable=AsyncMock) as mock_run:
        mock_run.return_value.final_output = "Agent-generated answer based on context."
        yield mock_run

@pytest.fixture
def mock_get_rag_service_dependency():
    with patch('app.routes.chat.get_rag_service_dependency', new_callable=AsyncMock) as mock_dependency:
        mock_rag_service_instance = AsyncMock()
        mock_rag_service_instance.retrieve_context.return_value = {
            "context": "This is a retrieved context about ROS 2 topics. It helps to explain concepts clearly.",
            "sources": [
                {"chapter_id": "m1/c1", "chapter_title": "ROS Intro", "url": "/docs/m1/c1", "relevance_score": 0.85}
            ],
            "relevance": 0.85
        }
        mock_dependency.return_value = mock_rag_service_instance
        app.dependency_overrides[get_rag_service_dependency] = mock_dependency
        yield mock_rag_service_instance
        app.dependency_overrides.clear()


@pytest.mark.asyncio
async def test_chat_endpoint_concept_query(client, mock_get_rag_service_dependency, mock_agent_run, db_session):
    # Ensure the Gemini client is setup before running tests
    from app.config.llm_client import setup_gemini_client
    setup_gemini_client()

    response = client.post(
        "/api/v1/chat",
        json={
            "message": "Explain ROS 2 topics",
            "context": None,
            "conversation_history": []
        }
    )

    assert response.status_code == 200
    data = response.json()

    assert "answer" in data
    assert "sources" in data
    assert data["agent_type"] == "concept_explainer"
    assert data["agent_used"] == "Concept Explainer Agent"
    assert data["agent_confidence"] > 0.6 # Ensure orchestrator routed with confidence
    assert data["context_relevance"] >= 0.7
    assert "response_time_ms" in data["metadata"]
    assert "rag_time_ms" in data["metadata"]
    assert "agent_time_ms" in data["metadata"]

    mock_get_rag_service_dependency.retrieve_context.assert_awaited_once()
    mock_agent_run.assert_awaited_once()

    # Verify chat history saved
    chat_entry = db_session.query(ChatHistory).first()
    assert chat_entry is not None
    assert chat_entry.message == "Explain ROS 2 topics"
    assert chat_entry.agent_type == "concept_explainer"
    assert chat_entry.response == data["answer"]

@pytest.mark.asyncio
async def test_chat_endpoint_no_rag_context(client, mock_get_rag_service_dependency, db_session):
    # Ensure the Gemini client is setup before running tests
    from app.config.llm_client import setup_gemini_client
    setup_gemini_client()

    # Configure RAG service to return no context
    mock_get_rag_service_dependency.retrieve_context.return_value = {
        "context": "",
        "sources": [],
        "relevance": 0.0
    }

    # Mock Runner.run to ensure no actual LLM call is made (this is already done by mock_agent_run fixture if needed, but explicitly for this test)
    with patch('agents.Runner.run', new_callable=AsyncMock) as mock_agent_run_local:
        mock_agent_run_local.return_value.final_output = "Mocked: I couldn't find relevant information in the book for your query. Please try rephrasing."

        response = client.post(
            "/api/v1/chat",
            json={
                "message": "Tell me about quantum computing in robotics (off-topic)",
                "context": None,
                "conversation_history": []
            }
        )

        assert response.status_code == 200
        data = response.json()

        assert "couldn't find relevant information" in data["answer"]
        assert data["agent_used"] == "N/A"
        assert data["agent_type"] == "N/A"
        mock_agent_run_local.assert_not_called() # Ensure agent was NOT called

        # Verify chat history saved
        chat_entry = db_session.query(ChatHistory).order_by(ChatHistory.created_at.desc()).first()
        assert chat_entry is not None
        assert "off-topic" in chat_entry.message
        assert "couldn't find relevant information" in chat_entry.response