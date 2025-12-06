from fastapi import APIRouter, HTTPException, Depends
from pydantic import BaseModel
import time
import logging
from sqlalchemy.orm import Session

from app.agents.orchestrator import AgentOrchestrator
from app.agents.concept_agent import ConceptExplainerAgent
from app.agents.code_agent import CodeHelperAgent
from app.agents.troubleshoot_agent import TroubleshootingAgent
from app.agents.greeting_agent import GreetingAgent
from app.chatbot.rag_service import RAGService
from app.config.settings import get_settings
from app.config.agents_config import AGENTS_CONFIG
from app.config.llm_client import setup_gemini_client
from app.db.session import get_db
from app.db.models.chat_history import ChatHistory
from qdrant_client import AsyncQdrantClient
from app.chatbot.embedder import Embedder
from app.chatbot.vector_search import VectorSearchService
from app.chatbot.context_builder import ContextBuilder

# Setup logging
logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)

router = APIRouter()
settings = get_settings()

# Initialize global agents and setup Gemini client
setup_gemini_client()
orchestrator = AgentOrchestrator()
greeting_agent = GreetingAgent()
concept_agent = ConceptExplainerAgent()
code_agent = CodeHelperAgent()
troubleshoot_agent = TroubleshootingAgent()

# Map agent types to agent instances
AGENT_INSTANCES = {
    "greeting": greeting_agent,
    "concept_explainer": concept_agent,
    "code_helper": code_agent,
    "troubleshoot": troubleshoot_agent,
}

# --- Dependency for RAGService ---
def get_rag_service_dependency() -> RAGService:
    qdrant_client_instance = AsyncQdrantClient(
        url=settings.QDRANT_URL,
        api_key=settings.QDRANT_API_KEY,
    )
    embedder_instance = Embedder()
    vector_search_instance = VectorSearchService(qdrant_client_instance, settings.QDRANT_COLLECTION_NAME)
    context_builder_instance = ContextBuilder()
    
    return RAGService(
        qdrant_client=qdrant_client_instance,
        embedder=embedder_instance,
        vector_search=vector_search_instance,
        context_builder=context_builder_instance
    )

class ChatRequest(BaseModel):
    message: str
    context: str | None = None  # For selected text
    conversation_history: list[dict] = [] # e.g. [{"role": "user", "content": "hi"}, {"role": "assistant", "content": "hello"}]

class SourceReference(BaseModel):
    chapter_id: str
    chapter_title: str
    url: str
    relevance_score: float

class ChatResponse(BaseModel):
    answer: str
    sources: list[SourceReference]
    agent_used: str  # "Concept Explainer Agent" etc. (human-readable name)
    agent_type: str  # "concept_explainer" (internal type)
    agent_confidence: float
    context_relevance: float
    metadata: dict

@router.post("/chat")
async def chat_endpoint(
    request: ChatRequest,
    db: Session = Depends(get_db),
    rag_service: RAGService = Depends(get_rag_service_dependency) # Inject RAGService
):
    start_time = time.time()
    
    try:
        # 1. Route query using orchestrator
        agent_type, agent_confidence = orchestrator.classify_query(request.message)
        agent_human_name = AGENTS_CONFIG[agent_type]["name"]
        logger.info(f"Routing to agent: {agent_type} (Confidence: {agent_confidence:.2f})")

        # 2. Retrieve RAG context
        rag_start_time = time.time()
        rag_result = await rag_service.retrieve_context(request.message, agent_type)
        rag_time_ms = int((time.time() - rag_start_time) * 1000)
        
        context_text = rag_result["context"]
        sources = [SourceReference(**s) for s in rag_result["sources"]]
        context_relevance = rag_result["relevance"]
        
        # 3. Call appropriate agent
        agent_instance = AGENT_INSTANCES.get(agent_type)
        if not agent_instance:
            raise HTTPException(status_code=500, detail=f"Agent '{agent_type}' not found.")
        
        agent_process_start_time = time.time()
        agent_response = await agent_instance.process_query(
            query=request.message,
            rag_context=context_text,
            sources=sources 
        )
        
        # Calculate final metrics
        agent_time_ms = int((time.time() - agent_process_start_time) * 1000)
        
        response = ChatResponse(
            answer=agent_response.get("answer", "I couldn't process your request."),
            sources=agent_response.get("sources", []),
            agent_used=agent_response.get("agent", "Unknown"),
            agent_type=agent_response.get("agent_type", agent_type),
            agent_confidence=rag_result.get("confidence", 0.0), # Use confidence from orchestrator/response
            context_relevance=context_relevance,
            metadata={
                "response_time_ms": int((time.time() - start_time) * 1000),
                "rag_time_ms": rag_time_ms,
                "agent_time_ms": agent_time_ms
            }
        )
        
        # Save chat history
        chat_entry = ChatHistory(
            message=request.message,
            response=response.answer,
            agent_used=response.agent_used,
            agent_confidence=1.0, # Placeholder or actual confidence
            context_relevance=response.context_relevance,
            response_time_ms=response.metadata.get("response_time_ms"),
            rag_time_ms=rag_time_ms,
            agent_time_ms=agent_time_ms
        )
        db.add(chat_entry)
        db.commit()
        db.refresh(chat_entry)
        
        return response


    except Exception as e:
        logger.exception(f"Chat endpoint error: {e}")
        raise HTTPException(status_code=500, detail="Internal server error")