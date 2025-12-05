from fastapi import APIRouter, Depends, HTTPException, Request
from typing import List, Dict, Optional

from app.schemas.chatbot import ChatRequest, ChatResponse, Source
from app.services.chatbot.rag_engine import RAGEngine
from app.main import rag_engine # Import the initialized rag_engine from main

router = APIRouter()

@router.post("/chat", response_model=ChatResponse)
async def chat_endpoint(request: ChatRequest):
    """
    Processes a chat message using the RAG engine to provide answers
    based on the book content.
    """
    try:
        response_data = rag_engine.process_query(
            message=request.message,
            selected_text=request.selected_text,
            conversation_history=request.conversation_history
        )
        
        # Convert sources to Pydantic models
        sources_pydantic = [
            Source(
                file_path=s.get("file_path", "N/A"),
                chapter_name=s.get("chapter_name", "N/A"),
                score=s.get("score", 0.0)
            ) for s in response_data.get("sources", [])
        ]

        return ChatResponse(
            answer=response_data.get("answer", "No answer found."),
            sources=sources_pydantic,
            confidence=response_data.get("confidence", 0.0)
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Internal server error: {e}")
