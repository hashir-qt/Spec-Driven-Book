from sqlalchemy import Column, String, JSON, DateTime, UUID, ForeignKey, Float, Integer
from sqlalchemy.dialects.postgresql import UUID as pgUUID
import uuid
from datetime import datetime
from sqlalchemy.orm import relationship
from .user import Base  # Import Base from a common file, or user.py if it's the first model

class ChatHistory(Base):
    __tablename__ = "chat_history"

    id = Column(pgUUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    user_id = Column(pgUUID(as_uuid=True), ForeignKey("users.id"), nullable=True, index=True) # Nullable for anonymous users
    
    message = Column(String, nullable=False)
    response = Column(String, nullable=False)
    
    # Agent metadata
    agent_used = Column(String(50))
    agent_confidence = Column(Float)
    
    # RAG metadata
    sources = Column(JSON)
    context_relevance = Column(Float)
    
    # Performance metrics
    response_time_ms = Column(Integer)
    rag_time_ms = Column(Integer)
    agent_time_ms = Column(Integer)
    
    created_at = Column(DateTime, default=datetime.utcnow, index=True)

    user = relationship("User")
