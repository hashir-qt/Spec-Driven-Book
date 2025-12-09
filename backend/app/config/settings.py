from pydantic_settings import BaseSettings
from functools import lru_cache
   
class Settings(BaseSettings):
    # API Keys
    GEMINI_API_KEY: str  # Primary LLM for agents
    QDRANT_URL: str
    QDRANT_API_KEY: str
    NEON_DATABASE_URL: str
       
    # Gemini OpenAI-compatible endpoint
    GEMINI_BASE_URL: str = "https://generativelanguage.googleapis.com/v1beta/openai/"
    GEMINI_MODEL: str = "gemini-2.5-flash"
       
    # JWT Settings
    JWT_SECRET_KEY: str
    JWT_ALGORITHM: str = "HS256"
    ACCESS_TOKEN_EXPIRE_MINUTES: int = 15
    REFRESH_TOKEN_EXPIRE_DAYS: int = 7
       
    # RAG Settings
    QDRANT_COLLECTION_NAME: str = "book_chapters_en"
    QDRANT_VECTOR_SIZE: int = 3072  # gemini-embedding-001
    RAG_TOP_K: int = 5
    RAG_SCORE_THRESHOLD: float = 0.35
    RAG_MAX_CONTEXT_TOKENS: int = 6000
       
    # Agent Settings
    AGENT_TEMPERATURE: float = 0.3
    AGENT_MAX_TOKENS: int = 800
       
    # CORS Settings
    # CORS_ORIGINS: list = ["http://localhost:3000"]
       
    class Config:
        env_file = ".env"
        case_sensitive = True
   
@lru_cache()
def get_settings():
    return Settings()