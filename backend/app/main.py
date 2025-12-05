from fastapi import FastAPI, APIRouter, Depends, HTTPException
from fastapi.middleware.cors import CORSMiddleware
import os
import openai
from qdrant_client import QdrantClient
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Initialize clients
openai_client = openai.OpenAI(
    api_key=os.getenv("GEMINI_API_KEY"),
    base_url=os.getenv("GEMINI_BASE_URL"),
)
qdrant_client = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY"),
)

# Initialize RAG Engine
from app.services.chatbot.rag_engine import RAGEngine
rag_engine = RAGEngine(
    openai_client=openai_client,
    qdrant_client=qdrant_client,
    vector_collection_name="book_chapters_en"
)

# Initialize FastAPI app
app = FastAPI(title="Physical AI Humanoid Robotics Book API")

# Configure CORS
# In a real production environment, you would restrict allow_origins to your frontend's domain.
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Allows all origins
    allow_credentials=True,
    allow_methods=["*"],  # Allows all methods
    allow_headers=["*"],  # Allows all headers
)

# Root endpoint
@app.get("/")
def read_root():
    return {"message": "Welcome to the Physical AI Humanoid Robotics Book API!"}

# Include API routers
from app.api.v1.endpoints import chatbot
app.include_router(chatbot.router, prefix="/api/v1")