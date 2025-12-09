from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from dotenv import load_dotenv

from app.config.settings import get_settings
from app.config.llm_client import setup_gemini_client
from app.routes import chat

# Load environment variables
load_dotenv()

# Get settings
settings = get_settings()

# Setup Gemini client for agents
setup_gemini_client()

# Initialize FastAPI app
app = FastAPI(
    title="Physical AI Humanoid Robotics Book API",
    docs_url="/docs"
    )

# Configure CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

# Root endpoint
@app.get("/")
def read_root():
    return {"message": "Welcome to the Physical AI Humanoid Robotics Book API!"}

# Include API router
app.include_router(chat.router, prefix="/api/v1")