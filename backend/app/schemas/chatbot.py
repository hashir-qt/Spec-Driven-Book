from pydantic import BaseModel, Field
from typing import List, Dict, Optional

class ChatRequest(BaseModel):
    message: str = Field(..., min_length=1, max_length=2000, description="The user's chat message.")
    selected_text: Optional[str] = Field(None, max_length=10000, description="Optional text selected by the user for context.")
    conversation_history: Optional[List[Dict]] = Field(None, description="Previous messages in the conversation.")

class Source(BaseModel):
    file_path: str = Field(..., description="The file path of the source document.")
    chapter_name: str = Field(..., description="The name of the chapter from which the content was sourced.")
    score: float = Field(..., description="The relevance score of the source.")

class ChatResponse(BaseModel):
    answer: str = Field(..., description="The assistant's response.")
    sources: List[Source] = Field(..., description="A list of source documents used to generate the answer.")
    confidence: float = Field(..., description="A confidence score for the answer.")
