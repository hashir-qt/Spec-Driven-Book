import os
import openai
from dotenv import load_dotenv

load_dotenv()

client = openai.OpenAI(
    api_key=os.getenv("GEMINI_API_KEY"),
    base_url=os.getenv("GEMINI_BASE_URL"),
)

try:
    response = client.embeddings.create(
        input="Hello world",
        model="text-embedding-004" 
    )
    # Using text-embedding-004 as it is the current Gemini embedding model via OpenAI compat
    # seed_vector_db.py uses 'gemini-embedding-001' which might map to older or same.
    # I'll test both.
    
    print(f"text-embedding-004 dimension: {len(response.data[0].embedding)}")
except Exception as e:
    print(f"text-embedding-004 failed: {e}")

try:
    response = client.embeddings.create(
        input="Hello world",
        model="gemini-embedding-001"
    )
    print(f"gemini-embedding-001 dimension: {len(response.data[0].embedding)}")
except Exception as e:
    print(f"gemini-embedding-001 failed: {e}")
