import asyncio
import os
import sys
from dotenv import load_dotenv

# Add parent directory to path so we can import app modules
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from app.chatbot.rag_service import RAGService
from app.chatbot.embedder import Embedder
from app.chatbot.vector_search import VectorSearchService
from app.chatbot.context_builder import ContextBuilder
from app.config.settings import get_settings
from qdrant_client import AsyncQdrantClient

# Load environment variables
load_dotenv()

async def verify_rag():
    print("--- Starting RAG Pipeline Verification ---")
    settings = get_settings()
    
    # 1. Initialize Clients
    print(f"Initializing Qdrant Client at {settings.QDRANT_URL}...")
    qdrant_client = AsyncQdrantClient(
        url=settings.QDRANT_URL,
        api_key=settings.QDRANT_API_KEY,
    )
    
    embedder = Embedder()
    vector_search = VectorSearchService(qdrant_client, settings.QDRANT_COLLECTION_NAME)
    context_builder = ContextBuilder()
    
    rag_service = RAGService(
        qdrant_client=qdrant_client,
        embedder=embedder,
        vector_search=vector_search,
        context_builder=context_builder
    )
    
    # 2. Test Query
    test_query = "What is a neural network?" # Common topic likely in the book
    agent_type = "concept_explainer"
    
    print(f"\nProcessing Query: '{test_query}' with agent '{agent_type}'...")
    
    try:
        # 3. Retrieve Context
        result = await rag_service.retrieve_context(test_query, agent_type)
        
        print("\n--- Verification Results ---")
        print(f"Relevance Score: {result['relevance']:.4f}")
        
        if result['sources']:
            print(f"\nFound {len(result['sources'])} sources:")
            for source in result['sources']:
                print(f" - {source['chapter_title']} (Score: {source['relevance_score']:.4f})")
                print(f"   URL: {source['url']}")
        else:
            print("\n[WARNING] No sources found. Is the database seeded?")

        if result['context']:
             print(f"\nContext Preview (first 200 chars):\n{result['context'][:200]}...")
        else:
             print("\n[WARNING] Context is empty.")
             
    except Exception as e:
        print(f"\n[ERROR] RAG Verification failed: {e}")
        import traceback
        traceback.print_exc()
    finally:
        await qdrant_client.close()
        print("\n--- Verification Complete ---")

if __name__ == "__main__":
    asyncio.run(verify_rag())
