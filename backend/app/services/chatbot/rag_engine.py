import os
import openai
from qdrant_client import QdrantClient
from typing import List, Dict, Optional

from .vector_search import VectorSearchService
from .context_builder import ContextBuilder

class RAGEngine:
    def __init__(
        self,
        openai_client: openai.OpenAI,
        qdrant_client: QdrantClient,
        vector_collection_name: str,
        embedding_model: str = "gemini-embedding-001",
        chat_model: str = "flash-2.5", # Assuming flash-2.5 is the chat completion model
        max_context_tokens: int = 6000
    ):
        self.openai_client = openai_client
        self.embedding_model = embedding_model
        self.chat_model = chat_model
        self.vector_search_service = VectorSearchService(qdrant_client, vector_collection_name)
        self.context_builder = ContextBuilder(max_tokens=max_context_tokens)

    def _get_query_embedding(self, text: str) -> List[float]:
        """Generates an embedding for the query text."""
        response = self.openai_client.embeddings.create(
            input=text,
            model=self.embedding_model
        )
        return response.data[0].embedding

    def process_query(
        self,
        message: str,
        selected_text: Optional[str] = None,
        conversation_history: Optional[List[Dict]] = None
    ) -> Dict:
        """
        Processes a user query using RAG.

        Args:
            message: The user's current message.
            selected_text: Optional text selected by the user in the UI.
            conversation_history: Optional list of previous messages in the chat.

        Returns:
            A dictionary containing the answer and source information.
        """
        query_embedding = self._get_query_embedding(message)
        search_results = self.vector_search_service.search(query_embedding)

        # Extract content and file_path for context building and source tracking
        context_search_results = [
            {"content": r.payload.get("content", ""), "file_path": r.payload.get("file_path", "")}
            for r in search_results
        ]
        
        # Build context for the LLM
        context = self.context_builder.build(
            search_results=context_search_results,
            selected_text=selected_text,
            conversation_history=conversation_history
        )

        # Prepare messages for chat completion
        messages = [
            {"role": "system", "content": "You are a helpful assistant for a Physical AI and Robotics book. Answer questions truthfully and only use information from the provided context. If the answer is not in the context, state that you cannot answer from the provided information."}
        ]
        if context:
            messages.append({"role": "system", "content": f"Here is relevant context from the book:\n{context}"})
        
        if conversation_history:
            for item in conversation_history:
                messages.append({"role": item['role'], "content": item['content']})

        messages.append({"role": "user", "content": message})

        # Call OpenAI Agents SDK for chat completion
        try:
            chat_completion = self.openai_client.chat.completions.create(
                model=self.chat_model,
                messages=messages
            )
            answer = chat_completion.choices[0].message.content
        except Exception as e:
            print(f"Error calling chat completion API: {e}")
            answer = "I am currently unable to provide a response. Please try again later."

        # Prepare source information
        sources = []
        for result in search_results:
            file_name = os.path.basename(result.payload.get("file_path", "Unknown"))
            sources.append({
                "file_path": result.payload.get("file_path", "Unknown"),
                "chapter_name": file_name.replace(".md", "").replace("-", " ").title(),
                "score": result.score
            })
        
        # Simple confidence score (can be improved)
        confidence = sum([r.score for r in search_results]) / len(search_results) if search_results else 0.0

        return {"answer": answer, "sources": sources, "confidence": confidence}

# Example usage (for testing purposes)
if __name__ == "__main__":
    load_dotenv()
    
    # Initialize OpenAI client (configured for Gemini)
    openai_client = openai.OpenAI(
        api_key=os.getenv("GEMINI_API_KEY"),
        base_url=os.getenv("GEMINI_BASE_URL"),
    )
    # Initialize Qdrant client
    qdrant_client = QdrantClient(
        url=os.getenv("QDRANT_URL"),
        api_key=os.getenv("QDRANT_API_KEY"),
    )

    rag_engine = RAGEngine(openai_client, qdrant_client, "book_chapters_en")

    # Example query
    print("--- Example Query 1 ---")
    response = rag_engine.process_query("What are ROS 2 nodes and how do they communicate?")
    print("Answer:", response["answer"])
    print("Sources:", response["sources"])
    print("Confidence:", response["confidence"])

    print("\n--- Example Query 2 (with selected text) ---")
    response2 = rag_engine.process_query(
        "Explain this simply:",
        selected_text="ROS 2 nodes are independent processes that communicate via topics, services, and actions."
    )
    print("Answer:", response2["answer"])
    print("Sources:", response2["sources"])
    print("Confidence:", response2["confidence"])

    print("\n--- Example Query 3 (with conversation history) ---")
    response3 = rag_engine.process_query(
        "Tell me more about actions.",
        conversation_history=[
            {"role": "user", "content": "What are ROS 2 topics?"},
            {"role": "assistant", "content": "ROS 2 topics are a publish-subscribe communication mechanism."}
        ]
    )
    print("Answer:", response3["answer"])
    print("Sources:", response3["sources"])
    print("Confidence:", response3["confidence"])
