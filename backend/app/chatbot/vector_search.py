from qdrant_client import AsyncQdrantClient, models
import os
import asyncio
from dotenv import load_dotenv

class VectorSearchService:
    def __init__(self, qdrant_client: AsyncQdrantClient, collection_name: str):
        self.qdrant_client = qdrant_client
        self.collection_name = collection_name

    async def search(self, query_vector: list[float], limit: int = 5, score_threshold: float = 0.6, query_filter: models.Filter | None = None):
        """
        Queries the Qdrant collection for similar vectors.

        Args:
            query_vector: The embedding vector of the query.
            limit: The maximum number of results to return.
            score_threshold: The minimum similarity score for results to be returned.
            query_filter: Optional: A Qdrant filter to apply to the search results.

        Returns:
            A list of search results with payload and score.
        """
        search_params = {
            "collection_name": self.collection_name,
            "query": query_vector,
            "limit": limit,
            "score_threshold": score_threshold, # Pass score_threshold directly
            "with_payload": True # Ensure payload is returned
        }

        if query_filter is not None:
            search_params["query_filter"] = query_filter
        
        search_result = await self.qdrant_client.query_points(
            **search_params
        )
        return search_result.points

# Example usage
if __name__ == "__main__":
    load_dotenv(dotenv_path=os.path.join(os.path.dirname(__file__), '..', '..', '.env'))
    
    async def main():
        settings = get_settings()
        # Use AsyncQdrantClient for asynchronous operations
        qdrant_client_instance = AsyncQdrantClient(
            url=os.getenv("QDRANT_URL"), # Use os.getenv as settings won't be initialized globally here
            api_key=os.getenv("QDRANT_API_KEY"),
        )
        
        vector_search_service = VectorSearchService(qdrant_client_instance, settings.QDRANT_COLLECTION_NAME)

        # Dummy vector for testing (size must match your collection, e.g., 3072 for gemini-embedding-001)
        dummy_query_vector = [0.1] * settings.QDRANT_VECTOR_SIZE 

        results = await vector_search_service.search(dummy_query_vector)
        
        for result in results:
            content_preview = result.payload.get('content', '')[:100]
            print(f"Score: {result.score}, Content: {content_preview}...")
            
        await qdrant_client_instance.close()

    asyncio.run(main())
