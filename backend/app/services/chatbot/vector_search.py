from qdrant_client import QdrantClient, models
import os

class VectorSearchService:
    def __init__(self, qdrant_client: QdrantClient, collection_name: str):
        self.qdrant_client = qdrant_client
        self.collection_name = collection_name

    def search(self, query_vector, limit: int = 5, score_threshold: float = 0.6):
        """
        Queries the Qdrant collection for similar vectors.

        Args:
            query_vector: The embedding vector of the query.
            limit: The maximum number of results to return.
            score_threshold: The minimum similarity score for results to be returned.

        Returns:
            A list of search results with payload and score.
        """
        search_result = self.qdrant_client.search(
            collection_name=self.collection_name,
            query_vector=query_vector,
            limit=limit,
            query_filter=models.Filter(
                must=[
                    models.FieldCondition(
                        key="score",
                        range=models.Range(gte=score_threshold),
                    ),
                ]
            )
        )
        return search_result

# Example usage (for testing purposes)
if __name__ == "__main__":
    load_dotenv()
    qdrant_client = QdrantClient(
        url=os.getenv("QDRANT_URL"),
        api_key=os.getenv("QDRANT_API_KEY"),
    )
    vector_search_service = VectorSearchService(qdrant_client, "book_chapters_en")

    # This requires an actual query vector, for demonstration, let's use a dummy one
    # In a real scenario, this would come from get_embedding function
    dummy_query_vector = [0.1] * 3072  # Replace with a real embedding

    results = vector_search_service.search(dummy_query_vector)
    for result in results:
        print(f"Score: {result.score}, Content: {result.payload['content'][:100]}...")
