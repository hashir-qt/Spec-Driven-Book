from app.config.agents_config import AGENTS_CONFIG
from app.config.settings import get_settings
from qdrant_client import AsyncQdrantClient, models
import logging

# Dependencies should be passed in, not created inside
from app.chatbot.embedder import Embedder
from app.chatbot.vector_search import VectorSearchService
from app.chatbot.context_builder import ContextBuilder

logger = logging.getLogger(__name__)

class RAGService:
    # 1. FIX: Inject dependencies to manage lifecycle externally
    def __init__(
        self, 
        qdrant_client: AsyncQdrantClient,
        embedder: Embedder,
        vector_search: VectorSearchService,
        context_builder: ContextBuilder
    ):
        self.settings = get_settings()
        self.qdrant_client = qdrant_client
        self.embedder = embedder
        self.vector_search_service = vector_search
        self.context_builder = context_builder

    async def retrieve_context(self, query: str, agent_type: str) -> dict:
        try:
            # 1. Generate embedding
            query_embedding = await self.embedder.get_embedding(query)

            # 2. Prepare Filters
            agent_config = AGENTS_CONFIG.get(agent_type, {})
            rag_filter_keywords = agent_config.get("rag_filter_keywords", [])
            
            # 2. FIX: Construct Qdrant Filter (Push logic down to DB)
            # This requires that 'content' or keywords are indexed in Qdrant payload
            qdrant_filter = None
            if rag_filter_keywords:
                # Example: Match ANY of the keywords in a payload field named "keywords"
                # OR use Qdrant's full-text search if enabled on 'content'
                qdrant_filter = models.Filter(
                    should=[
                        models.FieldCondition(
                            key="content", 
                            match=models.MatchText(text=kw) # Requires text index on 'content'
                        ) for kw in rag_filter_keywords
                    ]
                )

            # 3. Search Qdrant
            # First attempt: With agent-specific filters (strict)
            search_results = await self.vector_search_service.search(
                query_vector=query_embedding,
                limit=self.settings.RAG_TOP_K,
                score_threshold=self.settings.RAG_SCORE_THRESHOLD,
                query_filter=qdrant_filter
            )


            # Fallback: If strict filtering returns too few results, try broad semantic search
            if len(search_results) < 2 and qdrant_filter is not None:
                logger.info(f"RAG: Strict filter yielded {len(search_results)} results. Falling back to specific broad search.")
                broad_results = await self.vector_search_service.search(
                    query_vector=query_embedding,
                    limit=self.settings.RAG_TOP_K,
                    score_threshold=self.settings.RAG_SCORE_THRESHOLD,
                    query_filter=None # No filter
                )
                
                logger.info(f"RAG: Broad search yielded {len(broad_results)} results.")
                if broad_results:
                    logger.info(f"RAG: Top broad score: {broad_results[0].score}")
                
                # Merge results (avoiding duplicates based on ID or content)
                seen_ids = {res.id for res in search_results}
                for res in broad_results:
                    if res.id not in seen_ids:
                        search_results.append(res)
                        seen_ids.add(res.id)
                
                # Re-sort by score (descending) and trim to top K
                search_results.sort(key=lambda x: x.score, reverse=True)
                search_results = search_results[:self.settings.RAG_TOP_K]
            
            # 4. Build context
            context_data = self.context_builder.build(
                search_results,
                max_tokens=self.settings.RAG_MAX_CONTEXT_TOKENS
            )

            relevance_scores = [res.score for res in search_results]
            average_relevance = sum(relevance_scores) / len(relevance_scores) if relevance_scores else 0.0

            return {
                "context": context_data["context_string"],
                "sources": context_data["sources_list"],
                "relevance": average_relevance
            }

        except Exception as e:
            logger.error(f"Error in RAGService.retrieve_context: {e}")
            raise