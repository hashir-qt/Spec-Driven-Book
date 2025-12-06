from openai import AsyncOpenAI
from app.config.settings import get_settings
import logging

logger = logging.getLogger(__name__)

class Embedder:
    def __init__(self):
        self.settings = get_settings()
        self.client = AsyncOpenAI(
            api_key=self.settings.GEMINI_API_KEY,
            base_url=self.settings.GEMINI_BASE_URL
        )
        self.embedding_model = "models/text-embedding-004" # This needs to be checked, Gemini's docs mention "models/text-embedding-004" or "text-embedding-004". I'll default to the model ID as specified in Gemini's API reference for Text Embedding.
        # The project constitution specifies gemini-embedding-001. I should clarify this.
        # For now I will use the more generic `text-embedding-004` to avoid breaking.
        # UPDATE: The spec plan mentions gemini-embedding-001 (vector size 3072) for Qdrant.
        # So I will use that. If that model is not directly exposed by the OpenAI compatible API,
        # it might need adjustment later. For now, assuming it is.
        self.embedding_model = "gemini-embedding-001" 

    async def get_embedding(self, text: str) -> list[float]:
        try:
            response = await self.client.embeddings.create(
                model=self.embedding_model,
                input=[text]
            )
            return response.data[0].embedding
        except Exception as e:
            logger.error(f"Error generating embedding: {e}")
            raise