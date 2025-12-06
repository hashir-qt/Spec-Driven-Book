from openai import AsyncOpenAI
from agents import set_default_openai_client
from .settings import get_settings
   
def setup_gemini_client():
    """Configure OpenAI Agents SDK to use Gemini as backend LLM."""
    settings = get_settings()
       
    # Create Gemini-backed AsyncOpenAI client
    gemini_client = AsyncOpenAI(
        api_key=settings.GEMINI_API_KEY,
        base_url=settings.GEMINI_BASE_URL
    )
       
    # Set as default client for all agents
    # use_for_tracing=False to avoid OpenAI tracing errors
    set_default_openai_client(gemini_client, use_for_tracing=False)
       
    return gemini_client