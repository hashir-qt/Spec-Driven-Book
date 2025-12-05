import os
from dotenv import load_dotenv
import openai
from qdrant_client import QdrantClient
import psycopg2

# Load environment variables from .env file
load_dotenv()

def test_openai_connection():
    """Tests the connection to the OpenAI API."""
    print("Testing OpenAI connection...")
    try:
        api_key = os.getenv("GEMINI_API_KEY")
        if not api_key:
            print("❌ GEMINI_API_KEY not found in .env file.")
            return

        base_url = os.getenv("GEMINI_BASE_URL")
        if base_url:
            print(f"Using custom Gemini base_url: {base_url}")
            client = openai.OpenAI(api_key=api_key, base_url=base_url)
        else:
            print("Using default OpenAI base_url.")
            client = openai.OpenAI(api_key=api_key)
        response = client.embeddings.create(
            input="This is a test.",
            model="gemini-embedding-001"
        )
        embedding = response.data[0].embedding
        if len(embedding) == 3072:
            print("✅ OpenAI connection successful. Received 3072-dimensional embedding (Gemini).")
        else:
            print(f"❌ OpenAI connection successful, but embedding dimension is {len(embedding)}, expected 3072.")

    except Exception as e:
        print(f"❌ OpenAI connection failed: {e}")

def test_qdrant_connection():
    """Tests the connection to the Qdrant database."""
    print("\nTesting Qdrant connection...")
    try:
        qdrant_url = os.getenv("QDRANT_URL")
        qdrant_api_key = os.getenv("QDRANT_API_KEY")
        if not qdrant_url:
            print("❌ QDRANT_URL not found in .env file.")
            return

        client = QdrantClient(url=qdrant_url, api_key=qdrant_api_key)
        client.get_collections()
        print("✅ Qdrant connection successful. Able to list collections.")

    except Exception as e:
        print(f"❌ Qdrant connection failed: {e}")

def test_neon_connection():
    """Tests the connection to the Neon Postgres database."""
    print("\nTesting Neon Postgres connection...")
    try:
        neon_db_url = os.getenv("NEON_DATABASE_URL")
        if not neon_db_url:
            print("❌ NEON_DATABASE_URL not found in .env file.")
            return

        conn = psycopg2.connect(neon_db_url)
        cur = conn.cursor()
        cur.execute("SELECT 1;")
        cur.fetchone()
        print("✅ Neon Postgres connection successful. Able to execute a simple query.")
        cur.close()
        conn.close()

    except Exception as e:
        print(f"❌ Neon Postgres connection failed: {e}")

if __name__ == "__main__":
    test_openai_connection()
    test_qdrant_connection()
    test_neon_connection()
