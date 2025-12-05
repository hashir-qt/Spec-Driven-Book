import os
import glob
import frontmatter
import tiktoken
import openai
from qdrant_client import QdrantClient, models
from dotenv import load_dotenv
import hashlib

# Load environment variables from .env file
load_dotenv()

# Constants
DOCS_PATH = "../docs/docs"
CHUNK_SIZE = 500  # tokens
QDRANT_COLLECTION_NAME = "book_chapters_en"

# Initialize clients
client = openai.OpenAI(
    api_key=os.getenv("GEMINI_API_KEY"),
    base_url=os.getenv("GEMINI_BASE_URL"),
)
qdrant_client = QdrantClient(
    url=os.getenv("QDRANT_URL"), 
    api_key=os.getenv("QDRANT_API_KEY"),
)

def find_markdown_files(path):
    """Find all markdown files in a given path."""
    files = glob.glob(os.path.join(path, "**", "*.md"), recursive=True)
    print(f"DEBUG: Found {len(files)} markdown files.")
    return files

def parse_markdown(file_path):
    """Parse a markdown file and return its content, skipping frontmatter."""
    with open(file_path, 'r', encoding='utf-8') as f:
        post = frontmatter.load(f)
        return post.content

def chunk_text(text, chunk_size):
    """Chunk text into smaller pieces based on token count."""
    encoding = tiktoken.get_encoding("cl100k_base")
    tokens = encoding.encode(text)
    chunks = []
    for i in range(0, len(tokens), chunk_size):
        chunk_tokens = tokens[i:i + chunk_size]
        chunk_text = encoding.decode(chunk_tokens)
        chunks.append(chunk_text)
    print(f"DEBUG: Text chunked into {len(chunks)} chunks.")
    return chunks

def get_embedding(text):
    """Get embedding for a given text."""
    response = client.embeddings.create(
        input=text,
        model="gemini-embedding-001"
    )
    return response.data[0].embedding

def seed_database():
    """Seed the Qdrant database with book content."""
    # Recreate collection
    qdrant_client.recreate_collection(
        collection_name=QDRANT_COLLECTION_NAME,
        vectors_config=models.VectorParams(size=3072, distance=models.Distance.COSINE),
    )
    print(f"Collection '{QDRANT_COLLECTION_NAME}' created.")

    markdown_files = find_markdown_files(DOCS_PATH)
    print(f"DEBUG: Markdown files found: {markdown_files}")
    points = []

    for file_path in markdown_files:
        print(f"Processing {file_path}...")
        content = parse_markdown(file_path)
        print(f"DEBUG: Content length for {file_path}: {len(content)}")
        if not content.strip(): # Check if content is empty after stripping whitespace
            print(f"WARNING: Content for {file_path} is empty or only whitespace. Skipping.")
            continue
        
        chunks = chunk_text(content, CHUNK_SIZE)
        if not chunks:
            print(f"WARNING: No chunks generated for {file_path}. Skipping.")
            continue

        for i, chunk in enumerate(chunks):
            embedding = get_embedding(chunk)
            # Create a unique ID for the point
            point_id = hashlib.md5(f"{file_path}-{i}".encode()).hexdigest()
            
            points.append(
                models.PointStruct(
                    id=point_id,
                    vector=embedding,
                    payload={
                        "file_path": file_path,
                        "chunk_index": i,
                        "content": chunk,
                    },
                )
            )

    print(f"Upserting {len(points)} points to Qdrant...")
    qdrant_client.upsert(
        collection_name=QDRANT_COLLECTION_NAME,
        wait=True,
        points=points,
    )
    print("✅ Database seeding complete.")

if __name__ == "__main__":
    seed_database()