from typing import List, Dict, Optional
import tiktoken
import os
import asyncio
from qdrant_client import models

class ContextBuilder:
    def __init__(self):
        self.encoding = tiktoken.get_encoding("cl100k_base")

    def _count_tokens(self, text: str) -> int:
        return len(self.encoding.encode(text))

    def build(
        self,
        search_results: List[Dict],
        max_tokens: int,
        selected_text: Optional[str] = None,
        conversation_history: Optional[List[Dict]] = None,
    ) -> Dict:
        """
        Builds the context string for the LLM, prioritizing selected text,
        then conversation history, then search results. Truncates if needed.

        Args:
            search_results: A list of dictionaries, where each dict contains
                            'content', 'file_path', and 'chunk_index'.
            max_tokens: The maximum number of tokens allowed in the context.
            selected_text: Optional text selected by the user in the UI.
            conversation_history: Optional list of previous messages in the chat.

        Returns:
            A dictionary containing the formatted context string and a list of source references.
        """
        context_parts = []
        source_references = []
        current_tokens = 0

        # Helper to add a part if within token limit
        def add_part(text_to_add: str, is_source: bool = False, source_info: Optional[Dict] = None):
            nonlocal current_tokens
            tokens_for_part = self._count_tokens(text_to_add)
            if current_tokens + tokens_for_part <= max_tokens:
                context_parts.append(text_to_add)
                current_tokens += tokens_for_part
                if is_source and source_info:
                    source_references.append(source_info)
                return True
            return False

        # 1. Prioritize selected text
        if selected_text:
            formatted_selected_text = f"Selected Text:\n{selected_text}\n---\n"
            if not add_part(formatted_selected_text):
                return {"context_string": formatted_selected_text, "sources_list": []} # If selected text alone exceeds limit, just return it

        # 2. Add conversation history
        if conversation_history:
            history_buffer = []
            history_tokens = 0
            # Add recent history first (reversed for chronological display later)
            for msg in reversed(conversation_history):
                msg_text = f"{msg['role'].capitalize()}: {msg['content']}\n"
                tokens_for_msg = self._count_tokens(msg_text)
                if current_tokens + history_tokens + tokens_for_msg <= max_tokens:
                    history_buffer.insert(0, msg_text) # Insert at beginning to keep chronological order
                    history_tokens += tokens_for_msg
                else:
                    break
            
            if history_buffer:
                formatted_history = "Conversation History:\n" + "".join(history_buffer) + "---\n"
                add_part(formatted_history) # Add as a single part to respect max_tokens

        # 3. Add search results
        if search_results:
            for i, result in enumerate(search_results):
                file_name = os.path.basename(result.payload.get('file_path', 'unknown_file'))
                # Robustly extract relative path for chapter_id
                file_path = result.payload.get('file_path', 'unknown_file')
                # Normalize separators
                file_path_norm = file_path.replace('\\', '/')
                
                if 'docs/docs/' in file_path_norm:
                     # Extract part after docs/docs/
                     chapter_id = file_path_norm.split('docs/docs/')[-1]
                else:
                     # Fallback to basename if structure matches unexpected pattern
                     chapter_id = os.path.basename(file_path_norm)
                
                chapter_id = chapter_id.replace('.md', '')
                
                chapter_title = os.path.splitext(file_name)[0].replace('-', ' ').title() # Basic title formatting
                
                source_info = {
                    "chapter_id": chapter_id,
                    "chapter_title": chapter_title,
                    "url": f"/docs/{chapter_id}",
                    "relevance_score": result.score
                }

                source_text = f"Source: {chapter_title} (ID: {chapter_id})\n{result.payload.get('content', '')}\n---\n"
                add_part(source_text, is_source=True, source_info=source_info)

        return {"context_string": "\n".join(context_parts).strip(), "sources_list": source_references}

# Example usage (for testing purposes)
if __name__ == "__main__":
    # This part should ideally be in a separate test or example file
    from app.config.settings import get_settings
    from qdrant_client import AsyncQdrantClient
    from dotenv import load_dotenv

    load_dotenv(dotenv_path=os.path.join(os.path.dirname(__file__), '..', '..', '.env'))
    settings = get_settings()

    async def main_example():
        builder = ContextBuilder()
        # Dummy search results (ensure they are compatible with ScoredPoint structure)
        # Note: In real usage, search_results would come from qdrant_client.query_points
        dummy_search_results = [
            models.ScoredPoint(id=1, score=0.8, payload={"file_path": "../docs/docs/module-1/intro.md", "content": "Content from doc 1."}),
            models.ScoredPoint(id=2, score=0.7, payload={"file_path": "../docs/docs/module-2/chapter-x.md", "content": "Content from doc 2."}),
        ]

        dummy_conversation_history = [
            {"role": "user", "content": "Hello"},
            {"role": "assistant", "content": "Hi there!"}
        ]

        context_output = builder.build(
            search_results=dummy_search_results,
            max_tokens=1000,  # Specify max_tokens here for the example
            selected_text="This is a selected piece of text.",
            conversation_history=dummy_conversation_history
        )
        print("---")
        print("Example Context")
        print("---")
        print(context_output["context_string"])
        print("---")
        print("Example Sources")
        print("---")
        for source in context_output["sources_list"]:
            print(f"- {source['chapter_title']} (Score: {source['relevance_score']})")

        # Example of direct Qdrant client usage if needed
        # qdrant_client_example = AsyncQdrantClient(
        #     url=settings.QDRANT_URL,
        #     api_key=settings.QDRANT_API_KEY,
        # )
        # await qdrant_client_example.close()

    asyncio.run(main_example())