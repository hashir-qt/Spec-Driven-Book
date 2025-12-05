import tiktoken
from typing import List, Dict, Optional
import os

class ContextBuilder:
    def __init__(self, max_tokens: int = 6000):
        self.max_tokens = max_tokens
        self.encoding = tiktoken.get_encoding("cl100k_base")

    def _count_tokens(self, text: str) -> int:
        return len(self.encoding.encode(text))

    def build(
        self,
        search_results: List[Dict],
        selected_text: Optional[str] = None,
        conversation_history: Optional[List[Dict]] = None,
    ) -> str:
        """
        Builds the context string for the LLM, prioritizing selected text,
        then conversation history, then search results. Truncates if needed.

        Args:
            search_results: A list of dictionaries, where each dict contains
                            'content', 'file_path', and 'chunk_index'.
            selected_text: Optional text selected by the user in the UI.
            conversation_history: Optional list of previous messages in the chat.

        Returns:
            A formatted context string for the LLM.
        """
        context_parts = []
        current_tokens = 0

        # 1. Prioritize selected text
        if selected_text:
            formatted_selected_text = f"Selected Text:\n{selected_text}\n---\n"
            tokens = self._count_tokens(formatted_selected_text)
            if current_tokens + tokens <= self.max_tokens:
                context_parts.append(formatted_selected_text)
                current_tokens += tokens
            else:
                return formatted_selected_text # If selected text alone exceeds limit, just return it

        # 2. Add conversation history
        if conversation_history:
            formatted_history = ""
            for msg in reversed(conversation_history): # Add recent history first
                # Assuming conversation_history is a list of {"role": "user/assistant", "content": "..."}
                msg_text = f"{msg['role'].capitalize()}: {msg['content']}\n"
                formatted_history = msg_text + formatted_history # Prepend to maintain chronological order
            
            formatted_history = f"Conversation History:\n{formatted_history}\n---\n"
            tokens = self._count_tokens(formatted_history)
            if current_tokens + tokens <= self.max_tokens:
                context_parts.append(formatted_history)
                current_tokens += tokens
            else:
                # Truncate history if it's too long
                truncated_history = ""
                for msg in reversed(conversation_history):
                    msg_text = f"{msg['role'].capitalize()}: {msg['content']}\n"
                    if self._count_tokens(truncated_history + msg_text) + current_tokens <= self.max_tokens:
                        truncated_history = msg_text + truncated_history
                    else:
                        break
                if truncated_history:
                    context_parts.append(f"Conversation History:\n{truncated_history}\n---\n")
                    current_tokens += self._count_tokens(f"Conversation History:\n{truncated_history}\n---\n")


        # 3. Add search results
        if search_results:
            search_context = []
            for i, result in enumerate(search_results):
                # Assuming payload has 'file_path' and 'content'
                file_name = os.path.basename(result.payload.get('file_path', 'unknown_file'))
                source_text = f"Source {i+1} ({file_name}):\n{result.payload.get('content', '')}\n---\n"
                tokens = self._count_tokens(source_text)

                if current_tokens + tokens <= self.max_tokens:
                    search_context.append(source_text)
                    current_tokens += tokens
                else:
                    break # Stop adding search results if max tokens reached

            if search_context:
                context_parts.append("Relevant Document Chunks:\n" + "".join(search_context))

        return "\n".join(context_parts).strip()

# Example usage (for testing purposes)
if __name__ == "__main__":
    builder = ContextBuilder(max_tokens=100) # Small max_tokens for demonstration

    # Dummy search results
    dummy_search_results = [
        {"payload": {"file_path": "/path/to/doc1.md", "content": "This is content from document 1. It talks about ROS 2 nodes and topics."}},
        {"payload": {"file_path": "/path/to/doc2.md", "content": "Document 2 content discusses Gazebo simulation and URDF models."}},
        {"payload": {"file_path": "/path/to/doc3.md", "content": "Isaac Sim is a powerful platform for robotics development."}},
    ]

    # Dummy conversation history
    dummy_conversation_history = [
        {"role": "user", "content": "What is ROS?"},
        {"role": "assistant", "content": "ROS is a set of software libraries for robot application development."},
        {"role": "user", "content": "Tell me more about nodes."},
    ]

    # Test cases
    print("--- Test Case 1: Only search results ---")
    context1 = builder.build(search_results=dummy_search_results)
    print(context1)
    print(f"Tokens: {builder._count_tokens(context1)}")

    print("\n--- Test Case 2: Selected text + search results ---")
    context2 = builder.build(search_results=dummy_search_results, selected_text="ROS 2 nodes are fundamental components.")
    print(context2)
    print(f"Tokens: {builder._count_tokens(context2)}")

    print("\n--- Test Case 3: Conversation history + search results ---")
    context3 = builder.build(search_results=dummy_search_results, conversation_history=dummy_conversation_history)
    print(context3)
    print(f"Tokens: {builder._count_tokens(context3)}")

    print("\n--- Test Case 4: All components (priority check) ---")
    context4 = builder.build(search_results=dummy_search_results, selected_text="Key concept.", conversation_history=dummy_conversation_history)
    print(context4)
    print(f"Tokens: {builder._count_tokens(context4)}")

    print("\n--- Test Case 5: Truncation with selected text ---")
    context5 = builder.build(search_results=dummy_search_results, selected_text="This is a very long selected text that should take up most of the tokens available in the context builder. We need to ensure that the selected text is prioritized and other parts are truncated or excluded if the token limit is reached. The token limit for this test case is intentionally set very low to demonstrate truncation effectively.")
    print(context5)
    print(f"Tokens: {builder._count_tokens(context5)}")
