// Dummy types, replace with actual ones from backend
export interface SourceReference {
  chapter_id: string;
  chapter_title: string;
  url: string;
  relevance_score: number;
}

export interface ChatResponse {
  answer: string;
  sources: SourceReference[];
  agent_used: string; // Human readable name
  agent_type: string; // Internal type
  agent_confidence: number;
  context_relevance: number;
  metadata: {
    response_time_ms: number;
    rag_time_ms: number;
    agent_time_ms: number;
  };
}

// Assuming apiService is set up to make requests to your backend
const sendMessage = async (message: string, context: string | null): Promise<ChatResponse> => {
  // Replace with your actual API endpoint and logic
  // Use environment variable or default to localhost:8000
  // Use environment variable or default to localhost:8000
  const baseUrl = 'http://127.0.0.1:8000';
  const response = await fetch(`${baseUrl}/api/v1/chat`, {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
    },
    body: JSON.stringify({
      message: message,
      context: context,
      conversation_history: [],
    }),
  });

  if (!response.ok) {
    // Handle HTTP errors
    const errorData = await response.json();
    throw new Error(errorData.detail || `HTTP error! status: ${response.status}`);
  }

  return response.json();
};

const apiService = {
  sendMessage,
};

export default apiService;
