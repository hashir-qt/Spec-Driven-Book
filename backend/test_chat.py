import requests
import json
import time

def test_chat():
    url = "http://127.0.0.1:8000/api/v1/chat"
    
    # query = "What is a ROS 2 node?"
    # Let's use a query that should trigger the Concept Agent and RAG
    query = "Explain ROS 2 nodes and how they use topics"
    
    payload = {
        "message": query
    }
    
    print(f"Sending query: '{query}'...")
    
    try:
        start_time = time.time()
        response = requests.post(url, json=payload)
        end_time = time.time()
        
        if response.status_code == 200:
            data = response.json()
            print("\n✅ Response Received!")
            print(f"⏱️ Time taken: {end_time - start_time:.2f}s")
            print(f"🤖 Agent Used: {data['agent_used']} ({data['agent_confidence']:.2f})")
            print(f"📚 Context Relevance: {data['context_relevance']:.2f}")
            print(f"📄 Sources Found: {len(data['sources'])}")
            print("-" * 50)
            print("Answer:")
            print(data['answer'])
            print("-" * 50)
            
            if data['sources']:
                print("\nSources:")
                for source in data['sources']:
                    print(f"- {source['chapter_title']} (Score: {source['relevance_score']:.2f})")
        else:
            print(f"❌ Error: {response.status_code}")
            print(response.text)
            
    except Exception as e:
        print(f"❌ Exception: {e}")

if __name__ == "__main__":
    test_chat()
