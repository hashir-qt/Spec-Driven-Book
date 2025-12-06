from app.config.agents_config import AGENTS_CONFIG, ROUTING_CONFIG
import logging

logger = logging.getLogger(__name__)

class AgentOrchestrator:
    def __init__(self):
        self.agents_config = AGENTS_CONFIG
        self.routing_config = ROUTING_CONFIG
    
    def classify_query(self, query: str) -> tuple[str, float]:
        """
        Classify query to determine appropriate agent.
        
        Returns:
            (agent_name, confidence_score)
        """
        query_lower = query.lower()
        query_words = set(query_lower.split())
        
        scores = {}
        for agent_name, config in self.agents_config.items():
            keywords = config["task_keywords"]
            # Calculate confidence based on presence of keywords
            # Assign a higher confidence if any keyword is present
            # More sophisticated scoring can be added later (e.g., TF-IDF, embedding similarity)
            confidence = 0.0
            if any(kw in query_lower for kw in keywords):
                confidence = config.get("confidence_threshold", 0.7) # Use agent's defined threshold as base
            scores[agent_name] = confidence
            scores[agent_name] = confidence
        
        # Get agent with highest score
        # Handle case where scores might be empty, though AGENTS_CONFIG should prevent this
        if not scores:
            logger.warning("Agent configuration is empty, falling back to default.")
            return self.routing_config["default_agent"], 0.0

        best_agent = max(scores, key=scores.get)
        best_score = scores[best_agent]
        
        # Use default if confidence too low
        min_confidence = self.routing_config["min_confidence"]
        if best_score < min_confidence:
            logger.info(f"Low confidence ({best_score:.2f}) for '{query}', using default agent '{self.routing_config['default_agent']}'")
            return self.routing_config["default_agent"], best_score
        
        logger.info(f"Routed query '{query}' to '{best_agent}' with confidence {best_score:.2f}")
        return best_agent, best_score
    
    def route_query(self, query: str) -> str:
        """Simple wrapper that returns agent name only."""
        agent_name, _ = self.classify_query(query)
        return agent_name