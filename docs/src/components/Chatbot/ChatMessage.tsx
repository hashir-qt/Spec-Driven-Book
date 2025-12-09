import React from 'react';

interface ChatMessageProps {
  message: string;
  sender: 'user' | 'assistant';
  agentName?: string; // Optional: to display the agent name
  agentIcon?: React.ReactNode; // Optional: to display an agent icon
}

const ChatMessage: React.FC<ChatMessageProps> = ({ message, sender, agentName, agentIcon }) => {
  const messageClass = sender === 'user' ? 'user-message' : 'assistant-message';

  return (
    <div className={`message ${messageClass}`}>
      <div className="message-content">
        {sender === 'assistant' && agentIcon && <div className="agent-icon">{agentIcon}</div>}
        {sender === 'assistant' && agentName && <div className="agent-name">{agentName}</div>}
        <p>{message}</p>
      </div>
    </div>
  );
};

export default ChatMessage;