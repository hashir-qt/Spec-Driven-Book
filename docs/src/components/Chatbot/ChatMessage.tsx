import React from 'react';

interface ChatMessageProps {
  message: string;
  sender: 'user' | 'assistant';
  agentName?: string; // Optional: to display the agent name
  agentIcon?: React.ReactNode;
  timestamp?: Date;
}

const ChatMessage: React.FC<ChatMessageProps> = ({ message, sender, agentName, agentIcon, timestamp }) => {
  const messageClass = sender === 'user' ? 'user-message' : 'assistant-message';

  const formatTime = (date?: Date) => {
    if (!date) return '';
    return date.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' });
  };

  return (
    <div className={`message ${messageClass}`}>
      <div className="message-content">
        {sender === 'assistant' && agentIcon && <div className="agent-icon">{agentIcon}</div>}
        {sender === 'assistant' && agentName && <div className="agent-name">{agentName}</div>}
        <p>{message}</p>
        {timestamp && <span className="message-timestamp">{formatTime(timestamp)}</span>}
      </div>
    </div>
  );
};

export default ChatMessage;