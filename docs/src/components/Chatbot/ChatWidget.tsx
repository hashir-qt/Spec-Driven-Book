import React, { useState, useEffect } from 'react';
import ChatMessage from './ChatMessage';
import ChatInput from './ChatInput';
import { useChatbot } from '../../hooks/useChatbot';
import './ChatWidget.css';

import { FaRobot, FaChalkboardTeacher, FaCode, FaTools, FaCommentDots, FaTimes } from 'react-icons/fa';

const ChatWidget: React.FC = () => {
  const [isOpen, setIsOpen] = useState(false);
  const { messages, isLoading, error, selectedText, sendMessage } = useChatbot();

  const agentIcons: { [key: string]: React.ReactNode } = {
    'Concept Explainer Agent': <FaChalkboardTeacher />,
    'Code Helper Agent': <FaCode />,
    'Troubleshooting Agent': <FaTools />,
    'Default Agent': <FaRobot />
  };

  // Keyboard shortcut for selected text
  useEffect(() => {
    const handleKeyDown = (event: KeyboardEvent) => {
      if (event.ctrlKey && event.key === 'q') {
        event.preventDefault();
        if (selectedText) {
          if (!isOpen) setIsOpen(true);
          sendMessage(selectedText, true);
        }
      }
    };

    window.addEventListener('keydown', handleKeyDown);
    return () => window.removeEventListener('keydown', handleKeyDown);
  }, [selectedText, sendMessage, isOpen]);

  const toggleChat = () => setIsOpen(!isOpen);

  return (
    <>
      <button
        className={`chat-toggle-button ${isOpen ? 'hidden' : ''}`}
        onClick={toggleChat}
        aria-label="Open Chatbot"
      >
        <FaCommentDots />
      </button>

      {isOpen && (
        <div className="chat-widget-container">
          <div className="chat-widget-header">
            <div className="header-title">
              <FaRobot className="header-icon" />
              <h2>AI Book Assistant</h2>
            </div>
            <button onClick={toggleChat} className="close-button" aria-label="Close Chat">
              <FaTimes />
            </button>
          </div>

          <div className="chat-messages">
            {messages.length === 0 && (
              <div className="welcome-message">
                <p>Hello! I can help you with questions about the book content. Try asking about "ROS 2 nodes" or select text to ask about it.</p>
              </div>
            )}

            {messages.map((msg, index) => (
              <ChatMessage
                key={index}
                message={msg.message}
                sender={msg.sender}
                agentName={msg.agentName}
                agentIcon={msg.agentName ? (agentIcons[msg.agentName] || agentIcons['Default Agent']) : undefined}
              />
            ))}

            {isLoading && (
              <ChatMessage message="Thinking..." sender="assistant" />
            )}

            {error && <ChatMessage message={`Error: ${error}`} sender="assistant" />}
          </div>

          <div className="chat-footer">
            {selectedText && (
              <div className="selected-text-preview">
                <small>Selected: "{selectedText.substring(0, 30)}..."</small>
                <code className="shortcut-hint">Ctrl+Q to send</code>
              </div>
            )}
            <ChatInput onSendMessage={(msg) => sendMessage(msg)} />
          </div>
        </div>
      )}
    </>
  );
};

export default ChatWidget;
