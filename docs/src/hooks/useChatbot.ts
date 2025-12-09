import { useState, useCallback, useEffect } from 'react';
import apiService, { ChatResponse } from '../api/apiService';

interface Message {
    message: string;
    sender: 'user' | 'assistant';
    agentName?: string;
    agentIcon?: React.ReactNode;
}

interface UseChatbotReturn {
    messages: Message[];
    isLoading: boolean;
    error: string | null;
    selectedText: string | null;
    sendMessage: (message: string, isSelectedText?: boolean) => Promise<void>;
    clearSelection: () => void;
}

export const useChatbot = (): UseChatbotReturn => {
    const [messages, setMessages] = useState<Message[]>([]);
    const [isLoading, setIsLoading] = useState(false);
    const [error, setError] = useState<string | null>(null);
    const [selectedText, setSelectedText] = useState<string | null>(null);

    // Detect selected text
    useEffect(() => {
        const handleSelectionChange = () => {
            const selection = window.getSelection();
            if (selection && selection.toString().trim()) {
                setSelectedText(selection.toString().trim());
            } else {
                setSelectedText(null);
            }
        };

        document.addEventListener('selectionchange', handleSelectionChange);
        return () => {
            document.removeEventListener('selectionchange', handleSelectionChange);
        };
    }, []);

    const clearSelection = useCallback(() => {
        window.getSelection()?.empty();
        setSelectedText(null);
    }, []);

    const sendMessage = useCallback(async (message: string, isSelectedText: boolean = false) => {
        setMessages((prev) => [...prev, { message, sender: 'user' }]);
        setIsLoading(true);
        setError(null);

        try {
            const contextToSend = isSelectedText ? selectedText : null;
            const response: ChatResponse = await apiService.sendMessage(message, contextToSend);

            setMessages((prev) => [
                ...prev,
                {
                    message: response.answer,
                    sender: 'assistant',
                    agentName: response.agent_used,
                },
            ]);

            if (isSelectedText) {
                clearSelection();
            }
        } catch (err: any) {
            console.error("Error sending message:", err);
            const errorMessage = err.response?.data?.detail || err.message || "Failed to get response.";
            setError(errorMessage);
            setMessages((prev) => [
                ...prev,
                { message: `Error: ${errorMessage}`, sender: 'assistant' },
            ]);
        } finally {
            setIsLoading(false);
        }
    }, [selectedText, clearSelection]);

    return {
        messages,
        isLoading,
        error,
        selectedText,
        sendMessage,
        clearSelection
    };
};
