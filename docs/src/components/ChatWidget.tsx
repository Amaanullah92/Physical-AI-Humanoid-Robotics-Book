import React, { useState, useEffect, useRef } from 'react';

interface Message {
    id: number;
    text: string;
    sender: 'user' | 'bot';
}

const ChatWidget = () => {
    const [query, setQuery] = useState('');
    const [messages, setMessages] = useState<Message[]>([]);
    const [selectedTextOnly, setSelectedTextOnly] = useState(false);
    const [isChatOpen, setIsChatOpen] = useState(false);
    const messagesEndRef = useRef<HTMLDivElement>(null);

    const scrollToBottom = () => {
        messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
    };

    useEffect(() => {
        scrollToBottom();
    }, [messages]);

    const handleSubmit = async (e: React.FormEvent) => {
        e.preventDefault();
        if (!query.trim()) return;

        const userMessage: Message = { id: messages.length + 1, text: query, sender: 'user' };
        setMessages((prevMessages) => [...prevMessages, userMessage]);
        setQuery('');

        let selected_text: string | null = null;
        if (selectedTextOnly) {
            const selection = window.getSelection();
            if (selection && selection.toString().trim()) {
                selected_text = selection.toString();
            } else {
                const botMessage: Message = { id: messages.length + 2, text: "Please select some text to get an answer based on your selection.", sender: 'bot' };
                setMessages((prevMessages) => [...prevMessages, botMessage]);
                return;
            }
        }

        try {
            const response = await fetch('http://localhost:8000/query', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify({ query: userMessage.text, selected_text }),
            });
            const data = await response.json();
            const botMessage: Message = { id: messages.length + 2, text: data.answer, sender: 'bot' };
            setMessages((prevMessages) => [...prevMessages, botMessage]);
        } catch (error) {
            console.error("Failed to fetch:", error);
            const errorMessage: Message = { id: messages.length + 2, text: "Failed to connect to the chatbot. Please ensure the backend is running.", sender: 'bot' };
            setMessages((prevMessages) => [...prevMessages, errorMessage]);
        }
    };

    return (
        <>
            {/* Floating Chat Button */}
            <div className="chatbot-button-container">
                <button className="chatbot-button" onClick={() => setIsChatOpen(!isChatOpen)}>
                    {isChatOpen ? '−' : '💬'}
                </button>
            </div>

            {/* Mini Chat Window */}
            <div className={`chatbot-window ${isChatOpen ? 'open' : ''}`}>
                <div className="chatbot-header">
                    <h3>Chatbot</h3>
                    <button onClick={() => setIsChatOpen(false)}>✕</button>
                </div>
                <div className="chatbot-messages">
                    {messages.map((msg) => (
                        <div key={msg.id} className={`message-bubble ${msg.sender}-message`}>
                            {msg.text}
                        </div>
                    ))}
                    <div ref={messagesEndRef} />
                </div>
                <form className="chatbot-input-form" onSubmit={handleSubmit}>
                    <input
                        type="text"
                        value={query}
                        onChange={(e) => setQuery(e.target.value)}
                        placeholder="Ask a question..."
                    />
                    <button type="submit">Send</button>
                </form>
                <div className="chatbot-options">
                    <input
                        type="checkbox"
                        id="selected-text-only"
                        checked={selectedTextOnly}
                        onChange={(e) => setSelectedTextOnly(e.target.checked)}
                    />
                    <label htmlFor="selected-text-only">Answer only from selected text</label>
                </div>
            </div>
        </>
    );
};

export default ChatWidget;
