# Feature Specification: RAG Chatbot for "Physical AI & Humanoid Robotics" Docusaurus Book

## 1. Introduction

This document outlines the requirements for a Retrieval-Augmented Generation (RAG) chatbot integrated into the "Physical AI & Humanoid Robotics" Docusaurus book. The chatbot will assist users by answering questions based on the book's content.

## 2. User Roles & Personas

-   **End-User/Reader:** A student, researcher, or enthusiast reading the book. They want to ask questions and get answers from the book's content.
-   **Maintainer/Developer:** A developer responsible for the book's website. They need to ingest new or updated content into the chatbot's knowledge base.

## 3. User Scenarios & Testing

-   **Scenario 1: General Question**
    -   As a Reader, I want to ask a question about a topic in the book (e.g., "What is the difference between a stepper motor and a servo motor?").
    -   The chatbot should provide a concise answer based on the book's content and cite the sources.

-   **Scenario 2: Selected Text Question**
    -   As a Reader, I want to highlight a specific paragraph or section of the book and ask a question about it.
    -   The chatbot should answer the question based *only* on the selected text.

-   **Scenario 3: Content Ingestion**
    -   As a Maintainer, I want to update the chatbot's knowledge base with the latest version of the book.
    -   I should be able to trigger an ingestion process that is idempotent and updates the knowledge base without creating duplicate entries.

## 4. Functional Requirements

### 4.1. Ingestion Pipeline

-   The system must provide a mechanism to ingest content from the Docusaurus book's sitemap.
-   The ingestion process must be idempotent. Running the ingestion process multiple times should not create duplicate entries in the knowledge base.
-   The system must fetch the content of each page, clean it, and split it into manageable chunks.
-   Each chunk must be converted into a vector embedding using the configured Cohere model.
-   The vector embeddings and their corresponding text and metadata must be stored in a Qdrant collection named `physical_ai_book`.

### 4.2. RAG Agent & API

-   The system must expose a FastAPI endpoint (`/query`) that accepts user questions.
-   The API must support two modes of operation:
    1.  **Global Retrieval:** Answering questions based on the entire knowledge base.
    2.  **Selected-Text-Only:** Answering questions based only on a user-provided text selection.
-   The RAG agent must be implemented using the OpenAI Agents SDK and be compatible with Gemini.
-   The agent's responses must be grounded in the retrieved content and cite the sources.

### 4.3. Chat Widget

-   A React-based chat widget must be provided for integration into the Docusaurus book.
-   The widget must provide a user interface for asking questions and viewing answers.
-   The widget must include a toggle to enable "selected-text-only" mode.
-   The widget must display the sources for each answer, with clickable links to the relevant sections of the book.

## 5. Non-Functional Requirements

-   **Performance:** The median query time for a retrieval and LLM call should be less than 1.5 seconds in a development environment.
-   **Security:** API keys and other secrets must be loaded from environment variables and not be hardcoded in the source code. The `/ingest` endpoint must be protected by an authentication token in a production environment.
-   **Logging:** The system must provide detailed logs for ingestion, embedding, Qdrant operations, and agent responses.

## 6. Success Criteria

-   The ingestion pipeline successfully creates a Qdrant collection named `physical_ai_book` with a number of vectors corresponding to the number of chunks from the sitemap.
-   A query to the `/query` endpoint with a question about a book topic returns an answer with at least one source citation.
-   The "selected-text-only" mode returns an answer strictly based on the provided text selection.
-   The ingestion process is idempotent.
-   The median query latency is below 1.5 seconds.
-   Logs are generated for all major operations.

## 7. Assumptions

-   The Docusaurus book provides a sitemap.xml file at the specified URL.
-   The user has valid API keys for Cohere, Qdrant, and Gemini.

## 8. Out of Scope

-   Fine-tuning of LLMs.
-   Indexing of external, non-book sources.
-   Serving private user data.