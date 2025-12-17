# Research: RAG Chatbot for "Physical AI & Humanoid Robotics" Docusaurus Book

## 1. Chunking Strategy

-   **Decision:** Use a hybrid approach of section-based and sentence-aware chunking.
-   **Rationale:** Section-based chunking ensures that content is grouped by its semantic context (headings). Sentence-aware chunking within sections prevents sentences from being split, which improves the quality of the embeddings.
-   **Alternatives considered:**
    -   **Purely section-based:** This can lead to very large chunks that are not ideal for embedding.
    -   **Sliding window:** This can be effective but may not respect the semantic boundaries of the content as well as a section-based approach.

## 2. Embedding Model Choice and Vector Dimensionality Handling

-   **Decision:** Use the Cohere `embed-english-v3.0` model, which has a dimensionality of 1024.
-   **Rationale:** This model provides a good balance of performance and cost. The dimensionality of 1024 is a standard size that is well-supported by Qdrant.
-   **Alternatives considered:**
    -   **Other Cohere models:** Other models were considered, but `embed-english-v3.0` was chosen for its performance on a wide range of tasks.
    -   **OpenAI models:** OpenAI models were considered, but the user's prompt specified Cohere.

## 3. Qdrant Schema Design and Deterministic ID Strategy

-   **Decision:** Use a Qdrant collection named `physical_ai_book` with a payload schema that includes `text`, `url`, `title`, `section_heading`, `chunk_index`, and `chunk_id`. The `chunk_id` will be a SHA256 hash of the URL, section heading, and chunk index.
-   **Rationale:** This schema provides all the necessary metadata for the RAG agent to function correctly. The deterministic ID strategy ensures that the ingestion process is idempotent.
-   **Alternatives considered:**
    -   **UUIDs for IDs:** This would not be idempotent.
    -   **Different payload schema:** The chosen schema is based on best practices for RAG applications.

## 4. Retrieval Strategy

-   **Decision:** Use a top-k retrieval strategy with a default k of 5. The similarity metric will be cosine similarity.
-   **Rationale:** Top-k retrieval is a simple and effective strategy for RAG applications. Cosine similarity is a standard metric for comparing text embeddings.
-   **Alternatives considered:**
    -   **Hybrid search:** This could be considered in the future to improve performance, but top-k is sufficient for the initial implementation.
    -   **Other similarity metrics:** Other metrics such as dot product or Euclidean distance were considered, but cosine similarity is the most common for this type of application.

## 5. Selected-Text-Only Answering Implementation

-   **Decision:** When the user provides selected text, the backend will chunk and embed the text ephemerally. This context will be passed to the agent, which will be instructed to only use this context to answer the question.
-   **Rationale:** This approach ensures that the agent's response is strictly limited to the user's selection.
-   **Alternatives considered:**
    -   **Persisting selected text embeddings:** This would add unnecessary complexity and storage overhead.

## 6. Agent Prompt Constraints to Prevent Hallucination

-   **Decision:** The agent's system prompt will include a strict instruction to only use the provided context and to cite sources. If the context is insufficient, the agent will be instructed to respond with a message indicating that it cannot answer the question.
-   **Rationale:** This is a standard technique for preventing hallucination in RAG applications.
-   **Alternatives considered:**
    -   **Less strict prompts:** This would increase the risk of hallucination.

## 7. Deployment Approach

-   **Decision:** The FastAPI backend will be deployed as a serverless function (e.g., Vercel Serverless Functions or AWS Lambda). The `/ingest` endpoint will be protected by an authentication token in production.
-   **Rationale:** A serverless deployment is cost-effective and scalable. Protecting the `/ingest` endpoint is a necessary security measure.
-   **Alternatives considered:**
    -   **Container-based deployment:** This is also a valid option but may be more complex to manage for this use case.
    -   **No authentication:** This would be a security risk.
