# Architecture: RAG Chatbot

```mermaid
graph TD
    A[Sitemap.xml] --> B{Ingestion Pipeline};
    B --> C[Fetch Pages];
    C --> D[Clean & Chunk];
    D --> E[Embed with Cohere];
    E --> F[Store in Qdrant];

    subgraph "FastAPI Backend"
        G[/ingest] --> B;
        H[/query] --> I{RAG Agent};
    end

    J[Docusaurus] --> K[Chat Widget];
    K --> H;

    subgraph "RAG Agent"
        direction LR
        L[User Query] --> M{Query Router};
        M -- "Global" --> N[Retrieve from Qdrant];
        M -- "Selected Text" --> O[Embed Selected Text];
        N --> P[Grounding Prompt];
        O --> P;
        P --> Q[Gemini LLM];
        Q --> R[Format Response];
    end

    I --> L;
    R --> H;

```
