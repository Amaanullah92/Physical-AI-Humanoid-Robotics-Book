# Research: RAG Chatbot Integration Stability and Context Accuracy

## 1. Loading Strategy for ChatWidget

-   **Decision:** Implement dynamic import for the React ChatWidget component in Docusaurus.
-   **Rationale:** Dynamic import (using `React.lazy` and `Suspense`) will ensure that the chat widget's JavaScript bundle is only loaded when needed, preventing it from negatively impacting initial page load performance and avoiding potential crashes on pages where it's not immediately required. This aligns with the "stability" goal.
-   **Alternatives considered:**
    -   **SSR (Server-Side Rendering):** Not directly applicable for client-side chat widget integration into a static Docusaurus site without significant architectural changes.
    -   **Lazy loading with a visible trigger:** While effective, dynamic import offers a more seamless integration from a performance perspective.

## 2. Context Window Size for Retrieved Chunks

-   **Decision:** Maintain the current chunk size of 500-800 tokens with ~100 tokens overlap for Qdrant retrieval. This is a balance between granularity and context.
-   **Rationale:** This range provides sufficient context for the Gemini model without overwhelming it or losing fine-grained information. It's also consistent with typical RAG best practices.
-   **Alternatives considered:**
    -   **Smaller chunks:** Risk losing context and requiring more retrieval calls for a single query.
    -   **Larger chunks:** Risk introducing irrelevant information and increasing embedding costs/latency.

## 3. Top-K for Qdrant Search

-   **Decision:** Use a `top_k` value of 5 for Qdrant search.
-   **Rationale:** Retrieving 5 most relevant chunks provides a good balance between ensuring enough context for the LLM and minimizing the amount of irrelevant information passed to it. This can be configurable if testing indicates a different optimal value.
-   **Alternatives considered:**
    -   **Smaller `top_k` (e.g., 1-3):** Might not provide sufficient context for complex queries.
    -   **Larger `top_k` (e.g., 10+):** Can introduce noise and increase LLM inference cost/latency.

## 4. Error Handling: Invalid Token, Empty Selection, Fetch Failures

-   **Decision:** Implement comprehensive error handling at both the frontend (React ChatWidget) and backend (FastAPI) for invalid tokens, empty selections, and network failures.
-   **Rationale:**
    -   **Invalid Token:** The `/ingest` endpoint already has authentication. The `/query` endpoint currently does not. Error handling will prevent unexpected behavior.
    -   **Empty Selection:** When "selected-text-only" mode is active and no text is selected, the chatbot should provide a specific, user-friendly message indicating that no text was selected.
    -   **Fetch Failures:** The frontend should display a clear error message to the user if the backend API is unreachable or returns an error. The backend should log detailed errors.
-   **Alternatives considered:**
    -   **Generic error messages:** Less helpful for users and debugging.
    -   **Crashing:** Unacceptable user experience.
