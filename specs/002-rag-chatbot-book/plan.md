# Implementation Plan: RAG Chatbot

## 1. Technical Context

-   **Feature:** RAG Chatbot for "Physical AI & Humanoid Robotics" Docusaurus Book
-   **Language:** Python 3.11
-   **Framework:** FastAPI
-   **Database:** Qdrant
-   **Project Type:** Web Service

## 2. Constitution Check

-   [X] Technical accuracy: The plan aligns with the technical standards of the project.
-   [X] Clarity: The plan is written for a technical audience.
-   [X] Reproducibility: The plan outlines a clear path to a reproducible implementation.
-   [X] Alignment: The plan is aligned with the official documentation of the chosen technologies.
-   [X] Conceptual consistency: The plan maintains a unified terminology.
-   [X] Zero hallucinations: The plan includes measures to prevent hallucination.

## 3. Implementation Phases

### Phase 1 — Architecture & Environment Setup

-   Define system architecture diagram.
-   Finalize environment variables and secrets handling.
-   Set up FastAPI project skeleton.
-   Configure Qdrant Cloud connection.
-   Configure Cohere embedding client.
-   Configure OpenAI-compatible Gemini client for Agents SDK.

### Phase 2 — Ingestion Pipeline (Spec-Driven)

-   Fetch and parse sitemap.xml.
-   Filter valid book content URLs.
-   Fetch HTML pages.
-   Extract readable text and metadata.
-   Normalize content and remove boilerplate.
-   Split content into sections and chunks.
-   Generate deterministic chunk IDs.
-   Batch embed chunks using Cohere.
-   Upsert vectors and payloads into Qdrant.
-   Implement `/ingest` endpoint with idempotency.

### Phase 3 — Retrieval Layer

-   Implement query-to-embedding conversion.
-   Implement Qdrant similarity search (top-k, cosine).
-   Support optional URL-level filtering.
-   Normalize retrieved chunks for agent consumption.
-   Log retrieval scores and sources.

### Phase 4 — Selected-Text-Only Flow

-   Accept user-selected text from frontend.
-   Chunk selected text using same chunk rules.
-   Embed selected text (ephemeral).
-   Bypass Qdrant retrieval when selection is present.
-   Pass only selected-text context to agent.
-   Enforce strict refusal if answer is unsupported.

### Phase 5 — Agent Construction (OpenAI Agents SDK)

-   Define retrieval tool(s).
-   Register tools with Agent.
-   Implement grounding-strict system instructions.
-   Enforce source citation in final answers.
-   Disable hallucination beyond provided chunks.
-   Integrate Gemini model via OpenAI-compatible client.
-   Validate tool-calling behavior.

### Phase 6 — API Layer (FastAPI)

-   `/ingest` endpoint (protected in production).
-   `/query` endpoint (global + selected-text modes).
-   Structured JSON request/response models.
-   Error handling and timeout management.
-   CORS configuration for Docusaurus domain.

### Phase 7 — Frontend Integration (Docusaurus)

-   Create lightweight React chat widget.
-   Chat input and response rendering.
-   “Answer only from selected text” toggle.
-   Source display with clickable links.
-   Highlight source snippets.
-   Accessibility and keyboard support.

### Phase 8 — Validation & Hardening

-   Run all unit and integration tests.
-   Validate acceptance criteria.
-   Test idempotent ingestion.
-   Measure query latency.
-   Add structured logging.
-   Add rate-limiting and auth for `/ingest`.

### Phase 9 — Deployment & Documentation

-   Local dev setup (Docker optional).
-   Production deployment plan (Cloud Run / Serverless).
-   Environment configuration documentation.
-   README with setup, run, and test instructions.
-   Post-deployment verification checklist.

## 4. Deliverables

-   Architecture diagram
-   Phase-wise implementation artifacts
-   Fully tested FastAPI RAG backend
-   Embedded Docusaurus chatbot
-   Deployment-ready documentation