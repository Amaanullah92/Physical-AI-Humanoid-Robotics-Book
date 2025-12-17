# Tasks: RAG Chatbot

This document breaks down the implementation of the RAG chatbot into specific tasks, organized by user story.

## Phase 1: Setup

- [x] T001 Create a new FastAPI project in a `src` directory. `src/main.py`
- [x] T002 [P] Add FastAPI, uvicorn, python-dotenv, requests, beautifulsoup4, loguru, and tiktoken to `src/requirements.txt`.
- [x] T003 [P] Create a basic "hello world" endpoint in `src/main.py`.
- [x] T004 [P] Create a `.env.example` file with all the required environment variables.

## Phase 2: Foundational

- [x] T005 Create a module for Qdrant client configuration in `src/qdrant_client.py`.
- [x] T006 Create a module for Cohere client configuration in `src/cohere_client.py`.
- [x] T007 Create a module for the Gemini client configuration in `src/gemini_client.py`.

## Phase 3: User Story 1 - General Question

- [x] T008 [US1] Implement a data class for the `/query` request body in `src/models.py`.
- [x] T009 [US1] Implement the query-to-embedding conversion function in `src/retrieval.py`.
- [x] T010 [US1] Implement the Qdrant similarity search function in `src/retrieval.py`.
- [x] T011 [US1] Implement the retrieval tool for the RAG agent in `src/agent.py`.
- [x] T012 [US1] Implement the RAG agent with the grounding-strict system prompt in `src/agent.py`.
- [x] T013 [US1] Implement the `/query` endpoint in `src/main.py`.
- [x] T014 [US1] Create a React chat widget with an input field, a submit button, and a display area for the response in `docs/src/components/ChatWidget.tsx`.
- [x] T015 [US1] Integrate the chat widget into the Docusaurus site.

## Phase 4: User Story 2 - Selected Text Question

- [x] T016 [US2] Modify the `/query` endpoint to accept selected text in `src/main.py`.
- [x] T017 [US2] Implement the logic to chunk and embed the selected text in `src/retrieval.py`.
- [x] T018 [US2] Implement the logic to bypass Qdrant retrieval when a selection is present in `src/agent.py`.
- [x] T019 [US2] Add a "selected-text-only" toggle to the chat widget in `docs/src/components/ChatWidget.tsx`.

## Phase 5: User Story 3 - Content Ingestion

- [x] T020 [US3] Implement a function to fetch and parse the sitemap.xml in `src/ingestion.py`.
- [x] T021 [US3] Implement a function to fetch and clean HTML content in `src/ingestion.py`.
- [x] T022 [US3] Implement a function to chunk the content in `src/ingestion.py`.
- [x] T023 [US3] Implement a function to generate deterministic chunk IDs in `src/ingestion.py`.
- [x] T024 [US3] Implement a function to batch embed chunks with Cohere in `src/ingestion.py`.
- [x] T025 [US3] Implement a function to upsert vectors into Qdrant in `src/ingestion.py`.
- [x] T026 [US3] Implement the `/ingest` endpoint in `src/main.py`.

## Phase 6: Polish & Cross-Cutting Concerns

- [x] T027 [P] Add structured logging to all components.
- [x] T028 [P] Add error handling and timeout management to the API.
- [x] T029 [P] Add authentication to the `/ingest` endpoint.
- [x] T030 [P] Create a `docker-compose.yml` for local development.
- [x] T031 [P] Create a `Dockerfile` for the application.
- [x] T032 [P] Create a `.dockerignore` file.
- [x] T033 [P] Write a `README.md` with setup, run, and test instructions.
- [x] T034 [P] Add a new task for performance testing to measure query latency.

## Dependencies

-   US2 depends on US1.
-   US3 is independent.

## Parallel Execution

-   US1 and US3 can be implemented in parallel.
-   Within each story, tasks marked with `[P]` can be implemented in parallel.
