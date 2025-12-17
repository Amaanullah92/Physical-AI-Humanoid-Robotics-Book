# RAG Chatbot for "Physical AI & Humanoid Robotics" Docusaurus Book

This project is a RAG chatbot for the "Physical AI & Humanoid Robotics" Docusaurus book.

## Setup

1.  Install Docker and Docker Compose.
2.  Create a `.env` file from the `.env.example` file and fill in the required values.
3.  Install Python dependencies: `pip install -r src/requirements.txt`
4.  Run `docker-compose up -d` to start the application (optional, if you want to run Qdrant locally).

## Running the application

The FastAPI application will be available at `http://localhost:8000`.

**CORS Configuration:**
The FastAPI backend is configured to allow requests from `http://localhost:3000` (the default Docusaurus development server URL). If your Docusaurus site is running on a different URL, you will need to update the `origins` list in `src/main.py`.

## Ingestion

To start the ingestion process, ensure your `INGEST_AUTH_TOKEN` environment variable is set in your shell (e.g., `export INGEST_AUTH_TOKEN=YOUR_TOKEN`) and then send a POST request to the `/ingest` endpoint with a valid bearer token.

```
curl -X POST http://localhost:8000/ingest -H "Authorization: Bearer YOUR_TOKEN"
```

## Querying

To query the chatbot, send a POST request to the `/query` endpoint.

```
curl -X POST http://localhost:8000/query -H "Content-Type: application/json" -d '{"query": "What is the difference between a stepper motor and a servo motor?"}'
```
