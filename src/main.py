from fastapi import FastAPI, Depends, HTTPException, Request
from fastapi.responses import JSONResponse
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from fastapi.middleware.cors import CORSMiddleware
from src.models import QueryRequest
from src.agent import run_agent
from src.ingestion import get_urls_from_sitemap, fetch_and_clean_html, chunk_text, embed_chunks, upsert_vectors
import os
from loguru import logger

app = FastAPI()

origins = [
    "http://localhost:3000",
]

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

security = HTTPBearer()

logger.add("/tmp/app.log", rotation="500 MB")

@app.exception_handler(Exception)
async def unhandled_exception_handler(request: Request, exc: Exception):
    logger.error(f"Unhandled exception: {exc}")
    return JSONResponse(
        status_code=500,
        content={"message": "Internal Server Error"},
    )

def verify_token(credentials: HTTPAuthorizationCredentials = Depends(security)):
    if credentials.scheme != "Bearer" or credentials.credentials != os.getenv("INGEST_AUTH_TOKEN"):
        logger.error("Invalid token")
        raise HTTPException(status_code=401, detail="Invalid or missing token")
    return credentials

@app.get("/")
def read_root():
    logger.info("Root endpoint called")
    return {"Hello": "World"}

@app.post("/query")
def query(request: QueryRequest):
    logger.info(f"Query received: {request.query}")
    answer = run_agent(request.query, request.top_k, request.selected_text)
    logger.info(f"Answer returned: {answer}")
    return {"answer": answer}

@app.post("/ingest", dependencies=[Depends(verify_token)])
def ingest():
    logger.info("Ingestion started")
    sitemap_url = "https://amaanullah92.github.io/Physical-AI-Humanoid-Robotics-Book/sitemap.xml"
    urls = get_urls_from_sitemap(sitemap_url)
    total_pages = len(urls)
    total_chunks = 0
    total_vectors = 0
    for url in urls:
        logger.info(f"Ingesting {url}")
        text = fetch_and_clean_html(url)
        chunks = chunk_text(text)
        total_chunks += len(chunks)
        embeddings = embed_chunks(chunks)
        total_vectors += len(embeddings)
        payloads = [{"text": chunk, "url": url} for chunk in chunks]
        upsert_vectors("physical_ai_book", embeddings, payloads)
    logger.info("Ingestion complete")
    logger.info(f"Total pages: {total_pages}")
    logger.info(f"Total chunks: {total_chunks}")
    logger.info(f"Total vectors: {total_vectors}")
    return {"status": "Ingestion complete", "pages": total_pages, "chunks": total_chunks, "vectors": total_vectors}
