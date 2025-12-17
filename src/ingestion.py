import os
from dotenv import load_dotenv
import requests
import xml.etree.ElementTree as ET
from bs4 import BeautifulSoup
import hashlib
import uuid
from src.cohere_client import cohere_client
from src.qdrant_client import qdrant_client
from qdrant_client.http.models import PointStruct
import tiktoken
from fastapi import FastAPI, Header, HTTPException

load_dotenv()
COHERE_EMBED_MODEL = os.getenv("COHERE_EMBED_MODEL")
INGEST_AUTH_TOKEN = os.getenv("INGEST_AUTH_TOKEN")
app = FastAPI()

def get_urls_from_sitemap(sitemap_url: str):
    response = requests.get(sitemap_url, timeout=10)
    root = ET.fromstring(response.content)
    urls = [element.text for element in root.findall(".//{http://www.sitemaps.org/schemas/sitemap/0.9}loc")]
    return urls

def fetch_and_clean_html(url: str):
    response = requests.get(url, timeout=10)
    soup = BeautifulSoup(response.content, "html.parser")
    for script in soup(["script", "style"]):
        script.extract()
    text = soup.get_text()
    lines = (line.strip() for line in text.splitlines())
    chunks = (phrase.strip() for line in lines for phrase in line.split("  "))
    text = '\n'.join(chunk for chunk in chunks if chunk)
    return text

def chunk_text(text: str, chunk_size: int = 700, chunk_overlap: int = 100):
    encoding = tiktoken.get_encoding("cl100k_base")
    tokens = encoding.encode(text)
    chunks = []
    for i in range(0, len(tokens), chunk_size - chunk_overlap):
        chunk_tokens = tokens[i:i + chunk_size]
        chunk_text = encoding.decode(chunk_tokens)
        chunks.append(chunk_text)
    return chunks

import uuid

def generate_chunk_id(chunk: str):
    # Create a SHA256 hash of the chunk
    sha256_hash = hashlib.sha256(chunk.encode()).hexdigest()
    # Convert the hash to a UUID
    return str(uuid.UUID(sha256_hash[:32]))

def embed_chunks(chunks: list):
    response = cohere_client.embed(
        texts=chunks,
        model=COHERE_EMBED_MODEL,
        input_type="search_document",
    )
    return response.embeddings

def upsert_vectors(collection_name: str, vectors: list, payloads: list):
    # Check if collection exists, create if not
    try:
        qdrant_client.get_collection(collection_name=collection_name)
    except Exception:
        qdrant_client.recreate_collection(
            collection_name=collection_name,
            vectors_config={
                "size": 1024, # As per research.md
                "distance": "Cosine"
            }
        )
    qdrant_client.upsert(
        collection_name=collection_name,
        points=[
            PointStruct(
                id=generate_chunk_id(payload["text"]),
                vector=vector,
                payload=payload
            )
            for vector, payload in zip(vectors, payloads)
        ],
        wait=True
    )

@app.post("/ingest")
def ingest(authorization: str = Header(...)):
    if authorization != f"Bearer {INGEST_AUTH_TOKEN}":
        raise HTTPException(status_code=401, detail="Invalid or missing token")
    sitemap_url = "https://amaanullah92.github.io/Physical-AI-Humanoid-Robotics-Book/sitemap.xml"
    urls = get_urls_from_sitemap(sitemap_url)
    for url in urls:
        text = fetch_and_clean_html(url)
        chunks = chunk_text(text)
        embeddings = embed_chunks(chunks)
        payloads = [{"text": chunk, "url": url} for chunk in chunks]
        upsert_vectors("physical_ai_book", embeddings, payloads)
    return {"status": "Ingestion complete", "pages": len(urls)}
