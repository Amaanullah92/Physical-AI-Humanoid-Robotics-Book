import os
from dotenv import load_dotenv
from src.cohere_client import cohere_client
from src.qdrant_client import qdrant_client
load_dotenv()
COHERE_EMBED_MODEL = os.getenv("COHERE_EMBED_MODEL")
def get_embedding(query: str):
    response = cohere_client.embed(
        texts=[query],
        model=COHERE_EMBED_MODEL,
        input_type="search_query",
    )
    return response.embeddings[0]

def search_qdrant(embedding: list, top_k: int):
    search_result = qdrant_client.query_points(
        collection_name="physical_ai_book",
        query=embedding,
        limit=top_k,
    )
    return search_result

def embed_selected_text(text: str):
    response = cohere_client.embed(
        texts=[text],
        model=COHERE_EMBED_MODEL,
        input_type="search_document",
    )
    return response.embeddings[0]
