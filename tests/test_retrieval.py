import pytest
from src.retrieval import search_qdrant, get_embedding
from src.qdrant_client import qdrant_client

@pytest.fixture(scope="module", autouse=True)
def setup_qdrant_collection():
    collection_name = "test_physical_ai_book"
    qdrant_client.recreate_collection(
        collection_name=collection_name,
        vectors_config={"size": 1024, "distance": "Cosine"}
    )
    # Add some dummy data for testing
    qdrant_client.upsert(
        collection_name=collection_name,
        points=[
            {
                "id": "1",
                "vector": get_embedding("test query about stepper motors"),
                "payload": {"text": "Stepper motors are good for precise movements.", "url": "http://example.com/stepper"}
            },
            {
                "id": "2",
                "vector": get_embedding("test query about servo motors"),
                "payload": {"text": "Servo motors are good for high power applications.", "url": "http://example.com/servo"}
            },
        ]
    )
    yield
    qdrant_client.delete_collection(collection_name=collection_name)


def test_search_qdrant():
    query = "What is a stepper motor?"
    embedding = get_embedding(query)
    search_results = search_qdrant(embedding, top_k=1)
    
    assert len(search_results.points) == 1
    assert "stepper motors" in search_results.points[0].payload["text"].lower()
