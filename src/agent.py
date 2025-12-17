from src.retrieval import get_embedding, search_qdrant, embed_selected_text
from src.gemini_client import gemini_client

def retrieval_tool(query: str, top_k: int):
    """
    This tool retrieves relevant information from the Qdrant collection.
    """
    embedding = get_embedding(query)
    search_results = search_qdrant(embedding, top_k)
    return [result.payload["text"] for result in search_results.points]

def run_agent(query: str, top_k: int, selected_text: str = None):
    """
    This function runs the RAG agent.
    """
    if selected_text is not None and not selected_text.strip():
        return "Please select some text to get an answer based on your selection."
    
    if selected_text:
        retrieved_context = [selected_text]
    else:
        retrieved_context = retrieval_tool(query, top_k)
    
    system_prompt = f"""
    You are a helpful assistant for the book "Physical AI & Humanoid Robotics".
    You must answer the user's question based only on the following context:
    
    {retrieved_context}
    
    If the context is not sufficient to answer the question, you must say so.
    Do not make up information.
    """
    
    response = gemini_client.chat.completions.create(
        model="gemini-2.5-flash",
        messages=[
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": query},
        ],
    )
    
    return response.choices[0].message.content
