from pydantic import BaseModel
from typing import Optional

class QueryRequest(BaseModel):
    query: str
    top_k: Optional[int] = 5
    selected_text: Optional[str] = None
    filter_url: Optional[str] = None
