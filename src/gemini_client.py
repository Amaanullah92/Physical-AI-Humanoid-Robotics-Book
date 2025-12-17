import os
from dotenv import load_dotenv
from openai import OpenAI

load_dotenv()

GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")
AGENT_BASE_URL = os.getenv("AGENT_BASE_URL")

gemini_client = OpenAI(
    api_key=GEMINI_API_KEY,
    base_url=AGENT_BASE_URL,
)
