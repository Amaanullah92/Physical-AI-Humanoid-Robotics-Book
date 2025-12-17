import os
from dotenv import load_dotenv
import cohere

load_dotenv()

COHERE_API_KEY = os.getenv("COHERE_API_KEY")

cohere_client = cohere.Client(COHERE_API_KEY)
