---
id: 014
title: Fix Qdrant Collection Not Found Error
stage: implement
date: 2025-12-17
surface: agent
model: gemini
feature: 002-rag-chatbot-book
branch: 002-rag-chatbot-book
user: 
command: |
  File "C:\Users\Pc World\AppData\Local\Programs\Python\Python312\Lib\site-packages\qdrant_client\http\api_client.py", line 130, in send
      raise UnexpectedResponse.for_response(response)
  qdrant_client.http.exceptions.UnexpectedResponse: Unexpected Response: 404 (Not Found)
  Raw response content:
  b'{"status":{"error":"Not found: Collection `physical_ai_book` doesn\'t exist!"},"time":0.000088897}'
labels: []
links: {}
files_yaml: |
  - src/ingestion.py
tests_yaml: |

prompt_text: |
  File "C:\Users\Pc World\AppData\Local\Programs\Python\Python312\Lib\site-packages\qdrant_client\http\api_client.py", line 130, in send
      raise UnexpectedResponse.for_response(response)
  qdrant_client.http.exceptions.UnexpectedResponse: Unexpected Response: 404 (Not Found)
  Raw response content:
  b'{"status":{"error":"Not found: Collection `physical_ai_book` doesn\'t exist!"},"time":0.000088897}'
response_text: |
  Fixed the Qdrant "Collection doesn't exist!" error by adding logic to create the collection if it does not already exist.
---