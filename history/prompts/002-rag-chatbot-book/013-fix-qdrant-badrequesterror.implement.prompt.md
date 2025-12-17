---
id: 013
title: Fix Qdrant API BadRequestError
stage: implement
date: 2025-12-17
surface: {{SURFACE}}
model: {{MODEL}}
feature: 002-rag-chatbot-book
branch: 002-rag-chatbot-book
user: {{USER}}
command: {{COMMAND}}
labels: [{{LABELS}}]
links:
  spec: {{LINKS_SPEC}}
  ticket: {{LINKS_TICKET}}
  adr: {{LINKS_ADR}}
  pr: {{LINKS_PR}}
files:
{{FILES_YAML}}
tests:
{{TESTS_YAML}}
---

## Prompt

File "C:\Users\Pc World\AppData\Local\Programs\Python\Python312\Lib\site-packages\qdrant_client\http\api_client.py", line 95, in request    return self.send(request, type_)           ^^^^^^^^^^^^^^^^^^^^^^^^^  File "C:\Users\Pc World\AppData\Local\Programs\Python\Python312\Lib\site-packages\qdrant_client\http\api_client.py", line 130, in send    raise UnexpectedResponse.for_response(response)qdrant_client.http.exceptions.UnexpectedResponse: Unexpected Response: 400 (Bad Request)Raw response content:b'{"status":{"error":"Format error in JSON body: value d09c8d9fd85b70c604cfe8d437b4cfa63d1acf9111c4e31543f7dea7c4d535d8 is not a valid point ID, valid values are either an unsigned integer or a UUID" ...'

## Response snapshot

Fixed the Qdrant API BadRequestError by updating the ID generation to use UUIDs.

## Outcome

- âœ… Impact: {{OUTCOME_IMPACT}}
- ðŸ§ª Tests: {{TESTS_SUMMARY}}
- ðŸ“ Files: {{FILES_SUMMARY}}
- ðŸ” Next prompts: {{NEXT_PROMPTS}}
- ðŸ§  Reflection: {{REFLECTION_NOTE}}

## Evaluation notes (flywheel)

- Failure modes observed: {{FAILURE_MODES}}
- Graders run and results (PASS/FAIL): {{GRADER_RESULTS}}
- Prompt variant (if applicable): {{PROMPT_VARIANT_ID}}
- Next experiment (smallest change to try): {{NEXT_EXPERIMENT}}
