---
id: 022
title: Fix OpenAI NotFoundError for Gemini Model
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

File "C:\Users\Pc World\AppData\Local\Programs\Python\Python312\Lib\site-packages\openai\_base_client.py", line 1047, in 
request
    raise self._make_status_error_from_response(err.response) from None
openai.NotFoundError: Error code: 404 - [{'error': {'code': 404, 'message': 'models/gemini-pro is not found for API version v1main, or is not supported for generateContent. Call ListModels to see the list of available models and their supported methods.', 'status': 'NOT_FOUND'}}]

## Response snapshot

Fixed the openai.NotFoundError by changing the model to gemini-1.5-flash.

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
