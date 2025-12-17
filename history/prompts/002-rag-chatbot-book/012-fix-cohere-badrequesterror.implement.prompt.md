---
id: 012
title: Fix Cohere API BadRequestError
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

File "C:\Users\Pc World\AppData\Local\Programs\Python\Python312\Lib\site-packages\cohere\raw_base_client.py", line 1669, in embed
    raise BadRequestError(
cohere.errors.bad_request_error.BadRequestError: headers: {'access-control-expose-headers': 'X-Debug-Trace-ID', 'cache-control': 'no-cache, no-store, no-transform, must-revalidate, private, max-age=0', 'content-encoding': 'gzip', 'content-type': 'application/json', 'expires': 'Thu, 01 Jan 1970 00:00:00 GMT', 'pragma': 'no-cache', 'vary': 'Origin,Accept-Encoding', 'x-accel-expires': '0', 'x-debug-trace-id': '4746ed1cf0db8a286bcfb3971321a3f4', 'x-endpoint-monthly-call-limit': '1000', 'x-trial-endpoint-call-limit': '100', 'x-trial-endpoint-call-remaining': '99', 'date': 'Wed, 17 Dec 2025 18:49:20 GMT', 'x-envoy-upstream-service-time': '19', 'server': 'envoy', 'via': '1.1 google', 'alt-svc': 'h3=":443"; ma=2592000,h3-29=":443"; ma=2592000', 'transfer-encoding': 'chunked'}, status_code: 400, body: {'id': 'd99ac588-3cf1-4c6b-a32f-aa9c61dc9717', 'message': 'invalid request: valid input_type must be provided with the provided model'}

## Response snapshot

Fixed the Cohere API BadRequestError by adding the input_type parameter to all cohere_client.embed calls.

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
