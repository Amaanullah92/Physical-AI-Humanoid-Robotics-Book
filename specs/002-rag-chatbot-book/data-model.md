# Data Model: RAG Chatbot

This document describes the data model for the RAG chatbot.

## 1. Qdrant Collection: `physical_ai_book`

This collection stores the vector embeddings of the book's content.

### 1.1. Vector Parameters

-   **`size`**: 1024 (to match Cohere's `embed-english-v3.0` model)
-   **`distance`**: `Cosine`

### 1.2. Payload Schema

| Field             | Type      | Description                                                                 |
| ----------------- | --------- | --------------------------------------------------------------------------- |
| `text`            | `string`  | The text content of the chunk.                                              |
| `url`             | `keyword` | The URL of the page from which the chunk was extracted.                     |
| `title`           | `string`  | The title of the page.                                                      |
| `section_heading` | `string`  | The heading of the section from which the chunk was extracted.              |
| `chunk_index`     | `integer` | The index of the chunk within the section.                                  |
| `chunk_id`        | `string`  | A deterministic ID for the chunk (SHA256 of URL, section heading, and index). |

## 2. API Data Structures

### 2.1. `/query` Request Body

| Field           | Type     | Description                                                    |
| --------------- | -------- | -------------------------------------------------------------- |
| `query`         | `string` | The user's question.                                           |
| `top_k`         | `integer`| (Optional) The number of results to retrieve. Default is 5.    |
| `selected_text` | `string` | (Optional) The user's selected text.                           |
| `filter_url`    | `string` | (Optional) A URL to filter the retrieval by.                   |

### 2.2. `/query` Response Body

| Field    | Type    | Description                                                              |
| -------- | ------- | ------------------------------------------------------------------------ |
| `answer` | `string`  | The chatbot's answer.                                                    |
| `sources`| `array` | An array of source objects that the answer was based on.                 |
| `...`    | `...`   | Other fields from the OpenAI Agents SDK response may also be present.    |

### 2.3. Source Object

| Field   | Type     | Description                               |
| ------- | -------- | ----------------------------------------- |
| `url`   | `string` | The URL of the source.                    |
| `title` | `string` | The title of the source page.             |
| `section`| `string` | The section heading of the source.        |
| `score` | `float`  | The similarity score of the source chunk. |
