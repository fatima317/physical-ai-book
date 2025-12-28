---
id: "0014"
title: "rag-chatbot-embedding-implementation-tasks"
stage: "tasks"
date: "2025-12-27"
surface: "agent"
model: "claude-sonnet-4-5-20250929"
feature: "001-physical-ai-book"
branch: "main"
user: "claude"
command: "/sp.tasks"
labels: ["rag", "chatbot", "embedding", "docusaurus", "fastapi"]
links:
  spec: "null"
  ticket: "null"
  adr: "null"
  pr: "null"
files: []
tests: []
---

# Prompt History Record

## PROMPT_TEXT

```
/sp.tasks

Generate implementation tasks strictly and only according to
Specification Amendment 0013 (Educational RAG Chatbot Embedding).

HARD CONSTRAINTS — DO NOT VIOLATE:

1. Tasks MUST be limited to:
   - Client-side script loading in existing Docusaurus book
   - Calling existing FastAPI endpoints
   - Passing user query text
   - Passing selected page text
   - Rendering plain text responses

2. Tasks MUST NOT include:
   - UI elements, widgets, components, layout, styling
   - Directories, folder structures, or architectural decisions
   - Testing, verification, validation, or checks of any kind
   - Analytics, logging, monitoring
   - Authentication or sessions
   - State, persistence, storage
   - Documentation or comments
   - Performance or optimization

3. Assumptions:
   - Anonymous
   - Read-only
   - Educational-only
   - No backend changes
   - No new features beyond embedding

4. Output format:
   - Single flat checklist
   - ≤ 6 tasks
   - No explanations
   - No headings

If any task falls outside Amendment 0013, STOP and do not generate output.
```

## RESPONSE_TEXT

- [ ] Create client-side JavaScript file for embedding in Docusaurus book
- [ ] Implement function to call existing FastAPI /api/v1/query endpoint with user query
- [ ] Implement function to call existing FastAPI /api/v1/query-selected-text endpoint with selected text
- [ ] Add functionality to capture selected text from book pages
- [ ] Implement response display for text received from backend
- [ ] Integrate script loading into Docusaurus book pages