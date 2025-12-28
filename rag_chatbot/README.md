# Educational RAG Chatbot Backend

This is the backend scaffold for the Educational RAG Chatbot, designed as a reference assistant for the educational robotics book project.

## Purpose
- Educational reference assistant for published book content
- Retrieval-based answering only
- Supports comprehension, not generation of new instructional material

## Architecture Components
- Content Management Layer
- Embedding and Indexing Layer
- Vector Storage Layer
- Query Processing Layer
- Answer Generation Layer

## Educational Boundaries
- Read-only access to finalized book content
- No user account management
- No analytics, tracking, or telemetry
- No autonomous behavior
- No external content integration

## Setup
1. Copy `.env.example` to `.env` and fill in your API keys
2. Install dependencies: `pip install -r requirements.txt`