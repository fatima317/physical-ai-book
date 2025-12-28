---
id: 0015
title: RAG Logic Implementation Architecture Plan
stage: plan
date: 2025-12-27
surface: agent
model: claude-sonnet-4-5-20250929
feature: 001-physical-ai-book
branch: main
user: default_user
command: /sp.plan
labels: [architecture, plan, rag, chatbot, education, implementation]
links:
  spec: 0010-rag-chatbot-specification-amendment.spec.prompt.md
  spec: 0014-rag-chatbot-implementation-authorization-amendment.spec.prompt.md
  tasks: 0012-rag-chatbot-revised-task-list.tasks.prompt.md
  ticket: null
  adr: null
  pr: null
files:
 - /mnt/d/Q4/hackathon_1/history/prompts/001-physical-ai-book/0015-rag-logic-implementation-architecture-plan.plan.prompt.md
tests:
---

## Prompt

Create an implementation-level architectural plan
for RAG logic inside the existing rag_chatbot backend scaffold.

Constraints:
- Must use existing folders and files
- No new directories
- No UI
- No deployment assumptions
- Educational-only

Include:
- Embedding flow
- Retrieval flow
- Answer grounding rules
- Failure boundaries

## Response snapshot

# Implementation-Level Architectural Plan: RAG Logic in rag_chatbot Backend

## Overview
This implementation-level architectural plan defines the detailed structure for integrating RAG logic within the existing rag_chatbot backend scaffold. The plan maintains strict adherence to educational boundaries while specifying how embedding, retrieval, and answer synthesis will be implemented within the existing directory structure.

## 1. Existing Architecture Integration

### 1.1 Directory Mapping to Components
- `rag_chatbot/api/` → Educational chat API endpoints for query processing
- `rag_chatbot/models/` → Data models for request/response validation
- `rag_chatbot/services/` → Core RAG service implementations
- `rag_chatbot/data/` → Data access layer for book content
- `rag_chatbot/content/` → Content processing and segmentation
- `rag_chatbot/embedding/` → Embedding generation and management
- `rag_chatbot/vector_db/` → Vector storage and retrieval operations
- `rag_chatbot/utils/` → Utility functions for RAG operations
- `rag_chatbot/config/` → Configuration for RAG components
- `rag_chatbot/auth_hooks/` → Authentication boundary maintenance

### 1.2 Service Layer Architecture
- `services/content_service.py` → Manages access to book content
- `services/embedding_service.py` → Handles embedding generation and storage
- `services/retrieval_service.py` → Coordinates vector search and retrieval
- `services/vector_db_service.py` → Manages vector database operations

## 2. Embedding Flow Implementation

### 2.1 Content Ingestion Pipeline
- **Trigger**: Book content updates or initial indexing
- **Process**: Content from `data/` layer processed through `content/` module
- **Implementation**: `content/__init__.py` handles segmentation and preprocessing
- **Storage**: Embeddings generated via `embedding_service.py` and stored in vector DB

### 2.2 Embedding Generation Process
- **Component**: `embedding_service.py` implements Cohere integration
- **Input**: Segmented book content from content service
- **Processing**: Text chunks converted to vector embeddings using Cohere models
- **Output**: Embeddings stored in Qdrant with metadata in Neon Postgres
- **Metadata**: Chapter, section, content type, and pedagogical context stored

### 2.3 Embedding Management
- **Storage**: `vector_db_service.py` manages Qdrant Cloud integration
- **Indexing**: Efficient search indexes maintained for fast retrieval
- **Updates**: Embedding update mechanisms for content changes
- **Versioning**: Embedding versions aligned with book content versions

## 3. Retrieval Flow Implementation

### 3.1 Query Processing Layer
- **Entry Point**: `api/educational_chat.py` receives user queries
- **Validation**: `models/chat.py` validates request/response schemas
- **Processing**: Query interpreter in `services/retrieval_service.py`
- **Context**: Conversation context maintained within educational boundaries

### 3.2 Vector Search Implementation
- **Component**: `vector_db_service.py` executes similarity search
- **Input**: User query converted to embedding vector
- **Process**: Vector compared against stored embeddings in Qdrant
- **Output**: Most relevant content segments retrieved based on similarity

### 3.3 Retrieval Coordination
- **Orchestration**: `retrieval_service.py` coordinates search and retrieval
- **Filtering**: Results filtered by relevance and educational context
- **Ranking**: Retrieved segments ranked by similarity and pedagogical relevance
- **Limiting**: Maximum retrieval results controlled by configuration

## 4. Answer Synthesis and Grounding Implementation

### 4.1 Answer Generation Process
- **Component**: `retrieval_service.py` handles synthesis coordination
- **Input**: Retrieved content segments and original query
- **Process**: Answer synthesized exclusively from retrieved content
- **Output**: Educational response formatted with proper citations

### 4.2 Answer Grounding Rules
- **Rule 1**: All answers must be grounded in retrieved book content only
- **Rule 2**: No external information or knowledge may be incorporated
- **Rule 3**: Citations must accurately reference source material in book
- **Rule 4**: Responses must maintain pedagogical context of original content
- **Rule 5**: Answer synthesis must preserve accuracy of original content
- **Rule 6**: Explanations must be verifiably based on retrieved text only

### 4.3 Response Formatting
- **Component**: `api/educational_chat.py` handles response formatting
- **Citations**: Source material properly attributed in responses
- **Style**: Educational tone and approach maintained from original content
- **Validation**: Answers validated against grounding rules before delivery

## 5. Educational-Only Implementation Constraints

### 5.1 Content Boundaries
- **Access**: Read-only access to finalized book content maintained
- **Scope**: Answers limited to book content without external expansion
- **Fidelity**: Maintains pedagogical approach and style of original book
- **Integrity**: No modification, creation, or extension of book content

### 5.2 Functional Boundaries
- **Purpose**: Educational reference assistance only
- **Behavior**: No autonomous decision-making capabilities
- **Integration**: Maintains separation from authentication systems
- **Tracking**: No analytics, telemetry, or user behavior monitoring

## 6. Failure Boundaries and Error Handling

### 6.1 Content Access Failures
- **Boundary**: If book content unavailable, return educational-appropriate error
- **Implementation**: `content_service.py` handles content access errors
- **Fallback**: Graceful degradation with helpful educational guidance
- **Logging**: Errors recorded for system maintenance (not user tracking)

### 6.2 Embedding Generation Failures
- **Boundary**: If embedding service fails, maintain educational service availability
- **Implementation**: `embedding_service.py` implements retry and fallback mechanisms
- **Fallback**: Graceful degradation to reduced functionality
- **Validation**: Embedding quality checks prevent poor retrieval results

### 6.3 Vector Search Failures
- **Boundary**: If vector database unavailable, return appropriate educational response
- **Implementation**: `vector_db_service.py` handles connection and query errors
- **Fallback**: Alternative search methods or educational guidance
- **Resilience**: Circuit breaker patterns to prevent cascading failures

### 6.4 Answer Synthesis Failures
- **Boundary**: If answer synthesis fails, return educational-appropriate response
- **Implementation**: `retrieval_service.py` validates answer quality before delivery
- **Fallback**: Suggest alternative ways to find information in book content
- **Safety**: Prevents delivery of ungrounded or inappropriate responses

## 7. Configuration and Validation

### 7.1 Configuration Management
- **Location**: `config/__init__.py` manages RAG-specific configurations
- **Parameters**: Similarity thresholds, maximum retrieval results, content boundaries
- **Validation**: Configuration values validated against educational constraints
- **Security**: API keys and sensitive data handled through environment variables

### 7.2 Educational Boundary Validation
- **Component**: `utils/` module contains validation functions
- **Process**: All responses validated against grounding rules before delivery
- **Monitoring**: System checks ensure educational boundaries maintained
- **Compliance**: Regular validation of answer synthesis against book content

## 8. Implementation Sequence

### 8.1 Phase 1: Foundation
1. Implement `content_service.py` for book content access
2. Implement `embedding_service.py` for Cohere integration
3. Implement `vector_db_service.py` for Qdrant operations

### 8.2 Phase 2: Retrieval
1. Implement `retrieval_service.py` for search coordination
2. Implement `api/educational_chat.py` for query processing
3. Implement `models/chat.py` for data validation

### 8.3 Phase 3: Synthesis
1. Complete answer synthesis in `retrieval_service.py`
2. Implement grounding validation in `utils/` module
3. Integrate response formatting and citation generation

This implementation-level architectural plan ensures RAG logic is properly integrated within the existing rag_chatbot backend scaffold while maintaining all educational boundaries and constraints.

## Outcome

- ✅ Impact: Created detailed implementation-level architectural plan for RAG logic within existing scaffold
- 🧪 Tests: No tests required for architectural document
- 📁 Files: Created new PHR documenting the RAG logic implementation architecture
- 🔁 Next prompts: Ready for task breakdown based on this implementation architecture
- 🧠 Reflection: The plan provides clear implementation guidance while maintaining educational focus and proper component organization

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A