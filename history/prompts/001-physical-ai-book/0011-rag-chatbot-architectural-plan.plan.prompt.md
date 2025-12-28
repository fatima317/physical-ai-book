---
id: 0011
title: RAG Chatbot Architectural Plan
stage: plan
date: 2025-12-25
surface: agent
model: claude-sonnet-4-5-20250929
feature: 001-physical-ai-book
branch: main
user: default_user
command: /sp.plan
labels: [architecture, plan, rag, chatbot, education]
links:
  spec: 0010-rag-chatbot-specification-amendment.spec.prompt.md
  ticket: null
  adr: null
  pr: null
files:
 - /mnt/d/Q4/hackathon_1/history/prompts/001-physical-ai-book/0011-rag-chatbot-architectural-plan.plan.prompt.md
tests:
---

## Prompt

Create a high-level architectural plan for the Integrated Educational RAG Chatbot
based strictly on the approved specification amendments and project constitution.

Scope:
- Educational robotics book only
- Supporting reference assistant
- Read-only retrieval from published book content

Include:
- Conceptual data flow (book → embeddings → retrieval → answer)
- Separation of concerns (book content, metadata, vector store, agent)
- How "answer from selected text only" is handled conceptually
- Alignment with existing Authentication, Personalization, and Localization amendments

Constraints:
- No code
- No APIs
- No UI
- No SaaS assumptions
- No task breakdown yet
- No implementation details

## Response snapshot

# High-Level Architectural Plan: Integrated Educational RAG Chatbot

## Overview
This architectural plan defines the high-level structure for an Integrated Educational RAG Chatbot that serves as a reference assistant for the published educational robotics book. The architecture maintains strict adherence to the specification amendments, focusing on educational support without introducing platform assumptions.

## 1. Core Architecture Components

### 1.1 Book Content Management Layer
- **Source Content Repository**: Read-only access to finalized book content in various formats (text, code examples, diagrams, mathematical formulas)
- **Content Parser**: Extracts and structures book content for processing while maintaining pedagogical context
- **Content Validator**: Ensures content integrity and consistency with published material

### 1.2 Embedding and Indexing Layer
- **Embedding Engine**: Uses Cohere models to create vector representations of book content segments
- **Segmentation Logic**: Divides book content into semantically coherent chunks appropriate for retrieval
- **Metadata Enrichment**: Adds contextual metadata to embeddings (chapter, section, content type, difficulty level)

### 1.3 Vector Storage Layer
- **Vector Database**: Qdrant Cloud (Free Tier) for storing and retrieving vector embeddings
- **Index Management**: Maintains efficient search indexes for fast retrieval
- **Similarity Engine**: Implements vector similarity search algorithms for content matching

### 1.4 Query Processing Layer
- **Query Interpreter**: Processes user questions and extracts semantic meaning
- **Context Manager**: Maintains conversation context while respecting educational boundaries
- **Retrieval Coordinator**: Orchestrates the search and retrieval process from vector store

### 1.5 Answer Generation Layer
- **Retrieval Processor**: Combines retrieved content with user query for answer synthesis
- **Answer Formatter**: Ensures responses align with book's pedagogical style
- **Citation Generator**: Provides appropriate references to source material in the book

## 2. Conceptual Data Flow

### 2.1 Primary Flow (Book → Embeddings → Retrieval → Answer)
1. **Content Ingestion**: Published book content is processed into structured segments
2. **Embedding Generation**: Cohere models create vector embeddings for each content segment
3. **Index Storage**: Embeddings are stored in Qdrant vector database with metadata
4. **Query Processing**: User questions are embedded and compared against stored vectors
5. **Retrieval**: Most relevant content segments are retrieved based on vector similarity
6. **Answer Synthesis**: Retrieved content is synthesized into educational responses
7. **Response Delivery**: Answers are formatted and delivered while citing sources

### 2.2 "Answer from Selected Text Only" Flow
1. **Text Selection Detection**: System identifies when user explicitly requests answers based on selected text
2. **Context Isolation**: Focuses retrieval process solely on the user-selected text segment
3. **Constrained Retrieval**: Limits vector search to embeddings derived from the selected text
4. **Focused Answering**: Generates responses exclusively from the selected text context
5. **Boundary Enforcement**: Ensures no external content is incorporated into the response

## 3. Separation of Concerns

### 3.1 Book Content Domain
- Responsibility: Maintaining fidelity to published book content
- Boundaries: Read-only access, no modification capabilities
- Interfaces: Content parsing APIs, metadata extraction tools

### 3.2 Metadata Management Domain
- Responsibility: Managing document structure, relationships, and context
- Storage: Neon Serverless Postgres for metadata persistence
- Content: Document hierarchy, content relationships, pedagogical context markers

### 3.3 Vector Store Domain
- Responsibility: Efficient storage and retrieval of semantic embeddings
- Technology: Qdrant Cloud (Free Tier)
- Content: Vector embeddings, similarity indexes, retrieval metadata

### 3.4 Agent Interface Domain
- Responsibility: Managing conversation flow and user interaction
- Technology: OpenAI Agents / ChatKit SDK
- Scope: Query processing, response generation, context management

## 4. Integration with Existing Amendments

### 4.1 Authentication Amendment Integration
- The RAG system does not manage user accounts or authentication
- Authentication boundaries remain with the existing Authentication amendment
- User identification (if needed) is handled by the authentication layer

### 4.2 Personalization Amendment Integration
- Leverages personalization data to customize response delivery
- Respects user preferences for content presentation
- Maintains personalization boundaries as defined in the amendment

### 4.3 Localization Amendment Integration
- Answers are generated in the user's preferred language as defined by localization settings
- Technical terminology maintains consistency with localized book content
- Cultural context is preserved according to localization preferences

## 5. Architectural Principles

### 5.1 Educational Fidelity
- All responses must be grounded in published book content
- No external information or assumptions are introduced
- Maintains the pedagogical approach and style of the original book

### 5.2 Retrieval-Only Operation
- System operates in read-only mode with respect to book content
- No content modification, creation, or extension capabilities
- Answers are synthesized exclusively from retrieved content

### 5.2 Boundary Preservation
- Strict separation between educational content and system functionality
- No SaaS or platform assumptions introduced
- Maintains focus on educational support rather than commercial features

## 6. Non-Functional Requirements

### 6.1 Performance
- Fast retrieval response times to maintain educational flow
- Efficient vector search operations
- Optimized content segmentation for quick access

### 6.2 Reliability
- Consistent access to book content
- Robust error handling for unavailable resources
- Fallback mechanisms when retrieval fails

### 6.3 Educational Accuracy
- High precision in content retrieval
- Faithful representation of book content
- Proper attribution and citation of sources

## 7. Risk Mitigation

### 7.1 Content Drift Prevention
- Regular validation against published book content
- Version control alignment with book updates
- Automated checks for content consistency

### 7.2 Scope Creep Prevention
- Clear architectural boundaries preventing feature expansion
- Regular validation against specification amendments
- Strict adherence to educational-only purpose

This architectural plan maintains the educational focus while providing a clear structure for the RAG chatbot system that aligns with all existing amendments and project constitution.

## Outcome

- ✅ Impact: Created comprehensive architectural plan for educational RAG chatbot that aligns with specification amendments
- 🧪 Tests: No tests required for architectural document
- 📁 Files: Created new PHR documenting the RAG chatbot architectural plan
- 🔁 Next prompts: Ready for task breakdown based on this architectural foundation
- 🧠 Reflection: The plan maintains educational focus while clearly defining technical architecture and integration points

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A