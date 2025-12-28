# Specification Amendment: Educational RAG Chatbot Embedding

## Document Information
- **Amendment ID:** 0013-rag-chatbot-embed
- **Date:** 2025-12-27
- **Feature:** Educational RAG Chatbot Integration
- **Original Spec:** 001-physical-ai-book

## Amendment Summary
This specification amendment authorizes the embedding of an Educational RAG Chatbot into the already-published GitHub Pages book. This amendment addresses the integration requirements for client-side embedding of the already-implemented backend functionality.

## 1. Embedding Scope

### 1.1 In Scope
- The RAG chatbot SHALL be embedded into the published book as a client-side integration
- The chatbot interface SHALL be a lightweight client-side interface on book pages
- Client-side JavaScript SHALL communicate with the existing backend API endpoints
- The embedded chatbot SHALL maintain all existing functionality from the standalone version
- The integration SHALL be anonymous, read-only, and educational-only

### 1.2 Out of Scope
- Backend API development (already implemented)
- New RAG logic or processing capabilities
- Changes to the existing book content structure
- Modifications to the GitHub Pages deployment process
- New data storage or processing infrastructure
- User accounts, persistence, tracking, or authentication

## 2. Integration Requirements

### 2.1 Technical Approach
- The chatbot SHALL be embedded using a JavaScript interface that loads asynchronously
- Cross-origin resource sharing (CORS) configuration SHALL already support the GitHub Pages domain
- The integration SHALL NOT interfere with existing book functionality

### 2.2 User Interface Requirements
- A lightweight client-side interface SHALL be available for user interactions
- The interface SHALL support educational queries about book content

## 3. Security and Privacy

### 3.1 Data Handling
- User queries SHALL be transmitted securely using HTTPS

## 4. Acceptance Criteria

This amendment is considered satisfied when the embedded interface can
successfully send queries and selected text from the book pages to the
existing RAG backend and display responses.

## 5. Constraints and Dependencies

### 5.1 Technical Constraints
- The solution MUST work within GitHub Pages limitations (no server-side processing)
- The integration MUST be compatible with the existing book's Jekyll/Markdown structure
- The solution MUST not interfere with existing book functionality
- No user accounts, persistence, tracking, or authentication SHALL be implemented

### 5.2 Dependencies
- The backend RAG API endpoints MUST remain stable and accessible
- The GitHub Pages site configuration MAY require CORS header updates
- The existing RAG backend implementation is assumed to be complete and functional