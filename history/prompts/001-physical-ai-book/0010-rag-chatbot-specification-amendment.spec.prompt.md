---
id: 0010
title: RAG Chatbot Specification Amendment
stage: spec
date: 2025-12-25
surface: agent
model: claude-sonnet-4-5-20250929
feature: 001-physical-ai-book
branch: main
user: default_user
command: /sp.specify
labels: [specification, amendment, rag, chatbot, education]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - /mnt/d/Q4/hackathon_1/history/prompts/001-physical-ai-book/0010-rag-chatbot-specification-amendment.spec.prompt.md
tests:
---

## Prompt

Add a corrective specification amendment for an Integrated Educational RAG Chatbot.

Context:
- This project is an educational robotics book, not a SaaS platform
- A Retrieval-Augmented Generation (RAG) chatbot is a supporting educational feature only
- Existing amendments for Authentication, Personalization, and Localization remain authoritative
- This amendment MUST NOT introduce planning, tasks, workflows, or implementation assumptions

Purpose of Amendment:
- To declaratively define scope boundaries and constraints for an educational RAG chatbot
- To prevent platform drift or SaaS assumptions

Specification Amendment:

1. RAG Chatbot Purpose
- The RAG chatbot is an educational reference assistant for the published book
- It answers learner questions using only published book content
- It may optionally answer based solely on user-selected excerpts when explicitly requested
- It exists to support comprehension, not to generate new instructional material

2. Scope and Authority
- Read-only access to finalized book content
- Retrieval-based answering only
- Explanatory clarification is permitted only when directly grounded in retrieved book text
- No authority to modify, extend, rewrite, or reinterpret book content

3. Technology Declaration (Non-Operational, Informational Only)
- Embeddings: Cohere models
- Vector Store: Qdrant Cloud (Free Tier)
- Metadata Store: Neon Serverless Postgres
- Agent Interface: OpenAI Agents / ChatKit SDK
- Backend API: FastAPI
- This declaration does not authorize implementation

4. Explicit Constraints
- This is not a SaaS chatbot
- No user accounts are defined or managed by the RAG system
- No analytics, tracking, telemetry, or monetization
- No autonomous behavior or agent decision-making
- No UI, UX, frontend, or deployment assumptions
- No quality metrics, confidence scores, or evaluation logic
- No task creation, workflows, or operational behaviors

5. Amendment Authority
- This specification is additive and corrective only
- It does not override the project constitution
- It does not trigger planning or execution phases

Constraints:
- Specification text only
- No planning
- No tasks
- No implementation

## Response snapshot

# Corrective Specification Amendment: Integrated Educational RAG Chatbot

## Amendment Overview
This corrective specification amendment defines the scope boundaries and constraints for an integrated Retrieval-Augmented Generation (RAG) chatbot within the educational robotics book project. This amendment serves to prevent platform drift or SaaS assumptions while maintaining focus on the educational nature of the project.

## 1. RAG Chatbot Purpose
- The RAG chatbot is an educational reference assistant for the published book
- It answers learner questions using only published book content
- It may optionally answer based solely on user-selected excerpts when explicitly requested
- It exists to support comprehension, not to generate new instructional material
- The system serves as a pedagogical enhancement to the existing educational content

## 2. Scope and Authority
- Read-only access to finalized book content
- Retrieval-based answering only
- Explanatory clarification is permitted only when directly grounded in retrieved book text
- No authority to modify, extend, rewrite, or reinterpret book content
- Must maintain fidelity to the original educational material and pedagogical approach
- Cannot introduce external information or knowledge beyond the book's scope

## 3. Technology Declaration (Non-Operational, Informational Only)
- **Embeddings**: Cohere models
- **Vector Store**: Qdrant Cloud (Free Tier)
- **Metadata Store**: Neon Serverless Postgres
- **Agent Interface**: OpenAI Agents / ChatKit SDK
- **Backend API**: FastAPI
- This declaration does not authorize implementation
- Technology choices are specified for reference only, not execution

## 4. Explicit Constraints
- This is not a SaaS chatbot
- No user accounts are defined or managed by the RAG system
- No analytics, tracking, telemetry, or monetization
- No autonomous behavior or agent decision-making
- No UI, UX, frontend, or deployment assumptions
- No quality metrics, confidence scores, or evaluation logic
- No task creation, workflows, or operational behaviors
- No integration with external systems beyond book content access
- No data collection, user profiling, or behavioral analysis

## 5. Amendment Authority
- This specification is additive and corrective only
- It does not override the project constitution
- It does not trigger planning or execution phases
- It maintains compatibility with existing Authentication, Personalization, and Localization amendments
- This amendment focuses solely on declarative specification without implementation assumptions
- All boundaries defined here are enforceable constraints on future development

This corrective amendment ensures the RAG chatbot remains within educational boundaries and does not introduce platform assumptions inconsistent with the educational robotics book project.

## Outcome

- ✅ Impact: Created corrective specification amendment for educational RAG chatbot that maintains educational focus without introducing platform assumptions
- 🧪 Tests: No tests required for specification document
- 📁 Files: Created new PHR documenting the RAG chatbot specification amendment
- 🔁 Next prompts: Ready for implementation planning within defined boundaries
- 🧠 Reflection: The specification clearly defines scope boundaries to prevent platform drift while supporting the educational purpose

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A