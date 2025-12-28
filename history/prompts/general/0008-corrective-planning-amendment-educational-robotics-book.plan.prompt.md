---
id: 0008
title: Corrective Planning Amendment Educational Robotics Book
stage: plan
date: 2025-12-20
surface: agent
model: claude-sonnet-4-5-20250929
feature: 001-physical-ai-book
branch: main
user: user
command: '/sp.plan Create a CORRECTIVE PLANNING AMENDMENT to replace the previous implementation plan. Context: - The previous plan violated scope by including APIs, storage, UI, backend/frontend roles. - This project is an educational robotics book, not a SaaS platform. - Authentication via Better-Auth is conceptual only. - No infrastructure, no APIs, no databases, no UI implementation. - Existing components already exist: - Skills: personalization.claude, chapter_writing.claude, code_formatting.claude - MDX: PersonalizeChapter.jsx, TranslateToUrdu.jsx, SkillAssessment.jsx, LanguagePreference.jsx Planning Constraints: - Planning must stay at SPEC-INTEGRATION level only - Use capability alignment, content flow, and verification steps - No developer roles - No implementation tasks - No system setup Output: - Replace the previous plan entirely - High-level conceptual phases only - Clear verification checkpoints - Ready for /sp.task without scope drift Do NOT mention APIs, storage, sessions, backend, frontend, or UI implementation. Do NOT proceed to implementation.'
labels: [plan, robotics, education, ai, physical-ai, architecture, corrective]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - specs/001-physical-ai-book/plan.md
tests:
 - none
---

## Prompt

/sp.plan

Create a CORRECTIVE PLANNING AMENDMENT to replace the previous implementation plan.

Context:
- The previous plan violated scope by including APIs, storage, UI, backend/frontend roles.
- This project is an educational robotics book, not a SaaS platform.
- Authentication via Better-Auth is conceptual only.
- No infrastructure, no APIs, no databases, no UI implementation.
- Existing components already exist:
  - Skills: personalization.claude, chapter_writing.claude, code_formatting.claude
  - MDX: PersonalizeChapter.jsx, TranslateToUrdu.jsx, SkillAssessment.jsx, LanguagePreference.jsx

Planning Constraints:
- Planning must stay at SPEC-INTEGRATION level only
- Use capability alignment, content flow, and verification steps
- No developer roles
- No implementation tasks
- No system setup

Output:
- Replace the previous plan entirely
- High-level conceptual phases only
- Clear verification checkpoints
- Ready for /sp.task without scope drift

Do NOT mention APIs, storage, sessions, backend, frontend, or UI implementation.
Do NOT proceed to implementation.

## Response snapshot

Created a CORRECTIVE PLANNING AMENDMENT that addresses the scope violations in the previous plan. The amendment focuses on the educational robotics book project with the following key elements:

1. **Problem Recognition**: Identified that the previous plan inappropriately included SaaS platform elements like APIs, storage, and UI implementation in an educational content project.

2. **Corrective Phases**: Established four high-level conceptual phases focused on:
   - Educational content architecture alignment
   - Conceptual integration framework
   - Content flow verification
   - Educational delivery validation

3. **Verification Checkpoints**: Created clear verification criteria to ensure all components serve educational robotics content objectives.

4. **Scope Boundaries**: Explicitly defined out-of-scope elements including infrastructure, databases, backend/frontend roles, and production API development.

The plan maintains focus on integrating existing components (skills and MDX components) for educational purposes while keeping Better-Auth as a conceptual model only. This corrective plan is now ready for the `/sp.task` command without scope drift.

The plan adheres to all specified constraints:
- Stays at the spec-integration level only
- Uses capability alignment, content flow, and verification steps
- Avoids developer roles, implementation tasks, and system setup
- Does not mention APIs, storage, sessions, backend, frontend, or UI implementation

## Outcome

- ✅ Impact: Created corrective planning amendment that realigns project scope with educational robotics book objectives
- 🧪 Tests: none
- 📁 Files: Updated plan.md with corrective planning amendment
- 🔁 Next prompts: Proceed to task generation with /sp.tasks
- 🧠 Reflection: Plan successfully corrects previous scope violations and maintains focus on educational content

## Evaluation notes (flywheel)

- Failure modes observed: Previous plan scope violations corrected
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): Corrective planning amendment
- Next experiment (smallest change to try): Generate tasks based on corrected plan