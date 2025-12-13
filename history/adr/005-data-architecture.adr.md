# ADR-005: Data Architecture Decision

## Status
Accepted

## Date
2025-12-11

## Context
The educational platform needs to manage various types of data including learning modules, user profiles, progress tracking, interactive elements, and hardware configurations. The original plan specified file-based storage for modules, documentation, and configuration. The system needs to support personalized learning experiences, progress tracking, and content management while maintaining simplicity and reliability for educational use cases.

## Decision
We will implement a hybrid data architecture:

- **Content Storage**: File-based (Markdown/MDX files) for learning modules and documentation
- **Runtime Data**: In-memory and local file storage for user profiles and progress tracking
- **Models**: JavaScript objects and classes in `src/models/` for runtime data representation
- **Persistence**: Local file storage using StorageService in `src/services/storage-service.js`
- **Structure**: Data models in `physical-AI-book/src/models/` including:
  - LearningModule for course content
  - UserProfile for user preferences and progress
  - InteractiveElement for personalization and translation hooks
  - Subagent for specialized AI assistance
  - SkillWorkflow for recurring processes

This approach balances simplicity for the educational use case with necessary functionality for personalization and progress tracking.

## Consequences

### Positive
- Simple and straightforward implementation for educational use case
- No need for external database dependencies
- Easy backup and restoration of content
- Natural fit for documentation-based content
- Good performance for read-heavy educational content
- Easy to version control alongside the content

### Negative
- Limited scalability for very large numbers of concurrent users
- Potential performance issues with large datasets
- No real-time synchronization between users
- Less robust than database solutions for concurrent access
- Manual management of data consistency and validation

## Alternatives Considered

### Alternative 1: Full Database Solution
Use a full database (PostgreSQL, MongoDB, etc.) for all data storage
- Pros: Better scalability, ACID compliance, advanced querying, concurrent access handling
- Cons: Increased complexity, additional infrastructure, maintenance overhead, unnecessary for educational use case

### Alternative 2: Client-Side Only Storage
Store all data in browser local storage/session storage
- Pros: Extremely simple, no server-side persistence needed
- Cons: Data loss on browser clearing, no sharing between devices, limited storage space, no admin access to data

### Alternative 3: Hybrid Approach with External API
Keep content file-based but connect to external services for user data
- Pros: Balance of simplicity for content with robustness for user data
- Cons: Introduces external dependencies, complexity of external connections, potential privacy concerns

## References
- `/specs/001-physical-ai-book/plan.md` - Storage specification in Technical Context
- `/specs/001-physical-ai-book/data-model.md` - Detailed data models
- `/specs/001-physical-ai-book/contracts/api-contracts.md` - Data-related API specifications
- `physical-AI-book/src/models/` - Actual data model implementations
- `physical-AI-book/src/services/storage-service.js` - Storage service implementation