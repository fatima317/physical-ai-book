# ADR-003: API Architecture Decision

## Status
Accepted

## Date
2025-12-11

## Context
The system requires a backend API to support the interactive educational platform. The API needs to handle module content delivery with personalization, subagent execution, user profile management, and skill execution. The original plan specified Node.js with Express.js as the primary dependencies for API development. The system needs to support educational use cases like personalized content delivery, multilingual content, and integration with AI subagents.

## Decision
We will implement the API using:

- **Framework**: Express.js for Node.js as the web framework
- **API Style**: RESTful APIs with JSON responses
- **Endpoints**:
  - Module content API (GET /api/modules/{moduleId})
  - Subagent execution API (POST /api/subagents/{subagentId}/execute)
  - User profile API (PUT /api/users/profile)
  - Skill execution API (POST /api/skills/{skillId}/execute)
- **Middleware**: Helmet for security, CORS for cross-origin, rate limiting for protection
- **Structure**: API routes in `physical-AI-book/src/api/index.js`

This decision aligns with the API contracts specified in the original requirements and provides the necessary functionality for the educational platform.

## Consequences

### Positive
- Lightweight and performant for the educational use case
- Good integration with Node.js ecosystem
- Straightforward implementation of the required API contracts
- Well-documented patterns and community support
- Easy integration with Docusaurus frontend
- Familiar technology for development team

### Negative
- Statelessness may require additional consideration for user session management
- May need additional caching layers for performance at scale
- REST approach may require multiple requests for complex operations
- Less real-time capability compared to WebSocket-based solutions

## Alternatives Considered

### Alternative 1: GraphQL API
Use GraphQL instead of REST for more flexible data fetching
- Pros: More efficient data fetching, single endpoint for multiple use cases
- Cons: Additional complexity, learning curve for team, less suitable for simple educational use cases

### Alternative 2: Serverless Functions
Deploy API as serverless functions (AWS Lambda, Vercel functions, etc.)
- Pros: Better scalability, pay-per-use model, reduced infrastructure management
- Cons: Cold start issues, more complex deployment, potential for increased costs with high usage

### Alternative 3: Full Backend Framework
Use a more comprehensive backend framework like NestJS or Fastify
- Pros: Better structure for complex applications, enhanced features, TypeScript support
- Cons: Additional complexity, steeper learning curve, potential over-engineering for educational use case

## References
- `/specs/001-physical-ai-book/contracts/api-contracts.md` - Detailed API contracts
- `/specs/001-physical-ai-book/plan.md` - Technical Context and Primary Dependencies
- `/specs/001-physical-ai-book/data-model.md` - Data models that APIs will expose
- `physical-AI-book/src/api/index.js` - Actual implementation of the API