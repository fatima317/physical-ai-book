# ADR-004: Subagent Architecture Decision

## Status
Accepted

## Date
2025-12-11

## Context
The educational platform requires specialized AI subagents to provide targeted assistance to learners in different robotics domains. The original specification identified five specific subagents: ros_expert, gazebo_builder, isaac_trainer, vla_planner, and hardware_guide. These subagents need to provide personalized tutoring, code review, simulation assistance, and hardware setup guidance. The system must support modular, reusable AI assistance while maintaining educational quality and technical accuracy.

## Decision
We will implement the subagent architecture using:

- **Location**: Claude commands in `.claude/commands/` directory
- **Format**: `.claude` files with specific instructions and context for each subagent
- **Specialization**: Each subagent focuses on a specific domain:
  - ros_expert: ROS 2 code generation and best practices
  - gazebo_builder: Simulation creation and configuration
  - isaac_trainer: Isaac Sim perception and VSLAM
  - vla_planner: LLM-to-action pipelines with Whisper integration
  - hardware_guide: Jetson and sensor setup assistance
- **Execution**: Via API endpoint POST /api/subagents/{subagentId}/execute
- **Service Layer**: SubagentService in `physical-AI-book/src/services/subagent-service.js`

This approach leverages the Claude tool infrastructure for AI assistance while maintaining clear separation of concerns between different domains of expertise.

## Consequences

### Positive
- Clear separation of different robotics domains
- Reusable and modular AI assistance
- Integration with Claude tools ecosystem
- Scalable architecture for adding new subagents
- Domain-specific expertise without diluting general AI capability
- Consistent interface for different types of assistance

### Negative
- Potential redundancy in common AI capabilities
- Requires maintaining multiple specialized agents
- More complex orchestration and coordination between subagents
- Potential for inconsistent behavior across different subagents
- Increased initial setup and configuration overhead

## Alternatives Considered

### Alternative 1: Single General-Purpose AI Agent
Use a single AI agent with context switching for different domains
- Pros: Simpler architecture, shared learning across domains, reduced maintenance
- Cons: Less specialized expertise, potential confusion between domains, harder to maintain consistent quality across different types of assistance

### Alternative 2: Plugin-Based Architecture
Implement subagents as plugins with shared AI backend
- Pros: Shared common functionality, consistent behavior, easier to maintain common features
- Cons: More complex architecture, potential for plugin conflicts, harder to specialize individual agents

### Alternative 3: External AI Service Integration
Integrate with external AI services (OpenAI, Anthropic, etc.) directly
- Pros: Leverage mature AI platforms, reduced maintenance, better performance
- Cons: Higher costs, dependency on external services, less control over domain-specific tuning, potential privacy concerns for educational content

## References
- `/specs/001-physical-ai-book/plan.md` - Subagent specification in project structure
- `/specs/001-physical-ai-book/spec.md` - User stories for subagent-assisted learning
- `/specs/001-physical-ai-book/contracts/api-contracts.md` - Subagent API contracts
- `/specs/001-physical-ai-book/data-model.md` - Subagent entity specification
- `.claude/commands/` - Actual subagent implementations