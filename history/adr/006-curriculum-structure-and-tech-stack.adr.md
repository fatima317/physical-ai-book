# ADR-006: Physical AI Curriculum Structure and Educational Technology Stack

## Status
Proposed

## Date
2025-12-30

## Context
The project requires developing a comprehensive Physical AI & Humanoid Robotics textbook with 13 weeks of curriculum covering ROS 2, Gazebo & Unity, NVIDIA Isaac, and Vision-Language-Action systems. The curriculum must balance theoretical foundations with practical implementation, support progressive learning from beginner to advanced levels, and integrate seamlessly with interactive tools for hands-on learning. The original specification required a modular approach with quarter overview, weekly breakdowns (1-13), learning outcomes, assessments, and hardware requirements.

## Decision
We will structure the curriculum using the following integrated educational architecture:

- **Curriculum Organization**: 13-week modular structure with progressive complexity
- **Module Clusters**: 4 core modules (ROS Fundamentals, Simulation, Perception, AI Planning) with quarter overview
- **Content Format**: Docusaurus MDX with integrated code examples, diagrams, and learning checkpoints
- **Educational Components**: Learning objectives, key terms, practical examples, and assessment checkpoints per module
- **Technology Stack**: ROS 2 Humble, Gazebo Garden, NVIDIA Isaac Sim, Python 3.11, with supporting libraries
- **Hardware Integration**: Jetson NX platform with specific sensor configurations as outlined in hardware requirements

This decision clusters the educational and technical architecture that works together to deliver the Physical AI learning experience.

## Consequences

### Positive
- Clear progression path from basic concepts to advanced implementation
- Modular design allows for targeted learning and easy updates
- Integration of theory with hands-on practice through code examples
- Consistent structure across all 13 weeks facilitates learning
- Hardware-optimized approach ensures practical applicability
- Support for multiple learning styles through varied content types

### Negative
- Complex initial setup required for comprehensive curriculum
- Maintenance overhead for keeping all 13 weeks updated with technology changes
- Potential for uneven difficulty progression across modules
- Dependency on specific hardware and software stack limits accessibility
- Extensive content requires significant initial development effort

## Alternatives Considered

### Alternative 1: Topic-Based Organization
Organize content by technology stack rather than progressive learning (e.g., all ROS content together, all AI content together)
- Pros: Better for reference material, easier for experienced users to find specific topics
- Cons: Doesn't support progressive learning, harder for beginners to follow, disconnect between theory and practice

### Alternative 2: Project-Based Learning
Focus on building a single large project across all 13 weeks with concepts introduced as needed
- Pros: More engaging, better integration of concepts, real-world approach
- Cons: Harder to assess learning outcomes, may miss foundational concepts, difficult to adapt to different skill levels

### Alternative 3: Standalone Modules
Create independent modules that can be taken in any order
- Pros: Flexible learning paths, easier to update individual modules, customizable curriculum
- Cons: Loses progressive learning benefits, potential gaps in foundational knowledge, harder to maintain conceptual coherence

## References
- `/specs/001-physical-ai-book/spec.md` - Original curriculum structure requirements
- `/specs/001-physical-ai-book/plan.md` - Technical implementation approach
- `/physical-AI-book/docs/quarter-overview/` - Module implementation
- `/physical-AI-book/docs/weekly-breakdown/` - Weekly curriculum implementation
- `/physical-AI-book/docs/hardware-reqs/` - Hardware requirements specification