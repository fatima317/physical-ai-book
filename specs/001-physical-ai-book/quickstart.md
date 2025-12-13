# Quickstart Guide: Physical AI & Humanoid Robotics Book

## Prerequisites

- **Hardware**: Jetson NX development kit
- **Software**:
  - ROS 2 Humble Hawksbill
  - Gazebo Garden
  - Isaac Sim (for advanced perception modules)
  - Node.js 18+ (for MDX processing)
  - Docker (for consistent environments)

## Setup

### 1. Clone and Initialize
```bash
git clone <repository-url>
cd physical-ai-book
npm install
```

### 2. Configure Subagents
```bash
# Subagents are located in .claude/commands/
# Each subagent has specific responsibilities:
# - ros_expert: ROS 2 code generation
# - gazebo_builder: Simulation creation
# - isaac_trainer: Isaac Sim perception & VSLAM
# - vla_planner: LLM-to-action pipelines (Whisper)
# - hardware_guide: Jetson & sensor setup
```

### 3. Configure Skills
```bash
# Skills are located in .claude/skills/
# Available skills:
# - chapter_writing: Content creation workflows
# - code_formatting: Code example formatting
# - personalization: Personalization workflows
```

### 4. Initialize Documentation
```bash
# Documentation is located in physical-AI-book/docs/
# Contains module guides, setup instructions, and API references
```

## Running the Interactive Book

### 1. Start the Development Server
```bash
npm run dev
# The book will be available at http://localhost:3000
```

### 2. Access Personalized Content
- Navigate to the book interface
- Set your skill level (Beginner/Intermediate/Advanced)
- Select your language preference (English/Urdu)
- Choose your learning path

### 3. Use Subagents for Learning
- Access the subagent interface from any module
- Use `ros_expert` for ROS 2 code generation
- Use `gazebo_builder` to create simulation environments
- Use `hardware_guide` for Jetson setup assistance

## Module Structure

The book contains 6 modules:
1. **Introduction**: Getting started with Physical AI
2. **Module 1**: ROS 2 Fundamentals (Jetson NX)
3. **Module 2**: Simulation Environments (Gazebo)
4. **Module 3**: Perception Systems (Isaac Sim)
5. **Module 4**: AI Planning & Control
6. **Capstone**: Complete Robot Project
7. **Appendix**: Reference Materials

## Quality Validation

Each module includes:
- Code examples tested on Jetson NX
- Simulation-to-hardware transfer verification
- Multilingual accuracy checks
- Interactive element functionality tests

## Running Tests

```bash
# Run all module tests
npm run test

# Run specific module tests
npm run test:module1

# Validate code examples on Jetson NX
npm run validate:hardware

# Check multilingual content accuracy
npm run validate:translation
```