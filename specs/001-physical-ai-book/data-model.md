# Data Model: Physical AI & Humanoid Robotics Book

## Entity: Learning Module
- **Attributes**:
  - id: string (unique identifier)
  - title: string (module title)
  - description: string (brief description)
  - moduleNumber: integer (position in sequence 1-6)
  - category: string (Intro, Module, Capstone, Appendix)
  - content: string (module content in MDX format)
  - prerequisites: array of LearningModule references
  - learningObjectives: array of string
  - exercises: array of Exercise objects
  - assessments: array of Assessment objects
  - createdAt: datetime
  - updatedAt: datetime

- **Validation rules**:
  - moduleNumber must be between 1-6
  - category must be one of: Intro, Module, Capstone, Appendix
  - title and description required

- **State transitions**: Draft → Review → Approved → Published

## Entity: User Profile
- **Attributes**:
  - id: string (unique identifier)
  - name: string (user's name)
  - skillLevel: enum (Beginner, Intermediate, Advanced)
  - languagePreference: enum (English, Urdu)
  - learningPath: array of LearningModule references
  - progress: map of LearningModule ID to completion percentage
  - preferences: map of personalization settings
  - createdAt: datetime
  - updatedAt: datetime

- **Validation rules**:
  - skillLevel must be one of the defined enum values
  - languagePreference must be English or Urdu
  - progress values must be between 0-100

## Entity: Subagent
- **Attributes**:
  - id: string (unique identifier, e.g., ros_expert, gazebo_builder)
  - name: string (display name)
  - description: string (what the subagent does)
  - specialization: enum (ROS, Simulation, Perception, AI Planning, Hardware)
  - commands: array of Command objects
  - capabilities: array of string
  - createdAt: datetime
  - updatedAt: datetime

- **Validation rules**:
  - specialization must be one of the defined enum values
  - id must match the allowed subagent names from spec

## Entity: Interactive Element
- **Attributes**:
  - id: string (unique identifier)
  - type: enum (personalizeChapter, translateToUrdu, other)
  - moduleId: string (reference to LearningModule)
  - configuration: map of settings
  - createdAt: datetime
  - updatedAt: datetime

- **Validation rules**:
  - type must be one of the defined enum values
  - moduleId must reference an existing LearningModule

## Entity: Skill Workflow
- **Attributes**:
  - id: string (unique identifier)
  - name: string (e.g., chapter_writing, code_formatting, personalization)
  - description: string (what the skill does)
  - inputs: array of InputSpec objects
  - outputs: array of OutputSpec objects
  - implementationPath: string (path to skill implementation)
  - createdAt: datetime
  - updatedAt: datetime

- **Validation rules**:
  - name must be one of: chapter_writing, code_formatting, personalization
  - implementationPath must exist in .claude/skills/

## Entity: Hardware Configuration
- **Attributes**:
  - id: string (unique identifier)
  - name: string (configuration name)
  - targetPlatform: string (e.g., Jetson NX)
  - sensors: array of Sensor objects
  - actuators: array of Actuator objects
  - dependencies: array of string (required packages/libraries)
  - setupInstructions: string (step-by-step setup guide)
  - validationSteps: array of ValidationStep objects
  - createdAt: datetime
  - updatedAt: datetime

- **Validation rules**:
  - targetPlatform must be Jetson NX
  - sensors and actuators must be valid hardware components