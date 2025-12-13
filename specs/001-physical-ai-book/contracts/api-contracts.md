# API Contracts: Physical AI & Humanoid Robotics Book

## Module Content API

### GET /api/modules/{moduleId}
**Purpose**: Retrieve a specific learning module with personalized content

**Request**:
- Path: `/api/modules/{moduleId}`
- Method: `GET`
- Headers:
  - `Accept-Language`: user's preferred language (en/ur)
  - `Authorization`: optional (for personalized content)

**Response**:
- Status: `200 OK`
- Body:
```json
{
  "id": "string",
  "title": "string",
  "content": "string (MDX format)",
  "moduleNumber": "integer",
  "category": "string (Intro|Module|Capstone|Appendix)",
  "learningObjectives": ["string"],
  "prerequisites": ["string (module IDs)"],
  "personalizedContent": "string (personalized MDX)",
  "availableLanguages": ["string (language codes)"],
  "userProgress": "integer (0-100)",
  "createdAt": "datetime",
  "updatedAt": "datetime"
}
```

## Subagent API

### POST /api/subagents/{subagentId}/execute
**Purpose**: Execute a subagent command with user input

**Request**:
- Path: `/api/subagents/{subagentId}`
- Method: `POST`
- Headers:
  - `Content-Type`: `application/json`
- Body:
```json
{
  "command": "string",
  "parameters": "object",
  "context": "string (user's current context)"
}
```

**Response**:
- Status: `200 OK`
- Body:
```json
{
  "result": "string (generated code, simulation, etc.)",
  "explanation": "string (educational explanation)",
  "validations": ["string (validation messages)"],
  "nextSteps": ["string (suggested next actions)"]
}
```

## User Profile API

### PUT /api/users/profile
**Purpose**: Update user profile with preferences and learning progress

**Request**:
- Path: `/api/users/profile`
- Method: `PUT`
- Headers:
  - `Content-Type`: `application/json`
- Body:
```json
{
  "skillLevel": "string (Beginner|Intermediate|Advanced)",
  "languagePreference": "string (en|ur)",
  "learningPath": ["string (module IDs)"],
  "progress": {
    "moduleId": "integer (0-100)"
  },
  "preferences": {
    "key": "value"
  }
}
```

**Response**:
- Status: `200 OK`
- Body:
```json
{
  "profile": "object (updated user profile)",
  "personalizedRecommendations": ["string (recommended modules)"]
}
```

## Skill Execution API

### POST /api/skills/{skillId}/execute
**Purpose**: Execute a specific skill with provided parameters

**Request**:
- Path: `/api/skills/{skillId}/execute`
- Method: `POST`
- Headers:
  - `Content-Type`: `application/json`
- Body:
```json
{
  "parameters": "object",
  "context": "string (execution context)"
}
```

**Response**:
- Status: `200 OK`
- Body:
```json
{
  "result": "object (skill execution result)",
  "status": "string (success|error)",
  "messages": ["string (execution messages)"]
}
```