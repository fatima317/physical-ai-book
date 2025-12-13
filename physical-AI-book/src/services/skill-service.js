/**
 * Skill Execution Service
 * Manages the execution of reusable skill workflows for content creation and formatting
 */

const fs = require('fs');
const path = require('path');
const StorageService = require('./storage-service');

class SkillService {
  constructor() {
    this.storageService = new StorageService();
    this.skillsPath = path.join(process.cwd(), '.claude', 'skills');
    this.availableSkills = new Map();
    this.executionHistory = new Map();
    this.maxExecutionTime = 30000; // 30 seconds max execution time
    this.maxConcurrentExecutions = 5;

    this.initializeSkills();
  }

  /**
   * Initializes available skills by scanning the skills directory
   */
  initializeSkills() {
    try {
      if (fs.existsSync(this.skillsPath)) {
        const files = fs.readdirSync(this.skillsPath);
        for (const file of files) {
          if (file.endsWith('.claude')) {
            const skillName = file.replace('.claude', '');
            const skillPath = path.join(this.skillsPath, file);
            const skillContent = fs.readFileSync(skillPath, 'utf8');

            // Extract skill description from the file
            const descriptionMatch = skillContent.match(/^# (.+)$/m);
            const description = descriptionMatch ? descriptionMatch[1] : 'No description available';

            this.availableSkills.set(skillName, {
              name: skillName,
              path: skillPath,
              description: description,
              content: skillContent,
              lastModified: fs.statSync(skillPath).mtime
            });
          }
        }
      }
    } catch (error) {
      console.error(`Error initializing skills: ${error.message}`);
    }
  }

  /**
   * Gets available skills
   * @returns {Array} List of available skills
   */
  getAvailableSkills() {
    return Array.from(this.availableSkills.values()).map(skill => ({
      name: skill.name,
      description: skill.description,
      lastModified: skill.lastModified
    }));
  }

  /**
   * Executes a skill with the given parameters and context
   * @param {string} skillName - Name of the skill to execute
   * @param {object} parameters - Parameters for the skill
   * @param {object} context - Context for the skill execution
   * @returns {Promise<object>} Execution result
   */
  async executeSkill(skillName, parameters = {}, context = {}) {
    return new Promise((resolve, reject) => {
      // Validate skill exists
      if (!this.availableSkills.has(skillName)) {
        return reject(new Error(`Skill '${skillName}' not found`));
      }

      const skill = this.availableSkills.get(skillName);
      const executionId = this.generateExecutionId();

      // Create execution record
      const executionRecord = {
        id: executionId,
        skill: skillName,
        parameters: parameters,
        context: context,
        timestamp: new Date().toISOString(),
        status: 'running',
        output: '',
        error: null,
        executionTime: 0
      };

      this.executionHistory.set(executionId, executionRecord);

      // Start timing
      const startTime = Date.now();

      // For now, we'll simulate the skill execution by returning its prompt
      // In a real implementation, this would interface with an LLM API
      try {
        // Simulate processing time
        setTimeout(() => {
          const endTime = Date.now();
          const executionTime = endTime - startTime;

          // For simulation purposes, return a response based on the skill's purpose
          let output = '';
          if (skillName === 'chapter_writing') {
            output = this.generateChapterWritingResponse(parameters, context);
          } else if (skillName === 'code_formatting') {
            output = this.generateCodeFormattingResponse(parameters, context);
          } else if (skillName === 'personalization') {
            output = this.generatePersonalizationResponse(parameters, context);
          } else {
            // Default response for unknown skills
            output = `Executed ${skillName} with parameters: ${JSON.stringify(parameters)}\n\nContext: ${JSON.stringify(context)}\n\n${skill.content.substring(0, 500)}...`;
          }

          executionRecord.output = output;
          executionRecord.status = 'completed';
          executionRecord.executionTime = executionTime;

          this.executionHistory.set(executionId, executionRecord);

          // Save to storage
          this.storageService.saveData(`skill-executions/${executionId}.json`, executionRecord);

          resolve({
            executionId,
            skill: skillName,
            parameters,
            context,
            output,
            executionTime,
            status: 'completed'
          });
        }, Math.min(1000, Object.keys(parameters).length * 100)); // Simulate processing time based on parameters
      } catch (error) {
        const endTime = Date.now();
        executionRecord.status = 'error';
        executionRecord.error = error.message;
        executionRecord.executionTime = endTime - startTime;

        this.executionHistory.set(executionId, executionRecord);
        reject(error);
      }
    });
  }

  /**
   * Generates a chapter writing specific response based on parameters and context
   * @param {object} parameters - Skill parameters
   * @param {object} context - Execution context
   * @returns {string} Generated response
   */
  generateChapterWritingResponse(parameters, context) {
    const { topic, targetAudience, skillLevel, sections } = parameters;

    return `# Chapter Writing Response

## Topic: ${topic || 'Unspecified Topic'}
## Target Audience: ${targetAudience || 'General'}
## Skill Level: ${skillLevel || 'Beginner'}

Based on your requirements, here's a structured chapter outline:

### Introduction
- Brief overview of ${topic || 'the topic'}
- Learning objectives
- Prerequisites for this chapter

### Main Content
${sections ? sections.map((section, index) => `1. ${section}`).join('\n') : '1. Core concepts\n2. Implementation details\n3. Examples and use cases\n4. Summary and next steps'}

### Code Examples
- Relevant code snippets for ${topic || 'the topic'}
- Best practices and common pitfalls
- Troubleshooting tips

### Exercises
- Practice problems related to ${topic || 'the topic'}
- Implementation challenges
- Extension activities

This chapter structure follows the DRY (Don't Repeat Yourself) principle and is tailored to the ${skillLevel || 'beginner'} skill level.
`;
  }

  /**
   * Generates a code formatting specific response based on parameters and context
   * @param {object} parameters - Skill parameters
   * @param {object} context - Execution context
   * @returns {string} Generated response
   */
  generateCodeFormattingResponse(parameters, context) {
    const { language, styleGuide, code } = parameters;

    return `# Code Formatting Response

## Language: ${language || 'Unspecified'}
## Style Guide: ${styleGuide || 'Default'}
## Input Code: ${code ? code.substring(0, 100) + '...' : 'Not provided'}

Formatted code following ${styleGuide || 'standard'} guidelines:

\`\`\`${language || 'text'}
// Formatted code would be generated here based on the input
// This would include proper indentation, naming conventions,
// comment standards, and language-specific best practices
${code || '// No code provided for formatting'}
\`\`\`

### Formatting Applied:
- Proper indentation (${styleGuide === 'python' ? '4 spaces' : '2 spaces or tabs'})
- Naming conventions (${styleGuide === 'python' ? 'snake_case' : 'camelCase'})
- Comment standards (${styleGuide === 'python' ? '# for comments' : '// for comments'})
- Line length limits (${styleGuide === 'pep8' ? '79 characters' : '80 characters'})
- Import organization (for Python)

The formatted code follows industry best practices for ${language || 'the specified language'}.
`;
  }

  /**
   * Generates a personalization specific response based on parameters and context
   * @param {object} parameters - Skill parameters
   * @param {object} context - Execution context
   * @returns {string} Generated response
   */
  generatePersonalizationResponse(parameters, context) {
    const { skillLevel, learningStyle, preferences } = parameters;
    const { userProfile, module } = context;

    return `# Personalization Response

## User Profile Analysis
- Skill Level: ${skillLevel || userProfile?.skillLevel || 'Unknown'}
- Learning Style: ${preferences?.learningStyle || 'Mixed'}
- Preferred Difficulty: ${preferences?.difficulty || 'Standard'}

## Personalized Content Adaptation

### Content Complexity Adjustment
- **Beginner Level**: Simplified explanations with more examples
- **Intermediate Level**: Balanced approach with practical applications
- **Advanced Level**: Concise explanations with advanced concepts

### Learning Path Customization
Based on the user's profile (${skillLevel || userProfile?.skillLevel || 'Unknown'}), the content will be adapted to include:

1. **Appropriate Examples**: ${skillLevel === 'beginner' ? 'Simple, step-by-step examples' : skillLevel === 'intermediate' ? 'Moderate complexity with real-world applications' : 'Advanced scenarios with optimization techniques'}

2. **Challenge Level**: ${skillLevel === 'beginner' ? 'Basic exercises with guided solutions' : skillLevel === 'intermediate' ? 'Moderate challenges with hints' : 'Complex problems requiring independent thinking'}

3. **Explanation Depth**: ${skillLevel === 'beginner' ? 'Detailed explanations with visual aids' : skillLevel === 'intermediate' ? 'Balanced explanations with some theory' : 'Concise explanations focusing on implementation'}

### Recommended Learning Sequence
- Start with fundamentals
- Progress to intermediate concepts
- Advance to complex applications
- Include review and practice sessions

This personalization ensures the content matches the learner's current capabilities and growth trajectory.
`;
  }

  /**
   * Gets execution history
   * @param {string} skillName - Optional skill name to filter
   * @param {number} limit - Optional limit on results
   * @returns {Array} Execution history
   */
  getExecutionHistory(skillName = null, limit = 50) {
    let history = Array.from(this.executionHistory.values());

    if (skillName) {
      history = history.filter(exec => exec.skill === skillName);
    }

    // Sort by timestamp (newest first)
    history.sort((a, b) => new Date(b.timestamp) - new Date(a.timestamp));

    return history.slice(0, limit);
  }

  /**
   * Gets execution result by ID
   * @param {string} executionId - Execution ID
   * @returns {object} Execution result
   */
  getExecutionResult(executionId) {
    return this.executionHistory.get(executionId) || null;
  }

  /**
   * Generates a unique execution ID
   * @returns {string} Execution ID
   */
  generateExecutionId() {
    return `skill_exec_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
  }

  /**
   * Gets statistics about skill usage
   * @returns {object} Usage statistics
   */
  getUsageStats() {
    const stats = {
      totalExecutions: this.executionHistory.size,
      skillUsage: {},
      recentExecutions: []
    };

    // Count skill usage
    for (const [id, execution] of this.executionHistory) {
      if (!stats.skillUsage[execution.skill]) {
        stats.skillUsage[execution.skill] = 0;
      }
      stats.skillUsage[execution.skill]++;
    }

    // Get recent executions
    const sortedExecutions = Array.from(this.executionHistory.values())
      .sort((a, b) => new Date(b.timestamp) - new Date(a.timestamp))
      .slice(0, 10);

    stats.recentExecutions = sortedExecutions;

    return stats;
  }

  /**
   * Clears execution history older than specified days
   * @param {number} days - Number of days to keep
   */
  clearOldHistory(days = 30) {
    const cutoffDate = new Date();
    cutoffDate.setDate(cutoffDate.getDate() - days);

    for (const [id, execution] of this.executionHistory) {
      if (new Date(execution.timestamp) < cutoffDate) {
        this.executionHistory.delete(id);
      }
    }
  }
}

module.exports = new SkillService();