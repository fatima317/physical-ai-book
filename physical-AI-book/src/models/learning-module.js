/**
 * LearningModule Model
 * Represents an educational unit containing lessons, exercises, and assessments
 */

class LearningModule {
  /**
   * Create a LearningModule instance
   * @param {Object} data - Module data
   * @param {string} data.id - Unique identifier
   * @param {string} data.title - Module title
   * @param {string} data.description - Brief description
   * @param {number} data.moduleNumber - Position in sequence (1-6)
   * @param {string} data.category - Category (Intro, Module, Capstone, Appendix)
   * @param {string} data.content - Module content in MDX format
   * @param {Array<string>} [data.prerequisites] - Array of LearningModule IDs
   * @param {Array<string>} [data.learningObjectives] - Learning objectives
   * @param {Array<Object>} [data.exercises] - Exercise objects
   * @param {Array<Object>} [data.assessments] - Assessment objects
   */
  constructor(data) {
    this.id = data.id;
    this.title = data.title;
    this.description = data.description;
    this.moduleNumber = data.moduleNumber;
    this.category = data.category;
    this.content = data.content;
    this.prerequisites = data.prerequisites || [];
    this.learningObjectives = data.learningObjectives || [];
    this.exercises = data.exercises || [];
    this.assessments = data.assessments || [];
    this.createdAt = data.createdAt || new Date().toISOString();
    this.updatedAt = data.updatedAt || new Date().toISOString();
  }

  /**
   * Validates the LearningModule instance
   * @returns {Array<string>} Array of validation errors
   */
  validate() {
    const errors = [];

    if (!this.id) errors.push('ID is required');
    if (!this.title) errors.push('Title is required');
    if (!this.description) errors.push('Description is required');
    if (typeof this.moduleNumber !== 'number' || this.moduleNumber < 1 || this.moduleNumber > 6) {
      errors.push('Module number must be between 1-6');
    }
    if (!['Intro', 'Module', 'Capstone', 'Appendix'].includes(this.category)) {
      errors.push('Category must be one of: Intro, Module, Capstone, Appendix');
    }
    if (!this.content) errors.push('Content is required');

    return errors;
  }

  /**
   * Checks if module has prerequisites
   * @returns {boolean} True if module has prerequisites
   */
  hasPrerequisites() {
    return this.prerequisites && this.prerequisites.length > 0;
  }

  /**
   * Gets module state based on user progress
   * @param {Object} userProgress - User's progress object
   * @returns {string} State (locked, available, in-progress, completed)
   */
  getState(userProgress) {
    if (!this.hasPrerequisites()) {
      return 'available';
    }

    const completedPrerequisites = this.prerequisites.filter(
      prereqId => userProgress[prereqId] >= 100
    );

    if (completedPrerequisites.length === this.prerequisites.length) {
      return userProgress[this.id] >= 100 ? 'completed' :
             userProgress[this.id] > 0 ? 'in-progress' : 'available';
    }

    return 'locked';
  }

  /**
   * Updates module content with personalization
   * @param {Object} personalizationData - User's personalization preferences
   * @returns {string} Personalized content
   */
  getPersonalizedContent(personalizationData) {
    let personalizedContent = this.content;

    // Apply skill level personalization
    if (personalizationData && personalizationData.skillLevel) {
      // This would implement the logic to show/hide content based on skill level
      // For example, showing more detailed explanations for beginners
      const skillLevel = personalizationData.skillLevel.toLowerCase();

      // Placeholder for skill-level-specific content modifications
      // In a real implementation, this would parse the MDX content
      // and show/hide sections based on skill level tags
    }

    return personalizedContent;
  }
}

export default LearningModule;