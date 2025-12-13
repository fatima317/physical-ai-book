/**
 * UserProfile Model
 * Captures user preferences, skill level, language preference, and learning progress
 */

class UserProfile {
  /**
   * Create a UserProfile instance
   * @param {Object} data - User profile data
   * @param {string} data.id - Unique identifier
   * @param {string} [data.name] - User's name
   * @param {string} [data.skillLevel] - Skill level (Beginner, Intermediate, Advanced)
   * @param {string} [data.languagePreference] - Language preference (en, ur)
   * @param {Array<string>} [data.learningPath] - Array of LearningModule IDs
   * @param {Object} [data.progress] - Map of LearningModule ID to completion percentage
   * @param {Object} [data.preferences] - Map of personalization settings
   */
  constructor(data) {
    this.id = data.id;
    this.name = data.name || '';
    this.skillLevel = data.skillLevel || 'beginner'; // Default to beginner
    this.languagePreference = data.languagePreference || 'en'; // Default to English
    this.learningPath = data.learningPath || [];
    this.progress = data.progress || {};
    this.preferences = data.preferences || {};
    this.createdAt = data.createdAt || new Date().toISOString();
    this.updatedAt = data.updatedAt || new Date().toISOString();
  }

  /**
   * Validates the UserProfile instance
   * @returns {Array<string>} Array of validation errors
   */
  validate() {
    const errors = [];

    if (!this.id) errors.push('ID is required');
    if (!['beginner', 'intermediate', 'advanced'].includes(this.skillLevel.toLowerCase())) {
      errors.push('Skill level must be one of: beginner, intermediate, advanced');
    }
    if (!['en', 'ur'].includes(this.languagePreference.toLowerCase())) {
      errors.push('Language preference must be one of: en, ur');
    }

    // Validate progress values (should be between 0-100)
    if (this.progress) {
      Object.values(this.progress).forEach(value => {
        if (typeof value === 'number' && (value < 0 || value > 100)) {
          errors.push('Progress values must be between 0-100');
        }
      });
    }

    return errors;
  }

  /**
   * Updates user's skill level
   * @param {string} skillLevel - New skill level
   */
  updateSkillLevel(skillLevel) {
    if (['beginner', 'intermediate', 'advanced'].includes(skillLevel.toLowerCase())) {
      this.skillLevel = skillLevel.toLowerCase();
      this.updatedAt = new Date().toISOString();
    } else {
      throw new Error('Invalid skill level. Must be one of: beginner, intermediate, advanced');
    }
  }

  /**
   * Updates user's language preference
   * @param {string} language - New language preference
   */
  updateLanguagePreference(language) {
    if (['en', 'ur'].includes(language.toLowerCase())) {
      this.languagePreference = language.toLowerCase();
      this.updatedAt = new Date().toISOString();
    } else {
      throw new Error('Invalid language preference. Must be one of: en, ur');
    }
  }

  /**
   * Updates progress for a specific module
   * @param {string} moduleId - Module ID
   * @param {number} percentage - Completion percentage (0-100)
   */
  updateModuleProgress(moduleId, percentage) {
    if (typeof percentage === 'number' && percentage >= 0 && percentage <= 100) {
      this.progress[moduleId] = percentage;
      this.updatedAt = new Date().toISOString();
    } else {
      throw new Error('Progress percentage must be a number between 0-100');
    }
  }

  /**
   * Gets user's overall progress percentage
   * @returns {number} Overall progress percentage
   */
  getOverallProgress() {
    const progressValues = Object.values(this.progress);
    if (progressValues.length === 0) return 0;

    const totalProgress = progressValues.reduce((sum, value) => sum + value, 0);
    return Math.round(totalProgress / progressValues.length);
  }

  /**
   * Checks if user has completed a specific module
   * @param {string} moduleId - Module ID to check
   * @returns {boolean} True if module is completed (100%)
   */
  isModuleCompleted(moduleId) {
    return this.progress[moduleId] === 100;
  }

  /**
   * Gets modules in learning path that are not yet completed
   * @returns {Array<string>} Array of incomplete module IDs
   */
  getIncompleteModules() {
    return this.learningPath.filter(moduleId => this.progress[moduleId] < 100);
  }

  /**
   * Updates a personalization preference
   * @param {string} key - Preference key
   * @param {*} value - Preference value
   */
  updatePreference(key, value) {
    this.preferences[key] = value;
    this.updatedAt = new Date().toISOString();
  }

  /**
   * Gets a personalization preference
   * @param {string} key - Preference key
   * @param {*} defaultValue - Default value if preference not set
   * @returns {*} Preference value or default value
   */
  getPreference(key, defaultValue = null) {
    return this.preferences[key] !== undefined ? this.preferences[key] : defaultValue;
  }
}

export default UserProfile;