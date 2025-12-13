/**
 * InteractiveElement Model
 * Represents a dynamic content component that responds to user inputs
 */

class InteractiveElement {
  /**
   * Create an InteractiveElement instance
   * @param {Object} data - Interactive element data
   * @param {string} data.id - Unique identifier
   * @param {string} data.type - Type (personalizeChapter, translateToUrdu, other)
   * @param {string} data.moduleId - Reference to LearningModule
   * @param {Object} [data.configuration] - Configuration settings
   */
  constructor(data) {
    this.id = data.id;
    this.type = data.type;
    this.moduleId = data.moduleId;
    this.configuration = data.configuration || {};
    this.createdAt = data.createdAt || new Date().toISOString();
    this.updatedAt = data.updatedAt || new Date().toISOString();
  }

  /**
   * Validates the InteractiveElement instance
   * @returns {Array<string>} Array of validation errors
   */
  validate() {
    const errors = [];

    if (!this.id) errors.push('ID is required');
    if (!this.moduleId) errors.push('Module ID is required');

    const validTypes = ['personalizeChapter', 'translateToUrdu', 'other'];
    if (!validTypes.includes(this.type)) {
      errors.push(`Type must be one of: ${validTypes.join(', ')}`);
    }

    return errors;
  }

  /**
   * Renders the interactive element based on its type and configuration
   * @param {Object} context - Rendering context (user profile, preferences, etc.)
   * @returns {Object} Rendered element with props and content
   */
  render(context) {
    switch (this.type) {
      case 'personalizeChapter':
        return this._renderPersonalizeChapter(context);
      case 'translateToUrdu':
        return this._renderTranslateToUrdu(context);
      case 'other':
      default:
        return this._renderGenericElement(context);
    }
  }

  /**
   * Renders a personalization element
   * @private
   */
  _renderPersonalizeChapter(context) {
    const userProfile = context.userProfile || {};
    const skillLevel = userProfile.skillLevel || 'beginner';

    return {
      type: 'personalizeChapter',
      props: {
        defaultSkillLevel: skillLevel,
        configuration: this.configuration,
        moduleId: this.moduleId
      },
      component: 'PersonalizeChapter'
    };
  }

  /**
   * Renders a translation element
   * @private
   */
  _renderTranslateToUrdu(context) {
    const userProfile = context.userProfile || {};
    const languagePreference = userProfile.languagePreference || 'en';

    return {
      type: 'translateToUrdu',
      props: {
        languagePreference: languagePreference,
        configuration: this.configuration,
        moduleId: this.moduleId
      },
      component: 'TranslateToUrdu'
    };
  }

  /**
   * Renders a generic element
   * @private
   */
  _renderGenericElement(context) {
    return {
      type: this.type,
      props: {
        configuration: this.configuration,
        moduleId: this.moduleId,
        context: context
      },
      component: 'GenericInteractiveElement'
    };
  }

  /**
   * Processes user interaction with this element
   * @param {Object} interactionData - Data from user interaction
   * @param {Object} context - Context for processing
   * @returns {Object} Result of processing the interaction
   */
  processInteraction(interactionData, context) {
    switch (this.type) {
      case 'personalizeChapter':
        return this._processPersonalizeInteraction(interactionData, context);
      case 'translateToUrdu':
        return this._processTranslationInteraction(interactionData, context);
      case 'other':
      default:
        return this._processGenericInteraction(interactionData, context);
    }
  }

  /**
   * Processes personalization interaction
   * @private
   */
  _processPersonalizeInteraction(interactionData, context) {
    // Update user profile with new personalization preferences
    const newPreferences = {
      skillLevel: interactionData.skillLevel || context.userProfile?.skillLevel,
      ...interactionData.preferences
    };

    return {
      success: true,
      action: 'updatePersonalization',
      data: newPreferences,
      message: 'Personalization preferences updated'
    };
  }

  /**
   * Processes translation interaction
   * @private
   */
  _processTranslationInteraction(interactionData, context) {
    const newLanguage = interactionData.language || 'en';

    return {
      success: true,
      action: 'updateLanguage',
      data: { language: newLanguage },
      message: `Language preference updated to ${newLanguage}`
    };
  }

  /**
   * Processes generic interaction
   * @private
   */
  _processGenericInteraction(interactionData, context) {
    return {
      success: true,
      action: 'genericInteraction',
      data: interactionData,
      message: 'Interaction processed'
    };
  }

  /**
   * Updates the configuration of this interactive element
   * @param {Object} newConfiguration - New configuration settings
   */
  updateConfiguration(newConfiguration) {
    this.configuration = { ...this.configuration, ...newConfiguration };
    this.updatedAt = new Date().toISOString();
  }

  /**
   * Gets the element's configuration for a specific user
   * @param {Object} userProfile - User's profile
   * @returns {Object} User-specific configuration
   */
  getUserConfiguration(userProfile) {
    // Apply user-specific overrides to the default configuration
    const userConfig = { ...this.configuration };

    if (userProfile && userProfile.preferences) {
      // Apply user preferences to configuration if applicable
      Object.keys(userProfile.preferences).forEach(key => {
        if (this.configuration.hasOwnProperty(key)) {
          userConfig[key] = userProfile.preferences[key];
        }
      });
    }

    return userConfig;
  }
}

export default InteractiveElement;