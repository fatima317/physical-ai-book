/**
 * SkillWorkflow Model
 * Represents a reusable process for content creation, formatting, and personalization
 */

class SkillWorkflow {
  /**
   * Create a SkillWorkflow instance
   * @param {Object} data - Skill workflow data
   * @param {string} data.id - Unique identifier
   * @param {string} data.name - Name (e.g., chapter_writing, code_formatting, personalization)
   * @param {string} data.description - What the skill does
   * @param {Array<Object>} [data.inputs] - Array of InputSpec objects
   * @param {Array<Object>} [data.outputs] - Array of OutputSpec objects
   * @param {string} [data.implementationPath] - Path to skill implementation
   */
  constructor(data) {
    this.id = data.id;
    this.name = data.name;
    this.description = data.description;
    this.inputs = data.inputs || [];
    this.outputs = data.outputs || [];
    this.implementationPath = data.implementationPath;
    this.createdAt = data.createdAt || new Date().toISOString();
    this.updatedAt = data.updatedAt || new Date().toISOString();
  }

  /**
   * Validates the SkillWorkflow instance
   * @returns {Array<string>} Array of validation errors
   */
  validate() {
    const errors = [];

    if (!this.id) errors.push('ID is required');
    if (!this.name) errors.push('Name is required');
    if (!this.description) errors.push('Description is required');

    const validNames = ['chapter_writing', 'code_formatting', 'personalization'];
    if (!validNames.includes(this.name)) {
      errors.push(`Name must be one of: ${validNames.join(', ')}`);
    }

    // If implementation path is provided, verify it exists in the expected location
    if (this.implementationPath) {
      // This would be checked against the actual file system in a real implementation
    }

    return errors;
  }

  /**
   * Executes the skill workflow with given parameters
   * @param {Object} parameters - Parameters for the skill execution
   * @param {string} context - Execution context
   * @returns {Object} Result of the skill execution
   */
  async execute(parameters, context) {
    try {
      // Validate inputs against the defined input specification
      const inputValidation = this._validateInputs(parameters);
      if (!inputValidation.valid) {
        throw new Error(`Invalid inputs: ${inputValidation.errors.join(', ')}`);
      }

      // Execute the skill-specific logic
      const result = await this._executeSkillLogic(parameters, context);

      // Validate outputs against the defined output specification
      const outputValidation = this._validateOutputs(result);
      if (!outputValidation.valid) {
        throw new Error(`Invalid outputs: ${outputValidation.errors.join(', ')}`);
      }

      return {
        success: true,
        result: result,
        status: 'success',
        messages: ['Skill executed successfully']
      };
    } catch (error) {
      return {
        success: false,
        result: null,
        status: 'error',
        messages: [error.message]
      };
    }
  }

  /**
   * Validates input parameters against the skill's input specification
   * @private
   */
  _validateInputs(parameters) {
    const errors = [];

    for (const inputSpec of this.inputs) {
      const inputValue = parameters[inputSpec.name];

      // Check if required input is provided
      if (inputSpec.required && inputValue === undefined) {
        errors.push(`Required input '${inputSpec.name}' is missing`);
        continue;
      }

      // Skip validation if input is not provided and not required
      if (inputValue === undefined) continue;

      // Validate type if specified
      if (inputSpec.type) {
        let isValidType = false;
        switch (inputSpec.type) {
          case 'string':
            isValidType = typeof inputValue === 'string';
            break;
          case 'number':
            isValidType = typeof inputValue === 'number';
            break;
          case 'boolean':
            isValidType = typeof inputValue === 'boolean';
            break;
          case 'object':
            isValidType = typeof inputValue === 'object' && inputValue !== null && !Array.isArray(inputValue);
            break;
          case 'array':
            isValidType = Array.isArray(inputValue);
            break;
          default:
            // For custom types, we'll assume it's valid for now
            isValidType = true;
        }

        if (!isValidType) {
          errors.push(`Input '${inputSpec.name}' should be of type '${inputSpec.type}'`);
        }
      }
    }

    return {
      valid: errors.length === 0,
      errors: errors
    };
  }

  /**
   * Validates output result against the skill's output specification
   * @private
   */
  _validateOutputs(result) {
    const errors = [];

    for (const outputSpec of this.outputs) {
      if (!result || result[outputSpec.name] === undefined) {
        if (outputSpec.required) {
          errors.push(`Required output '${outputSpec.name}' is missing`);
        }
        continue;
      }

      const outputValue = result[outputSpec.name];

      // Validate type if specified
      if (outputSpec.type) {
        let isValidType = false;
        switch (outputSpec.type) {
          case 'string':
            isValidType = typeof outputValue === 'string';
            break;
          case 'number':
            isValidType = typeof outputValue === 'number';
            break;
          case 'boolean':
            isValidType = typeof outputValue === 'boolean';
            break;
          case 'object':
            isValidType = typeof outputValue === 'object' && outputValue !== null && !Array.isArray(outputValue);
            break;
          case 'array':
            isValidType = Array.isArray(outputValue);
            break;
          default:
            // For custom types, we'll assume it's valid for now
            isValidType = true;
        }

        if (!isValidType) {
          errors.push(`Output '${outputSpec.name}' should be of type '${outputSpec.type}'`);
        }
      }
    }

    return {
      valid: errors.length === 0,
      errors: errors
    };
  }

  /**
   * Executes the skill-specific logic
   * @private
   */
  async _executeSkillLogic(parameters, context) {
    // This is a placeholder implementation - in a real system, this would
    // call the actual skill implementation based on the skill name
    switch (this.name) {
      case 'chapter_writing':
        return this._executeChapterWriting(parameters, context);
      case 'code_formatting':
        return this._executeCodeFormatting(parameters, context);
      case 'personalization':
        return this._executePersonalization(parameters, context);
      default:
        throw new Error(`Unknown skill: ${this.name}`);
    }
  }

  /**
   * Executes chapter writing logic
   * @private
   */
  _executeChapterWriting(parameters, context) {
    // Placeholder for chapter writing logic
    return {
      formattedContent: parameters.content || '',
      metadata: {
        wordCount: (parameters.content || '').split(' ').length,
        readingTime: Math.ceil(((parameters.content || '').split(' ').length) / 200) // 200 wpm
      }
    };
  }

  /**
   * Executes code formatting logic
   * @private
   */
  _executeCodeFormatting(parameters, context) {
    // Placeholder for code formatting logic
    return {
      formattedCode: parameters.code || '',
      language: parameters.language || 'unknown',
      formatted: true
    };
  }

  /**
   * Executes personalization logic
   * @private
   */
  _executePersonalization(parameters, context) {
    // Placeholder for personalization logic
    return {
      personalizedContent: parameters.content || '',
      userPreferences: parameters.userPreferences || {},
      adapted: true
    };
  }

  /**
   * Gets the skill's input specification
   * @returns {Array<Object>} Array of input specifications
   */
  getInputs() {
    return this.inputs;
  }

  /**
   * Gets the skill's output specification
   * @returns {Array<Object>} Array of output specifications
   */
  getOutputs() {
    return this.outputs;
  }

  /**
   * Checks if the skill requires a specific input
   * @param {string} inputName - Name of the input to check
   * @returns {boolean} True if the input is required
   */
  requiresInput(inputName) {
    return this.inputs.some(input => input.name === inputName && input.required);
  }

  /**
   * Checks if the skill produces a specific output
   * @param {string} outputName - Name of the output to check
   * @returns {boolean} True if the output is produced
   */
  producesOutput(outputName) {
    return this.outputs.some(output => output.name === outputName);
  }
}

export default SkillWorkflow;