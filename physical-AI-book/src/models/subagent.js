/**
 * Subagent Model
 * Represents a specialized AI assistant with specific domain expertise
 */

class Subagent {
  /**
   * Create a Subagent instance
   * @param {Object} data - Subagent data
   * @param {string} data.id - Unique identifier (e.g., ros_expert, gazebo_builder)
   * @param {string} data.name - Display name
   * @param {string} data.description - What the subagent does
   * @param {string} data.specialization - Specialization (ROS, Simulation, Perception, AI Planning, Hardware)
   * @param {Array<Object>} [data.commands] - Array of Command objects
   * @param {Array<string>} [data.capabilities] - Array of capabilities
   */
  constructor(data) {
    this.id = data.id;
    this.name = data.name;
    this.description = data.description;
    this.specialization = data.specialization;
    this.commands = data.commands || [];
    this.capabilities = data.capabilities || [];
    this.createdAt = data.createdAt || new Date().toISOString();
    this.updatedAt = data.updatedAt || new Date().toISOString();
  }

  /**
   * Validates the Subagent instance
   * @returns {Array<string>} Array of validation errors
   */
  validate() {
    const errors = [];

    if (!this.id) errors.push('ID is required');
    if (!this.name) errors.push('Name is required');
    if (!this.description) errors.push('Description is required');
    if (!this.specialization) errors.push('Specialization is required');

    const validSpecializations = ['ROS', 'Simulation', 'Perception', 'AI Planning', 'Hardware'];
    if (!validSpecializations.includes(this.specialization)) {
      errors.push(`Specialization must be one of: ${validSpecializations.join(', ')}`);
    }

    // Validate ID against allowed subagent names from spec
    const allowedIds = ['ros_expert', 'gazebo_builder', 'isaac_trainer', 'vla_planner', 'hardware_guide'];
    if (!allowedIds.includes(this.id)) {
      errors.push(`ID must be one of: ${allowedIds.join(', ')}`);
    }

    return errors;
  }

  /**
   * Executes a command with given parameters
   * @param {string} commandName - Name of the command to execute
   * @param {Object} parameters - Parameters for the command
   * @param {string} context - User's current context
   * @returns {Object} Result of the command execution
   */
  async executeCommand(commandName, parameters, context) {
    // Find the command in the subagent's command list
    const command = this.commands.find(cmd => cmd.name === commandName);

    if (!command) {
      throw new Error(`Command '${commandName}' not found in subagent '${this.id}'`);
    }

    try {
      // This is a placeholder implementation - in a real system, this would
      // call the actual command execution logic
      const result = await this._executeCommandLogic(command, parameters, context);

      return {
        success: true,
        result: result,
        explanation: command.explanation || 'Command executed successfully',
        validations: command.validations || [],
        nextSteps: command.nextSteps || []
      };
    } catch (error) {
      return {
        success: false,
        error: error.message,
        explanation: command.errorExplanation || 'Command execution failed'
      };
    }
  }

  /**
   * Internal method to execute command logic
   * @private
   */
  async _executeCommandLogic(command, parameters, context) {
    // Placeholder for actual command execution logic
    // This would be implemented differently for each subagent type
    return `Executed command: ${command.name} with parameters: ${JSON.stringify(parameters)}`;
  }

  /**
   * Checks if the subagent has a specific capability
   * @param {string} capability - Capability to check
   * @returns {boolean} True if subagent has the capability
   */
  hasCapability(capability) {
    return this.capabilities.includes(capability);
  }

  /**
   * Gets all available commands for this subagent
   * @returns {Array<Object>} Array of command objects
   */
  getAvailableCommands() {
    return this.commands;
  }

  /**
   * Adds a new command to the subagent
   * @param {Object} command - Command object to add
   */
  addCommand(command) {
    if (!this.commands.find(cmd => cmd.name === command.name)) {
      this.commands.push(command);
      this.updatedAt = new Date().toISOString();
    }
  }

  /**
   * Gets subagent-specific configuration based on context
   * @param {string} context - Context for configuration
   * @returns {Object} Configuration object
   */
  getConfiguration(context) {
    // Placeholder for context-specific configuration
    // Different subagents would return different configurations
    return {
      id: this.id,
      specialization: this.specialization,
      context: context,
      capabilities: this.capabilities
    };
  }
}

export default Subagent;