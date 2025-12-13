/**
 * HardwareConfiguration Model
 * Represents specifications for Jetson NX setup with sensors and actuators
 */

class HardwareConfiguration {
  /**
   * Create a HardwareConfiguration instance
   * @param {Object} data - Hardware configuration data
   * @param {string} data.id - Unique identifier
   * @param {string} data.name - Configuration name
   * @param {string} data.targetPlatform - Target platform (e.g., Jetson NX)
   * @param {Array<Object>} [data.sensors] - Array of Sensor objects
   * @param {Array<Object>} [data.actuators] - Array of Actuator objects
   * @param {Array<string>} [data.dependencies] - Required packages/libraries
   * @param {string} [data.setupInstructions] - Step-by-step setup guide
   * @param {Array<Object>} [data.validationSteps] - Array of ValidationStep objects
   */
  constructor(data) {
    this.id = data.id;
    this.name = data.name;
    this.targetPlatform = data.targetPlatform;
    this.sensors = data.sensors || [];
    this.actuators = data.actuators || [];
    this.dependencies = data.dependencies || [];
    this.setupInstructions = data.setupInstructions || '';
    this.validationSteps = data.validationSteps || [];
    this.createdAt = data.createdAt || new Date().toISOString();
    this.updatedAt = data.updatedAt || new Date().toISOString();
  }

  /**
   * Validates the HardwareConfiguration instance
   * @returns {Array<string>} Array of validation errors
   */
  validate() {
    const errors = [];

    if (!this.id) errors.push('ID is required');
    if (!this.name) errors.push('Name is required');
    if (!this.targetPlatform) errors.push('Target platform is required');

    // Validate target platform is Jetson NX as per requirements
    if (this.targetPlatform.toLowerCase() !== 'jetson nx') {
      errors.push('Target platform must be Jetson NX');
    }

    // Validate sensors and actuators if provided
    if (this.sensors) {
      this.sensors.forEach((sensor, index) => {
        if (!sensor.name) {
          errors.push(`Sensor at index ${index} must have a name`);
        }
      });
    }

    if (this.actuators) {
      this.actuators.forEach((actuator, index) => {
        if (!actuator.name) {
          errors.push(`Actuator at index ${index} must have a name`);
        }
      });
    }

    return errors;
  }

  /**
   * Gets all required dependencies for this configuration
   * @returns {Array<string>} Array of dependency names/packages
   */
  getDependencies() {
    return this.dependencies;
  }

  /**
   * Checks if a specific dependency is required
   * @param {string} dependency - Dependency name to check
   * @returns {boolean} True if dependency is required
   */
  requiresDependency(dependency) {
    return this.dependencies.includes(dependency);
  }

  /**
   * Gets setup instructions formatted for the target platform
   * @param {string} format - Format type ('markdown', 'html', 'text')
   * @returns {string} Formatted setup instructions
   */
  getSetupInstructions(format = 'markdown') {
    switch (format) {
      case 'html':
        // Convert markdown to simple HTML
        return this.setupInstructions
          .split('\n')
          .map(line => {
            if (line.startsWith('# ')) return `<h1>${line.substring(2)}</h1>`;
            if (line.startsWith('## ')) return `<h2>${line.substring(3)}</h2>`;
            if (line.startsWith('### ')) return `<h3>${line.substring(4)}</h3>`;
            if (line.startsWith('- ')) return `<li>${line.substring(2)}</li>`;
            if (line.trim() === '') return '<br>';
            return `<p>${line}</p>`;
          })
          .join('');
      case 'text':
        // Remove markdown formatting
        return this.setupInstructions
          .replace(/[#*`_]/g, '')
          .replace(/\[([^\]]+)\]\([^)]+\)/g, '$1'); // Remove links
      default:
        // Return as markdown
        return this.setupInstructions;
    }
  }

  /**
   * Gets list of all sensors in the configuration
   * @returns {Array<Object>} Array of sensor objects
   */
  getSensors() {
    return this.sensors;
  }

  /**
   * Gets list of all actuators in the configuration
   * @returns {Array<Object>} Array of actuator objects
   */
  getActuators() {
    return this.actuators;
  }

  /**
   * Checks if the configuration includes a specific sensor
   * @param {string} sensorName - Name of the sensor to check
   * @returns {boolean} True if sensor is included
   */
  hasSensor(sensorName) {
    return this.sensors.some(sensor => sensor.name === sensorName);
  }

  /**
   * Checks if the configuration includes a specific actuator
   * @param {string} actuatorName - Name of the actuator to check
   * @returns {boolean} True if actuator is included
   */
  hasActuator(actuatorName) {
    return this.actuators.some(actuator => actuator.name === actuatorName);
  }

  /**
   * Gets validation steps for this configuration
   * @returns {Array<Object>} Array of validation step objects
   */
  getValidationSteps() {
    return this.validationSteps;
  }

  /**
   * Executes validation steps for this configuration
   * @param {Object} context - Context for validation (e.g., hardware status)
   * @returns {Object} Validation results
   */
  async executeValidation(context = {}) {
    const results = {
      configurationId: this.id,
      platform: this.targetPlatform,
      totalSteps: this.validationSteps.length,
      passedSteps: 0,
      failedSteps: 0,
      results: []
    };

    for (const step of this.validationSteps) {
      try {
        const stepResult = await this._executeValidationStep(step, context);
        results.results.push(stepResult);

        if (stepResult.passed) {
          results.passedSteps++;
        } else {
          results.failedSteps++;
        }
      } catch (error) {
        results.results.push({
          step: step.description,
          passed: false,
          error: error.message,
          details: step
        });
        results.failedSteps++;
      }
    }

    results.success = results.passedSteps === results.totalSteps;

    return results;
  }

  /**
   * Executes a single validation step
   * @private
   */
  async _executeValidationStep(step, context) {
    // This is a placeholder implementation
    // In a real system, this would execute actual hardware validation
    return {
      step: step.description,
      command: step.command,
      passed: true, // Placeholder - would be determined by actual validation
      details: step,
      context: context
    };
  }

  /**
   * Generates a bill of materials for this configuration
   * @returns {Object} Bill of materials with sensors and actuators
   */
  getBillOfMaterials() {
    return {
      configurationId: this.id,
      name: this.name,
      targetPlatform: this.targetPlatform,
      sensors: this.sensors.map(sensor => ({
        name: sensor.name,
        type: sensor.type,
        purpose: sensor.purpose,
        quantity: sensor.quantity || 1
      })),
      actuators: this.actuators.map(actuator => ({
        name: actuator.name,
        type: actuator.type,
        purpose: actuator.purpose,
        quantity: actuator.quantity || 1
      })),
      dependencies: this.dependencies,
      createdAt: this.createdAt
    };
  }

  /**
   * Checks compatibility with ROS 2 Humble
   * @returns {Object} Compatibility report
   */
  checkROSCompatibility() {
    const compatibleDeps = [];
    const incompatibleDeps = [];

    this.dependencies.forEach(dep => {
      // In a real implementation, this would check against known ROS 2 Humble packages
      if (dep.toLowerCase().includes('ros-humble') ||
          dep.toLowerCase().includes('rclcpp') ||
          dep.toLowerCase().includes('rclpy')) {
        compatibleDeps.push(dep);
      } else {
        incompatibleDeps.push(dep);
      }
    });

    return {
      isCompatible: incompatibleDeps.length === 0,
      compatibleDependencies: compatibleDeps,
      incompatibleDependencies: incompatibleDeps,
      message: incompatibleDeps.length === 0
        ? 'Configuration is fully compatible with ROS 2 Humble'
        : `Configuration has ${incompatibleDeps.length} potentially incompatible dependencies`
    };
  }
}

export default HardwareConfiguration;