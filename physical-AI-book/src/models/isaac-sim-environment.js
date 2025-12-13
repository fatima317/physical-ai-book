/**
 * IsaacSimEnvironment Model
 * Represents Isaac Sim simulation environment configurations
 */

class IsaacSimEnvironment {
  /**
   * Create an IsaacSimEnvironment instance
   * @param {Object} data - Isaac Sim environment data
   * @param {string} data.id - Unique identifier
   * @param {string} data.name - Environment name
   * @param {string} data.description - Environment description
   * @param {string} data.version - Isaac Sim version (e.g., 2023.1.1)
   * @param {string} data.stagePath - Path to the USD stage file
   * @param {Array<Object>} [data.robots] - Array of robot configurations in the environment
   * @param {Array<Object>} [data.objects] - Array of static objects in the environment
   * @param {Object} [data.physics] - Physics engine configuration
   * @param {Array<string>} [data.dependencies] - Required Isaac Sim extensions/packages
   * @param {string} [data.setupInstructions] - Environment setup instructions
   * @param {Array<Object>} [data.validationSteps] - Validation steps for the environment
   * @param {Object} [data.rendering] - Rendering configuration
   * @param {Object} [data.sensors] - Sensor configurations available in the environment
   */
  constructor(data) {
    this.id = data.id;
    this.name = data.name;
    this.description = data.description;
    this.version = data.version || '2023.1.1'; // Default to a recent version
    this.stagePath = data.stagePath;
    this.robots = data.robots || [];
    this.objects = data.objects || [];
    this.physics = data.physics || {
      engine: 'PhysX',
      stepSize: 0.008333, // 120 Hz default
      solverType: 'TGS'
    };
    this.dependencies = data.dependencies || [];
    this.setupInstructions = data.setupInstructions || '';
    this.validationSteps = data.validationSteps || [];
    this.rendering = data.rendering || {
      resolution: { width: 1920, height: 1080 },
      fps: 60,
      renderer: 'Hydra'
    };
    this.sensors = data.sensors || [];
    this.createdAt = data.createdAt || new Date().toISOString();
    this.updatedAt = data.updatedAt || new Date().toISOString();
  }

  /**
   * Validates the IsaacSimEnvironment instance
   * @returns {Array<string>} Array of validation errors
   */
  validate() {
    const errors = [];

    if (!this.id) errors.push('ID is required');
    if (!this.name) errors.push('Name is required');
    if (!this.version) errors.push('Isaac Sim version is required');
    if (!this.stagePath) errors.push('Stage path is required');

    // Validate version format (should be in format like "2023.1.1")
    const versionRegex = /^\d{4}\.\d+(\.\d+)?$/;
    if (this.version && !versionRegex.test(this.version)) {
      errors.push('Version must follow format YYYY.M[.m] (e.g., 2023.1.1)');
    }

    // Validate robots if provided
    if (this.robots) {
      this.robots.forEach((robot, index) => {
        if (!robot.name) {
          errors.push(`Robot at index ${index} must have a name`);
        }
        if (!robot.usdPath) {
          errors.push(`Robot at index ${index} must have a USD path`);
        }
      });
    }

    // Validate objects if provided
    if (this.objects) {
      this.objects.forEach((obj, index) => {
        if (!obj.name) {
          errors.push(`Object at index ${index} must have a name`);
        }
        if (!obj.usdPath) {
          errors.push(`Object at index ${index} must have a USD path`);
        }
      });
    }

    // Validate physics configuration
    if (this.physics) {
      const supportedEngines = ['PhysX', 'Builtin'];
      if (this.physics.engine && !supportedEngines.includes(this.physics.engine)) {
        errors.push(`Physics engine must be one of: ${supportedEngines.join(', ')}`);
      }

      if (this.physics.stepSize && typeof this.physics.stepSize !== 'number') {
        errors.push('Physics step size must be a number');
      }

      if (this.physics.solverType && !['TGS', 'PGS'].includes(this.physics.solverType)) {
        errors.push('Physics solver type must be TGS or PGS');
      }
    }

    // Validate rendering configuration
    if (this.rendering) {
      if (this.rendering.resolution) {
        if (!this.rendering.resolution.width || !this.rendering.resolution.height) {
          errors.push('Rendering resolution must include both width and height');
        }
      }

      if (this.rendering.fps && typeof this.rendering.fps !== 'number') {
        errors.push('Rendering FPS must be a number');
      }
    }

    return errors;
  }

  /**
   * Gets all required dependencies for this environment
   * @returns {Array<string>} Array of dependency names/packages
   */
  getDependencies() {
    return [...this.dependencies];
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
   * Gets setup instructions formatted for the environment
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
   * Gets list of all robots in the environment
   * @returns {Array<Object>} Array of robot objects
   */
  getRobots() {
    return [...this.robots];
  }

  /**
   * Gets list of all objects in the environment
   * @returns {Array<Object>} Array of object objects
   */
  getObjects() {
    return [...this.objects];
  }

  /**
   * Gets list of all sensors in the environment
   * @returns {Array<Object>} Array of sensor objects
   */
  getSensors() {
    return [...this.sensors];
  }

  /**
   * Adds a robot to the environment
   * @param {Object} robot - Robot configuration to add
   * @returns {boolean} True if robot was added successfully
   */
  addRobot(robot) {
    if (!robot || !robot.name || !robot.usdPath) {
      return false;
    }

    // Check if robot with same name already exists
    if (this.robots.some(r => r.name === robot.name)) {
      return false;
    }

    this.robots.push(robot);
    this.updatedAt = new Date().toISOString();
    return true;
  }

  /**
   * Adds an object to the environment
   * @param {Object} object - Object configuration to add
   * @returns {boolean} True if object was added successfully
   */
  addObject(object) {
    if (!object || !object.name || !object.usdPath) {
      return false;
    }

    // Check if object with same name already exists
    if (this.objects.some(o => o.name === object.name)) {
      return false;
    }

    this.objects.push(object);
    this.updatedAt = new Date().toISOString();
    return true;
  }

  /**
   * Adds a sensor to the environment
   * @param {Object} sensor - Sensor configuration to add
   * @returns {boolean} True if sensor was added successfully
   */
  addSensor(sensor) {
    if (!sensor || !sensor.name || !sensor.type) {
      return false;
    }

    // Check if sensor with same name already exists
    if (this.sensors.some(s => s.name === sensor.name)) {
      return false;
    }

    this.sensors.push(sensor);
    this.updatedAt = new Date().toISOString();
    return true;
  }

  /**
   * Checks if the environment includes a specific robot
   * @param {string} robotName - Name of the robot to check
   * @returns {boolean} True if robot is included
   */
  hasRobot(robotName) {
    return this.robots.some(robot => robot.name === robotName);
  }

  /**
   * Checks if the environment includes a specific object
   * @param {string} objectName - Name of the object to check
   * @returns {boolean} True if object is included
   */
  hasObject(objectName) {
    return this.objects.some(obj => obj.name === objectName);
  }

  /**
   * Checks if the environment includes a specific sensor
   * @param {string} sensorName - Name of the sensor to check
   * @returns {boolean} True if sensor is included
   */
  hasSensor(sensorName) {
    return this.sensors.some(sensor => sensor.name === sensorName);
  }

  /**
   * Gets validation steps for this environment
   * @returns {Array<Object>} Array of validation step objects
   */
  getValidationSteps() {
    return [...this.validationSteps];
  }

  /**
   * Executes validation steps for this environment
   * @param {Object} context - Context for validation (e.g., simulation status)
   * @returns {Object} Validation results
   */
  async executeValidation(context = {}) {
    const results = {
      environmentId: this.id,
      name: this.name,
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
    // In a real system, this would execute actual Isaac Sim validation
    return {
      step: step.description,
      command: step.command,
      passed: true, // Placeholder - would be determined by actual validation
      details: step,
      context: context
    };
  }

  /**
   * Gets the complete configuration for launching this environment
   * @returns {Object} Complete environment configuration
   */
  getLaunchConfiguration() {
    return {
      id: this.id,
      name: this.name,
      version: this.version,
      stagePath: this.stagePath,
      robots: this.robots,
      objects: this.objects,
      physics: this.physics,
      rendering: this.rendering,
      sensors: this.sensors,
      dependencies: this.dependencies,
      setupInstructions: this.setupInstructions
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
          dep.toLowerCase().includes('isaac') ||
          dep.toLowerCase().includes('omni') ||
          dep.toLowerCase().includes('usd')) {
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
        ? 'Environment is fully compatible with ROS 2 Humble'
        : `Environment has ${incompatibleDeps.length} potentially incompatible dependencies`
    };
  }

  /**
   * Gets the bill of materials for this simulation environment
   * @returns {Object} Bill of materials with required assets
   */
  getBillOfMaterials() {
    return {
      environmentId: this.id,
      name: this.name,
      version: this.version,
      requiredAssets: {
        stageFile: this.stagePath,
        robotModels: this.robots.map(robot => robot.usdPath),
        objectModels: this.objects.map(obj => obj.usdPath),
        sensors: this.sensors.map(sensor => sensor.type)
      },
      dependencies: this.dependencies,
      physicsEngine: this.physics.engine,
      renderingEngine: this.rendering.renderer,
      createdAt: this.createdAt
    };
  }

  /**
   * Gets perception capabilities of this environment
   * @returns {Array<string>} List of supported perception capabilities
   */
  getPerceptionCapabilities() {
    const capabilities = [];

    // Check for different sensor types in the environment
    if (this.sensors.some(sensor => sensor.type.toLowerCase().includes('camera'))) {
      capabilities.push('Computer Vision');
    }

    if (this.sensors.some(sensor => sensor.type.toLowerCase().includes('lidar') ||
                              sensor.type.toLowerCase().includes('ray'))) {
      capabilities.push('3D Perception');
    }

    if (this.sensors.some(sensor => sensor.type.toLowerCase().includes('depth'))) {
      capabilities.push('Depth Sensing');
    }

    if (this.sensors.some(sensor => sensor.type.toLowerCase().includes('imu'))) {
      capabilities.push('Inertial Measurement');
    }

    return [...new Set(capabilities)]; // Remove duplicates
  }
}

export default IsaacSimEnvironment;