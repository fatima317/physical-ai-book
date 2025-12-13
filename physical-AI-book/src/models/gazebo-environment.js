/**
 * GazeboEnvironment Model
 * Represents Gazebo simulation environment configurations
 */

class GazeboEnvironment {
  /**
   * Create a GazeboEnvironment instance
   * @param {Object} data - Gazebo environment data
   * @param {string} data.id - Unique identifier
   * @param {string} data.name - Environment name
   * @param {string} data.description - Environment description
   * @param {string} data.version - Gazebo version (e.g., Garden)
   * @param {string} data.worldFile - Path to the world file
   * @param {Array<Object>} [data.robots] - Array of robot configurations in the environment
   * @param {Array<Object>} [data.objects] - Array of static objects in the environment
   * @param {Object} [data.physics] - Physics engine configuration
   * @param {Array<string>} [data.dependencies] - Required Gazebo packages/plugins
   * @param {string} [data.setupInstructions] - Environment setup instructions
   * @param {Array<Object>} [data.validationSteps] - Validation steps for the environment
   */
  constructor(data) {
    this.id = data.id;
    this.name = data.name;
    this.description = data.description;
    this.version = data.version || 'Garden'; // Default to Garden version
    this.worldFile = data.worldFile;
    this.robots = data.robots || [];
    this.objects = data.objects || [];
    this.physics = data.physics || {
      engine: 'ode',
      stepSize: 0.001,
      realTimeFactor: 1.0
    };
    this.dependencies = data.dependencies || [];
    this.setupInstructions = data.setupInstructions || '';
    this.validationSteps = data.validationSteps || [];
    this.createdAt = data.createdAt || new Date().toISOString();
    this.updatedAt = data.updatedAt || new Date().toISOString();
  }

  /**
   * Validates the GazeboEnvironment instance
   * @returns {Array<string>} Array of validation errors
   */
  validate() {
    const errors = [];

    if (!this.id) errors.push('ID is required');
    if (!this.name) errors.push('Name is required');
    if (!this.version) errors.push('Gazebo version is required');
    if (!this.worldFile) errors.push('World file path is required');

    // Validate version is a supported version
    const supportedVersions = ['classic', 'garden', 'harmonic', 'fortress'];
    if (!supportedVersions.includes(this.version.toLowerCase())) {
      errors.push(`Version must be one of: ${supportedVersions.join(', ')}`);
    }

    // Validate robots if provided
    if (this.robots) {
      this.robots.forEach((robot, index) => {
        if (!robot.name) {
          errors.push(`Robot at index ${index} must have a name`);
        }
        if (!robot.model) {
          errors.push(`Robot at index ${index} must have a model`);
        }
      });
    }

    // Validate objects if provided
    if (this.objects) {
      this.objects.forEach((obj, index) => {
        if (!obj.name) {
          errors.push(`Object at index ${index} must have a name`);
        }
      });
    }

    // Validate physics configuration
    if (this.physics) {
      const supportedEngines = ['ode', 'bullet', 'dart', 'simbody'];
      if (this.physics.engine && !supportedEngines.includes(this.physics.engine.toLowerCase())) {
        errors.push(`Physics engine must be one of: ${supportedEngines.join(', ')}`);
      }

      if (this.physics.stepSize && typeof this.physics.stepSize !== 'number') {
        errors.push('Physics step size must be a number');
      }

      if (this.physics.realTimeFactor && typeof this.physics.realTimeFactor !== 'number') {
        errors.push('Physics real time factor must be a number');
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
   * Adds a robot to the environment
   * @param {Object} robot - Robot configuration to add
   * @returns {boolean} True if robot was added successfully
   */
  addRobot(robot) {
    if (!robot || !robot.name || !robot.model) {
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
    if (!object || !object.name) {
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
    // In a real system, this would execute actual simulation validation
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
      worldFile: this.worldFile,
      robots: this.robots,
      objects: this.objects,
      physics: this.physics,
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
          dep.toLowerCase().includes('gazebo') ||
          dep.toLowerCase().includes('ignition')) {
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
        worldFiles: [this.worldFile],
        robotModels: this.robots.map(robot => robot.model),
        objectModels: this.objects.map(obj => obj.model || obj.name)
      },
      dependencies: this.dependencies,
      physicsEngine: this.physics.engine,
      createdAt: this.createdAt
    };
  }
}

export default GazeboEnvironment;