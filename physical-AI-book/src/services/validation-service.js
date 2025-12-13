/**
 * ValidationService
 * Handles content and hardware configuration validation
 */

import HardwareConfiguration from '../models/hardware-configuration.js';

class ValidationService {
  constructor() {
    // In a real implementation, this would connect to various validation tools
    // For now, we'll implement basic validation logic
    this.validationRules = {
      content: {
        minimumWordCount: 100,
        maximumWordCount: 10000,
        requiredSections: ['introduction', 'content', 'conclusion'],
        prohibitedContent: ['TODO', 'FIXME', 'PLACEHOLDER']
      },
      code: {
        requiredElements: ['imports', 'function_definition', 'error_handling'],
        prohibitedPatterns: ['hardcoded_values', 'secrets_in_code']
      },
      hardware: {
        requiredComponents: ['processor', 'memory', 'storage'],
        validationTimeout: 30000 // 30 seconds
      }
    };
  }

  /**
   * Validates a learning module's content
   * @param {Object} module - The module to validate
   * @returns {Object} Validation results
   */
  async validateModuleContent(module) {
    if (!module) {
      return {
        valid: false,
        errors: ['Module is undefined or null'],
        warnings: [],
        details: {}
      };
    }

    const errors = [];
    const warnings = [];
    const details = {};

    // Validate required fields
    if (!module.id) {
      errors.push('Module ID is required');
    }

    if (!module.title || module.title.trim() === '') {
      errors.push('Module title is required');
    }

    if (!module.content || module.content.trim() === '') {
      errors.push('Module content is required');
    }

    // Validate content length
    if (module.content) {
      const wordCount = module.content.split(/\s+/).filter(word => word.length > 0).length;
      details.wordCount = wordCount;

      if (wordCount < this.validationRules.content.minimumWordCount) {
        errors.push(`Content is too short. Minimum ${this.validationRules.content.minimumWordCount} words required, got ${wordCount}`);
      }

      if (wordCount > this.validationRules.content.maximumWordCount) {
        errors.push(`Content is too long. Maximum ${this.validationRules.content.maximumWordCount} words allowed, got ${wordCount}`);
      }
    }

    // Check for prohibited content
    for (const prohibited of this.validationRules.content.prohibitedContent) {
      if (module.content && module.content.toLowerCase().includes(prohibited.toLowerCase())) {
        warnings.push(`Prohibited content found: ${prohibited}`);
      }
    }

    // Validate learning objectives
    if (module.learningObjectives && Array.isArray(module.learningObjectives)) {
      if (module.learningObjectives.length === 0) {
        warnings.push('No learning objectives defined');
      } else {
        for (let i = 0; i < module.learningObjectives.length; i++) {
          if (!module.learningObjectives[i] || module.learningObjectives[i].trim() === '') {
            errors.push(`Learning objective at index ${i} is empty`);
          }
        }
      }
    }

    // Validate interactive elements
    if (module.interactiveElements && Array.isArray(module.interactiveElements)) {
      for (let i = 0; i < module.interactiveElements.length; i++) {
        const element = module.interactiveElements[i];
        if (!element.type) {
          errors.push(`Interactive element at index ${i} is missing type`);
        }
        if (!element.title) {
          errors.push(`Interactive element at index ${i} is missing title`);
        }
      }
    }

    // Validate code examples
    if (module.codeExamples && Array.isArray(module.codeExamples)) {
      for (let i = 0; i < module.codeExamples.length; i++) {
        const codeExample = module.codeExamples[i];
        const codeValidation = await this.validateCodeExample(codeExample);
        if (!codeValidation.valid) {
          errors.push(`Code example at index ${i}: ${codeValidation.errors.join(', ')}`);
        }
      }
    }

    // Validate references/citations
    if (module.references && Array.isArray(module.references)) {
      for (let i = 0; i < module.references.length; i++) {
        const reference = module.references[i];
        if (!reference.title || !reference.url) {
          warnings.push(`Reference at index ${i} is missing title or URL`);
        }
      }
    }

    return {
      valid: errors.length === 0,
      errors,
      warnings,
      details
    };
  }

  /**
   * Validates a code example
   * @param {Object} codeExample - The code example to validate
   * @returns {Object} Validation results
   */
  async validateCodeExample(codeExample) {
    if (!codeExample) {
      return {
        valid: false,
        errors: ['Code example is undefined or null'],
        warnings: [],
        details: {}
      };
    }

    const errors = [];
    const warnings = [];
    const details = {};

    if (!codeExample.code || codeExample.code.trim() === '') {
      errors.push('Code example is empty');
    }

    if (!codeExample.language) {
      warnings.push('Language not specified for code example');
    }

    if (!codeExample.description) {
      warnings.push('Description not provided for code example');
    }

    // Check for common issues in the code
    if (codeExample.code) {
      // Check for TODOs or FIXMEs
      if (codeExample.code.toLowerCase().includes('todo')) {
        warnings.push('TODO found in code example');
      }

      if (codeExample.code.toLowerCase().includes('fixme')) {
        warnings.push('FIXME found in code example');
      }

      // Check for hardcoded values
      if (/\b\d{4,}\b/.test(codeExample.code)) { // Check for numbers with 4+ digits
        warnings.push('Potential hardcoded value found in code example');
      }

      // Check for potential secrets
      const secretPatterns = [
        /password\s*[:=]/i,
        /secret\s*[:=]/i,
        /token\s*[:=]/i,
        /key\s*[:=]/i,
        /api_key\s*[:=]/i
      ];

      for (const pattern of secretPatterns) {
        if (pattern.test(codeExample.code)) {
          errors.push('Potential secret found in code example');
          break;
        }
      }
    }

    // Check if the code example is executable (syntax validation would go here)
    // For now, we'll just check if it has some basic structure
    if (codeExample.code) {
      details.lineCount = codeExample.code.split('\n').length;
      details.hasImports = /import|from|require/.test(codeExample.code);
      details.hasFunctions = /function|def|class/.test(codeExample.code);
    }

    return {
      valid: errors.length === 0,
      errors,
      warnings,
      details
    };
  }

  /**
   * Validates hardware configuration
   * @param {string} configurationId - Configuration ID
   * @param {Object} hardwareSpec - Hardware specification to validate
   * @returns {Object} Validation results
   */
  async validateHardwareConfiguration(configurationId, hardwareSpec) {
    if (!configurationId) {
      return {
        valid: false,
        errors: ['Configuration ID is required'],
        warnings: [],
        details: {}
      };
    }

    if (!hardwareSpec) {
      return {
        valid: false,
        errors: ['Hardware specification is required'],
        warnings: [],
        details: {}
      };
    }

    const errors = [];
    const warnings = [];
    const details = {};

    // Create a HardwareConfiguration instance to validate
    try {
      const config = new HardwareConfiguration({
        id: configurationId,
        ...hardwareSpec
      });

      const validationErrors = config.validate();
      if (validationErrors.length > 0) {
        errors.push(...validationErrors);
      }

      // Validate required components
      if (!hardwareSpec.sensors || hardwareSpec.sensors.length === 0) {
        warnings.push('No sensors specified in hardware configuration');
      }

      if (!hardwareSpec.actuators || hardwareSpec.actuators.length === 0) {
        warnings.push('No actuators specified in hardware configuration');
      }

      if (!hardwareSpec.dependencies || hardwareSpec.dependencies.length === 0) {
        warnings.push('No dependencies specified in hardware configuration');
      }

      // Validate ROS compatibility
      const rosCompatibility = config.checkROSCompatibility();
      details.rosCompatibility = rosCompatibility;

      if (!rosCompatibility.isCompatible) {
        warnings.push(rosCompatibility.message);
      }

      // Validate specific hardware requirements for Jetson NX
      if (hardwareSpec.targetPlatform && hardwareSpec.targetPlatform.toLowerCase() === 'jetson nx') {
        // Check for common Jetson NX requirements
        if (!config.requiresDependency('nvidia-jetpack')) {
          warnings.push('nvidia-jetpack dependency not specified for Jetson NX');
        }
      }

      details.billOfMaterials = config.getBillOfMaterials();
      details.validationSteps = config.getValidationSteps();

    } catch (error) {
      errors.push(`Error validating hardware configuration: ${error.message}`);
    }

    return {
      valid: errors.length === 0,
      errors,
      warnings,
      details
    };
  }

  /**
   * Validates multiple modules
   * @param {Array<Object>} modules - Array of modules to validate
   * @returns {Object} Overall validation results
   */
  async validateModules(modules) {
    if (!Array.isArray(modules)) {
      return {
        valid: false,
        errors: ['Input must be an array of modules'],
        warnings: [],
        details: { modulesValidated: 0 }
      };
    }

    const results = {
      valid: true,
      errors: [],
      warnings: [],
      details: {
        modulesValidated: modules.length,
        moduleResults: [],
        summary: {
          total: modules.length,
          valid: 0,
          invalid: 0,
          warnings: 0
        }
      }
    };

    for (let i = 0; i < modules.length; i++) {
      const module = modules[i];
      const moduleResult = await this.validateModuleContent(module);

      results.moduleResults.push({
        moduleId: module.id || `module-${i}`,
        ...moduleResult
      });

      if (!moduleResult.valid) {
        results.valid = false;
        results.summary.invalid++;
      } else {
        results.summary.valid++;
      }

      if (moduleResult.warnings.length > 0) {
        results.summary.warnings++;
      }

      results.errors.push(...moduleResult.errors.map(error =>
        `Module ${module.id || i}: ${error}`
      ));

      results.warnings.push(...moduleResult.warnings.map(warning =>
        `Module ${module.id || i}: ${warning}`
      ));
    }

    return results;
  }

  /**
   * Validates module prerequisites
   * @param {Object} module - The module to validate prerequisites for
   * @param {Array<Object>} allModules - All available modules
   * @returns {Object} Validation results
   */
  async validatePrerequisites(module, allModules) {
    if (!module || !Array.isArray(allModules)) {
      return {
        valid: false,
        errors: ['Module and allModules array are required'],
        warnings: [],
        details: {}
      };
    }

    const errors = [];
    const warnings = [];
    const details = {
      missingPrerequisites: [],
      invalidPrerequisites: []
    };

    if (!module.prerequisites || !Array.isArray(module.prerequisites)) {
      return {
        valid: true,
        errors: [],
        warnings: [],
        details: { ...details, prerequisiteCount: 0 }
      };
    }

    details.prerequisiteCount = module.prerequisites.length;

    for (const prereqId of module.prerequisites) {
      const prereqModule = allModules.find(m => m.id === prereqId);
      if (!prereqModule) {
        errors.push(`Prerequisite module with ID ${prereqId} does not exist`);
        details.missingPrerequisites.push(prereqId);
      }
    }

    return {
      valid: errors.length === 0,
      errors,
      warnings,
      details
    };
  }

  /**
   * Validates module's technical accuracy based on source verification
   * @param {Object} module - The module to validate
   * @returns {Object} Validation results for technical accuracy
   */
  async validateTechnicalAccuracy(module) {
    if (!module) {
      return {
        valid: false,
        errors: ['Module is required'],
        warnings: [],
        details: {}
      };
    }

    const errors = [];
    const warnings = [];
    const details = {
      verifiedSources: 0,
      unverifiedSources: 0,
      citationQuality: 0
    };

    // Check for proper citations and references
    if (module.content) {
      // Look for citation patterns like [1], [2], etc. or (Author, Year)
      const citationPattern = /\[\d+\]|\([A-Za-z]+, \d{4}\)/g;
      const citations = module.content.match(citationPattern) || [];

      if (citations.length === 0) {
        warnings.push('No citations found in module content');
      } else {
        details.citationCount = citations.length;
      }
    }

    // Validate references section
    if (module.references && Array.isArray(module.references) && module.references.length > 0) {
      for (let i = 0; i < module.references.length; i++) {
        const reference = module.references[i];

        if (!reference.title) {
          errors.push(`Reference at index ${i} is missing title`);
        }

        if (!reference.url && !reference.doi) {
          warnings.push(`Reference at index ${i} is missing URL or DOI`);
        }

        if (reference.url) {
          try {
            // Basic URL validation
            new URL(reference.url);
            details.verifiedSources++;
          } catch {
            errors.push(`Invalid URL in reference at index ${i}: ${reference.url}`);
            details.unverifiedSources++;
          }
        }
      }
    } else {
      warnings.push('No references section found');
    }

    // Check for technical claims that should be cited
    const technicalTerms = [
      'ROS 2 Humble', 'Gazebo Garden', 'Isaac Sim', 'Jetson NX',
      'VSLAM', 'Whisper', 'LLM-to-action', 'perception pipeline'
    ];

    for (const term of technicalTerms) {
      if (module.content && module.content.includes(term) && !module.content.includes('[') && !module.content.includes(')') && module.references && module.references.length === 0) {
        warnings.push(`Technical term "${term}" found without citation`);
      }
    }

    details.citationQuality = details.verifiedSources / (details.verifiedSources + details.unverifiedSources || 1) * 100;

    return {
      valid: errors.length === 0,
      errors,
      warnings,
      details
    };
  }

  /**
   * Performs comprehensive validation of a module
   * @param {Object} module - The module to validate
   * @param {Array<Object>} allModules - All available modules (for prerequisites)
   * @returns {Object} Comprehensive validation results
   */
  async validateModuleComprehensive(module, allModules = []) {
    if (!module) {
      return {
        valid: false,
        errors: ['Module is required'],
        warnings: [],
        details: {}
      };
    }

    // Run all validations
    const contentValidation = await this.validateModuleContent(module);
    const prerequisiteValidation = await this.validatePrerequisites(module, allModules);
    const technicalValidation = await this.validateTechnicalAccuracy(module);

    // Combine results
    const allErrors = [
      ...contentValidation.errors,
      ...prerequisiteValidation.errors,
      ...technicalValidation.errors
    ];

    const allWarnings = [
      ...contentValidation.warnings,
      ...prerequisiteValidation.warnings,
      ...technicalValidation.warnings
    ];

    const details = {
      contentValidation: contentValidation.details,
      prerequisiteValidation: prerequisiteValidation.details,
      technicalValidation: technicalValidation.details,
      overallScore: 0 // Will be calculated based on validation results
    };

    // Calculate overall score (simplified)
    const totalChecks = 10; // Total number of validation checks we consider
    const failedChecks = [
      contentValidation.valid,
      prerequisiteValidation.valid,
      technicalValidation.valid
    ].filter(valid => !valid).length;

    details.overallScore = ((totalChecks - failedChecks) / totalChecks) * 100;

    return {
      valid: allErrors.length === 0,
      errors: allErrors,
      warnings: allWarnings,
      details
    };
  }

  /**
   * Updates validation rules
   * @param {Object} newRules - New validation rules to apply
   */
  updateValidationRules(newRules) {
    this.validationRules = { ...this.validationRules, ...newRules };
  }

  /**
   * Gets current validation rules
   * @returns {Object} Current validation rules
   */
  getValidationRules() {
    return { ...this.validationRules };
  }
}

export { ValidationService };