/**
 * LearningModuleService
 * Handles learning module management and operations
 */

import LearningModule from '../models/learning-module.js';
import { StorageService } from './storage-service.js';

class LearningModuleService {
  constructor(storagePath = './data/learning-modules') {
    this.storage = new StorageService(storagePath);
  }

  /**
   * Creates a new learning module
   * @param {Object} moduleData - Module data
   * @returns {LearningModule} Created learning module
   */
  async createModule(moduleData) {
    if (!moduleData.id) {
      throw new Error('Module ID is required to create a module');
    }

    const existing = await this.storage.load('modules', moduleData.id);
    if (existing) {
      throw new Error(`Module with ID ${moduleData.id} already exists`);
    }

    const module = new LearningModule(moduleData);
    const validationErrors = module.validate();

    if (validationErrors.length > 0) {
      throw new Error(`Validation errors: ${validationErrors.join(', ')}`);
    }

    return await this.storage.save('modules', module.id, module);
  }

  /**
   * Gets a learning module by ID
   * @param {string} moduleId - Module ID
   * @returns {LearningModule|null} Learning module or null if not found
   */
  async getModule(moduleId) {
    if (!moduleId) {
      throw new Error('Module ID is required');
    }

    const module = await this.storage.load('modules', moduleId);
    return module;
  }

  /**
   * Updates a learning module
   * @param {string} moduleId - Module ID
   * @param {Object} updates - Module updates
   * @returns {LearningModule|null} Updated module or null if not found
   */
  async updateModule(moduleId, updates) {
    if (!moduleId) {
      throw new Error('Module ID is required');
    }

    const existingModule = await this.storage.load('modules', moduleId);
    if (!existingModule) {
      return null;
    }

    // Update module properties
    const updatedModule = { ...existingModule };
    Object.keys(updates).forEach(key => {
      if (key !== 'id' && key !== 'createdAt') { // Don't allow ID or creation date to be changed
        updatedModule[key] = updates[key];
      }
    });

    updatedModule.updatedAt = new Date().toISOString();

    // Create a LearningModule instance to validate
    const moduleInstance = new LearningModule(updatedModule);
    const validationErrors = moduleInstance.validate();
    if (validationErrors.length > 0) {
      throw new Error(`Validation errors after update: ${validationErrors.join(', ')}`);
    }

    return await this.storage.save('modules', moduleId, updatedModule);
  }

  /**
   * Deletes a learning module
   * @param {string} moduleId - Module ID
   * @returns {boolean} True if module was deleted
   */
  async deleteModule(moduleId) {
    if (!moduleId) {
      throw new Error('Module ID is required');
    }

    return await this.storage.delete('modules', moduleId);
  }

  /**
   * Gets all learning modules
   * @returns {Array<LearningModule>} Array of all learning modules
   */
  async getAllModules() {
    return await this.storage.list('modules');
  }

  /**
   * Gets modules by category
   * @param {string} category - Category to filter by
   * @returns {Array<LearningModule>} Array of matching learning modules
   */
  async getModulesByCategory(category) {
    const allModules = await this.storage.list('modules');
    return allModules.filter(
      module => module.category && module.category.toLowerCase() === category.toLowerCase()
    );
  }

  /**
   * Gets modules by skill level
   * @param {string} skillLevel - Skill level to filter by
   * @returns {Array<LearningModule>} Array of matching learning modules
   */
  async getModulesBySkillLevel(skillLevel) {
    const allModules = await this.storage.list('modules');
    return allModules.filter(
      module => module.requiredSkillLevel && module.requiredSkillLevel.toLowerCase() === skillLevel.toLowerCase()
    );
  }

  /**
   * Gets modules by language
   * @param {string} language - Language to filter by
   * @returns {Array<LearningModule>} Array of matching learning modules
   */
  async getModulesByLanguage(language) {
    const allModules = await this.storage.list('modules');
    return allModules.filter(
      module => module.language && module.language.toLowerCase() === language.toLowerCase()
    );
  }

  /**
   * Adds a prerequisite to a module
   * @param {string} moduleId - Module ID
   * @param {string} prerequisiteId - Prerequisite module ID to add
   * @returns {LearningModule|null} Updated module or null if not found
   */
  async addPrerequisite(moduleId, prerequisiteId) {
    const module = await this.storage.load('modules', moduleId);
    if (!module) {
      return null;
    }

    if (!module.prerequisites.includes(prerequisiteId)) {
      module.prerequisites.push(prerequisiteId);
      module.updatedAt = new Date().toISOString();
      return await this.storage.save('modules', moduleId, module);
    }

    return module;
  }

  /**
   * Removes a prerequisite from a module
   * @param {string} moduleId - Module ID
   * @param {string} prerequisiteId - Prerequisite module ID to remove
   * @returns {LearningModule|null} Updated module or null if not found
   */
  async removePrerequisite(moduleId, prerequisiteId) {
    const module = await this.storage.load('modules', moduleId);
    if (!module) {
      return null;
    }

    const index = module.prerequisites.indexOf(prerequisiteId);
    if (index > -1) {
      module.prerequisites.splice(index, 1);
      module.updatedAt = new Date().toISOString();
      return await this.storage.save('modules', moduleId, module);
    }

    return module;
  }

  /**
   * Gets module prerequisites
   * @param {string} moduleId - Module ID
   * @returns {Array<string>} Array of prerequisite module IDs
   */
  async getPrerequisites(moduleId) {
    const module = await this.storage.load('modules', moduleId);
    if (!module) {
      return [];
    }

    return [...(module.prerequisites || [])];
  }

  /**
   * Gets modules that depend on a specific module
   * @param {string} moduleId - Module ID
   * @returns {Array<LearningModule>} Array of modules that have this module as a prerequisite
   */
  async getDependentModules(moduleId) {
    const allModules = await this.storage.list('modules');
    return allModules.filter(
      module => module.prerequisites && module.prerequisites.includes(moduleId)
    );
  }

  /**
   * Validates module content for technical accuracy
   * @param {string} moduleId - Module ID
   * @returns {Object} Validation results
   */
  async validateModuleContent(moduleId) {
    const module = await this.storage.load('modules', moduleId);
    if (!module) {
      return { valid: false, errors: ['Module not found'] };
    }

    // In a real implementation, this would validate code examples, links, etc.
    const validationErrors = [];

    // Check if content exists
    if (!module.content || module.content.trim() === '') {
      validationErrors.push('Module content is empty');
    }

    // Check if title exists
    if (!module.title || module.title.trim() === '') {
      validationErrors.push('Module title is required');
    }

    // Validate interactive elements
    if (module.interactiveElements && Array.isArray(module.interactiveElements)) {
      for (let i = 0; i < module.interactiveElements.length; i++) {
        const element = module.interactiveElements[i];
        if (!element.type) {
          validationErrors.push(`Interactive element at index ${i} is missing type`);
        }
      }
    }

    return {
      valid: validationErrors.length === 0,
      errors: validationErrors,
      warnings: [] // Additional warnings could be added here
    };
  }

  /**
   * Gets module statistics
   * @param {string} moduleId - Module ID
   * @returns {Object} Module statistics
   */
  async getModuleStats(moduleId) {
    const module = await this.storage.load('modules', moduleId);
    if (!module) {
      return null;
    }

    return {
      id: module.id,
      title: module.title,
      wordCount: module.content ? module.content.split(/\s+/).length : 0,
      estimatedReadingTime: module.content ? Math.ceil(module.content.split(/\s+/).length / 200) : 0, // 200 words per minute
      interactiveElementCount: module.interactiveElements ? module.interactiveElements.length : 0,
      codeExampleCount: module.codeExamples ? module.codeExamples.length : 0,
      createdAt: module.createdAt,
      updatedAt: module.updatedAt
    };
  }

  /**
   * Updates module progress for a user
   * @param {string} moduleId - Module ID
   * @param {string} userId - User ID
   * @param {number} progress - Progress percentage (0-100)
   * @returns {Object} Updated progress information
   */
  async updateModuleProgress(moduleId, userId, progress) {
    const module = await this.storage.load('modules', moduleId);
    if (!module) {
      return null;
    }

    // Create userProgress if it doesn't exist
    if (!module.userProgress) {
      module.userProgress = {};
    }

    // Update progress for the user
    module.userProgress[userId] = {
      progress: Math.max(0, Math.min(100, progress)), // Clamp between 0 and 100
      lastAccessed: new Date().toISOString(),
      completed: progress === 100
    };

    module.updatedAt = new Date().toISOString();

    // Save the updated module back to storage
    const updatedModule = await this.storage.save('modules', moduleId, module);

    return updatedModule.userProgress[userId];
  }

  /**
   * Gets user's progress for a module
   * @param {string} moduleId - Module ID
   * @param {string} userId - User ID
   * @returns {Object|null} Progress information or null if not found
   */
  async getModuleProgress(moduleId, userId) {
    const module = await this.storage.load('modules', moduleId);
    if (!module || !module.userProgress) {
      return null;
    }

    return module.userProgress[userId] || null;
  }
}

export { LearningModuleService };