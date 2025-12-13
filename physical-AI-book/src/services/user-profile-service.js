/**
 * UserProfileService
 * Handles user profile management and operations
 */

import UserProfile from '../models/user-profile.js';
import { StorageService } from './storage-service.js';

class UserProfileService {
  constructor(storagePath = './data/user-profiles') {
    this.storage = new StorageService(storagePath);
  }

  /**
   * Creates a new user profile
   * @param {Object} profileData - User profile data
   * @returns {UserProfile} Created user profile
   */
  async createProfile(profileData) {
    if (!profileData.id) {
      throw new Error('User ID is required to create a profile');
    }

    const existing = await this.storage.load('profiles', profileData.id);
    if (existing) {
      throw new Error(`Profile with ID ${profileData.id} already exists`);
    }

    const profile = new UserProfile(profileData);
    const validationErrors = profile.validate();

    if (validationErrors.length > 0) {
      throw new Error(`Validation errors: ${validationErrors.join(', ')}`);
    }

    return await this.storage.save('profiles', profile.id, profile);
  }

  /**
   * Gets a user profile by ID
   * @param {string} userId - User ID
   * @returns {UserProfile|null} User profile or null if not found
   */
  async getProfile(userId) {
    if (!userId) {
      throw new Error('User ID is required');
    }

    const profile = await this.storage.load('profiles', userId);
    return profile;
  }

  /**
   * Updates a user profile
   * @param {string} userId - User ID
   * @param {Object} updates - Profile updates
   * @returns {UserProfile|null} Updated profile or null if not found
   */
  async updateProfile(userId, updates) {
    if (!userId) {
      throw new Error('User ID is required');
    }

    const existingProfile = await this.storage.load('profiles', userId);
    if (!existingProfile) {
      return null;
    }

    // Update profile properties
    const updatedProfile = { ...existingProfile };
    Object.keys(updates).forEach(key => {
      if (key !== 'id' && key !== 'createdAt') { // Don't allow ID or creation date to be changed
        updatedProfile[key] = updates[key];
      }
    });

    updatedProfile.updatedAt = new Date().toISOString();

    // Create a UserProfile instance to validate
    const profileInstance = new UserProfile(updatedProfile);
    const validationErrors = profileInstance.validate();
    if (validationErrors.length > 0) {
      throw new Error(`Validation errors after update: ${validationErrors.join(', ')}`);
    }

    return await this.storage.save('profiles', userId, updatedProfile);
  }

  /**
   * Deletes a user profile
   * @param {string} userId - User ID
   * @returns {boolean} True if profile was deleted
   */
  async deleteProfile(userId) {
    if (!userId) {
      throw new Error('User ID is required');
    }

    return await this.storage.delete('profiles', userId);
  }

  /**
   * Gets all user profiles
   * @returns {Array<UserProfile>} Array of all user profiles
   */
  async getAllProfiles() {
    return await this.storage.list('profiles');
  }

  /**
   * Updates a user's skill level
   * @param {string} userId - User ID
   * @param {string} skillLevel - New skill level
   * @returns {UserProfile|null} Updated profile or null if not found
   */
  async updateSkillLevel(userId, skillLevel) {
    const profile = await this.storage.load('profiles', userId);
    if (!profile) {
      return null;
    }

    // Create a UserProfile instance to use its method
    const profileInstance = new UserProfile(profile);
    profileInstance.updateSkillLevel(skillLevel);

    return await this.storage.save('profiles', userId, profileInstance);
  }

  /**
   * Updates a user's language preference
   * @param {string} userId - User ID
   * @param {string} language - New language preference
   * @returns {UserProfile|null} Updated profile or null if not found
   */
  async updateLanguagePreference(userId, language) {
    const profile = await this.storage.load('profiles', userId);
    if (!profile) {
      return null;
    }

    // Create a UserProfile instance to use its method
    const profileInstance = new UserProfile(profile);
    profileInstance.updateLanguagePreference(language);

    return await this.storage.save('profiles', userId, profileInstance);
  }

  /**
   * Updates progress for a specific module
   * @param {string} userId - User ID
   * @param {string} moduleId - Module ID
   * @param {number} percentage - Completion percentage (0-100)
   * @returns {UserProfile|null} Updated profile or null if not found
   */
  async updateModuleProgress(userId, moduleId, percentage) {
    const profile = await this.storage.load('profiles', userId);
    if (!profile) {
      return null;
    }

    // Create a UserProfile instance to use its method
    const profileInstance = new UserProfile(profile);
    profileInstance.updateModuleProgress(moduleId, percentage);

    return await this.storage.save('profiles', userId, profileInstance);
  }

  /**
   * Gets users by skill level
   * @param {string} skillLevel - Skill level to filter by
   * @returns {Array<UserProfile>} Array of matching user profiles
   */
  async getUsersBySkillLevel(skillLevel) {
    const allProfiles = await this.storage.list('profiles');
    return allProfiles.filter(
      profile => profile.skillLevel && profile.skillLevel.toLowerCase() === skillLevel.toLowerCase()
    );
  }

  /**
   * Gets users by language preference
   * @param {string} language - Language to filter by
   * @returns {Array<UserProfile>} Array of matching user profiles
   */
  async getUsersByLanguage(language) {
    const allProfiles = await this.storage.list('profiles');
    return allProfiles.filter(
      profile => profile.languagePreference && profile.languagePreference.toLowerCase() === language.toLowerCase()
    );
  }

  /**
   * Adds a module to a user's learning path
   * @param {string} userId - User ID
   * @param {string} moduleId - Module ID to add
   * @returns {UserProfile|null} Updated profile or null if not found
   */
  async addToLearningPath(userId, moduleId) {
    const profile = await this.storage.load('profiles', userId);
    if (!profile) {
      return null;
    }

    if (!profile.learningPath.includes(moduleId)) {
      profile.learningPath.push(moduleId);
      profile.updatedAt = new Date().toISOString();
      return await this.storage.save('profiles', userId, profile);
    }

    return profile;
  }

  /**
   * Gets user's personalized learning path based on skill level and progress
   * @param {string} userId - User ID
   * @returns {Array<string>} Array of module IDs in recommended order
   */
  async getPersonalizedLearningPath(userId) {
    const profile = await this.storage.load('profiles', userId);
    if (!profile) {
      return [];
    }

    // This would implement more sophisticated logic in a real system
    // For now, we'll return the learning path as-is
    return profile.learningPath || [];
  }

  /**
   * Resets a user's progress for a specific module
   * @param {string} userId - User ID
   * @param {string} moduleId - Module ID to reset
   * @returns {UserProfile|null} Updated profile or null if not found
   */
  async resetModuleProgress(userId, moduleId) {
    const profile = await this.storage.load('profiles', userId);
    if (!profile) {
      return null;
    }

    if (profile.progress && profile.progress[moduleId] !== undefined) {
      profile.progress[moduleId] = 0;
      profile.updatedAt = new Date().toISOString();
      return await this.storage.save('profiles', userId, profile);
    }

    return profile;
  }

  /**
   * Updates a personalization preference for a user
   * @param {string} userId - User ID
   * @param {string} key - Preference key
   * @param {*} value - Preference value
   * @returns {UserProfile|null} Updated profile or null if not found
   */
  async updatePreference(userId, key, value) {
    const profile = await this.storage.load('profiles', userId);
    if (!profile) {
      return null;
    }

    // Create a UserProfile instance to use its method
    const profileInstance = new UserProfile(profile);
    profileInstance.updatePreference(key, value);

    return await this.storage.save('profiles', userId, profileInstance);
  }
}

export { UserProfileService };