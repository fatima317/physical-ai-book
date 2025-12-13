/**
 * ProgressTrackingService
 * Handles comprehensive user progress tracking and analytics
 */

class ProgressTrackingService {
  constructor(userProfileService, learningModuleService) {
    this.userProfileService = userProfileService;
    this.learningModuleService = learningModuleService;
  }

  /**
   * Records user's progress for a specific module
   * @param {string} userId - User ID
   * @param {string} moduleId - Module ID
   * @param {number} progress - Progress percentage (0-100)
   * @param {Object} metadata - Additional progress metadata
   * @returns {Object} Updated progress record
   */
  async recordProgress(userId, moduleId, progress, metadata = {}) {
    // Validate inputs
    if (!userId || !moduleId) {
      throw new Error('User ID and Module ID are required');
    }

    if (progress < 0 || progress > 100) {
      throw new Error('Progress must be between 0 and 100');
    }

    // Get user profile
    const userProfile = await this.userProfileService.getProfile(userId);
    if (!userProfile) {
      throw new Error(`User profile not found for ID: ${userId}`);
    }

    // Get module
    const module = await this.learningModuleService.getModule(moduleId);
    if (!module) {
      throw new Error(`Module not found for ID: ${moduleId}`);
    }

    // Update progress in user profile
    const updatedProfile = await this.userProfileService.updateModuleProgress(
      userId,
      moduleId,
      progress
    );

    // Create progress record with metadata
    const progressRecord = {
      userId,
      moduleId,
      progress,
      timestamp: new Date().toISOString(),
      completed: progress === 100,
      timeSpent: metadata.timeSpent || 0,
      interactions: metadata.interactions || 0,
      score: metadata.score || null,
      attempts: metadata.attempts || 1
    };

    // Save progress record to storage (in a real implementation)
    // For now, we'll just return the record
    return progressRecord;
  }

  /**
   * Gets user's progress for a specific module
   * @param {string} userId - User ID
   * @param {string} moduleId - Module ID
   * @returns {Object} Progress record
   */
  async getProgress(userId, moduleId) {
    if (!userId || !moduleId) {
      throw new Error('User ID and Module ID are required');
    }

    const userProfile = await this.userProfileService.getProfile(userId);
    if (!userProfile || !userProfile.progress) {
      return null;
    }

    const moduleProgress = userProfile.progress[moduleId];
    if (!moduleProgress) {
      return null;
    }

    return {
      userId,
      moduleId,
      progress: moduleProgress,
      completed: moduleProgress === 100,
      lastAccessed: userProfile.updatedAt
    };
  }

  /**
   * Gets user's overall progress across all modules
   * @param {string} userId - User ID
   * @returns {Object} Overall progress summary
   */
  async getOverallProgress(userId) {
    const userProfile = await this.userProfileService.getProfile(userId);
    if (!userProfile) {
      throw new Error(`User profile not found for ID: ${userId}`);
    }

    const allModules = await this.learningModuleService.getAllModules();
    const userProgress = userProfile.progress || {};

    const completedModules = Object.keys(userProgress).filter(
      moduleId => userProgress[moduleId] === 100
    );

    const inProgressModules = Object.keys(userProgress).filter(
      moduleId => userProgress[moduleId] > 0 && userProgress[moduleId] < 100
    );

    // Calculate overall progress percentage
    const totalProgress = Object.values(userProgress).reduce((sum, progress) => sum + progress, 0);
    const overallPercentage = allModules.length > 0
      ? Math.round((totalProgress / (allModules.length * 100)) * 100)
      : 0;

    return {
      userId,
      totalModules: allModules.length,
      completedModules: completedModules.length,
      inProgressModules: inProgressModules.length,
      notStartedModules: allModules.length - completedModules.length - inProgressModules.length,
      overallPercentage,
      completedModuleIds: completedModules,
      inProgressModuleIds: inProgressModules,
      lastActivity: userProfile.updatedAt
    };
  }

  /**
   * Gets progress for all users in a specific module
   * @param {string} moduleId - Module ID
   * @returns {Array<Object>} Array of user progress records
   */
  async getModuleProgressOverview(moduleId) {
    const allProfiles = await this.userProfileService.getAllProfiles();
    const progressRecords = [];

    for (const profile of allProfiles) {
      if (profile.progress && profile.progress[moduleId] !== undefined) {
        progressRecords.push({
          userId: profile.id,
          progress: profile.progress[moduleId],
          completed: profile.progress[moduleId] === 100,
          lastUpdated: profile.updatedAt
        });
      }
    }

    // Calculate module statistics
    const totalUsers = progressRecords.length;
    const completedUsers = progressRecords.filter(record => record.completed).length;
    const averageProgress = totalUsers > 0
      ? Math.round(progressRecords.reduce((sum, record) => sum + record.progress, 0) / totalUsers)
      : 0;

    return {
      moduleId,
      totalUsers,
      completedUsers,
      averageProgress,
      completionRate: totalUsers > 0 ? Math.round((completedUsers / totalUsers) * 100) : 0,
      progressRecords
    };
  }

  /**
   * Resets user's progress for a specific module
   * @param {string} userId - User ID
   * @param {string} moduleId - Module ID
   * @returns {boolean} True if reset was successful
   */
  async resetProgress(userId, moduleId) {
    if (!userId || !moduleId) {
      throw new Error('User ID and Module ID are required');
    }

    const result = await this.userProfileService.resetModuleProgress(userId, moduleId);
    return result !== null;
  }

  /**
   * Gets user's learning path progress
   * @param {string} userId - User ID
   * @returns {Object} Learning path progress
   */
  async getLearningPathProgress(userId) {
    const userProfile = await this.userProfileService.getProfile(userId);
    const allModules = await this.learningModuleService.getAllModules();

    if (!userProfile || !allModules) {
      return null;
    }

    // Get user's personalized learning path
    const learningPath = await this.userProfileService.getPersonalizedLearningPath(userId);

    // Map each module in the learning path with its progress
    const pathProgress = learningPath.map(moduleId => {
      const module = allModules.find(m => m.id === moduleId);
      const progress = (userProfile.progress && userProfile.progress[moduleId]) || 0;

      return {
        moduleId: module ? module.id : moduleId,
        title: module ? module.title : `Unknown Module (${moduleId})`,
        progress,
        completed: progress === 100,
        requiredSkillLevel: module ? module.requiredSkillLevel : 'unknown',
        category: module ? module.category : 'unknown',
        estimatedTime: module ? module.estimatedTime : null
      };
    });

    // Calculate path statistics
    const completedCount = pathProgress.filter(item => item.completed).length;
    const totalCount = pathProgress.length;
    const pathCompletion = totalCount > 0 ? Math.round((completedCount / totalCount) * 100) : 0;

    return {
      userId,
      path: pathProgress,
      completedCount,
      totalCount,
      pathCompletion,
      nextRecommended: this._getNextRecommended(pathProgress)
    };
  }

  /**
   * Gets next recommended module based on progress
   * @private
   */
  _getNextRecommended(pathProgress) {
    // Find the first module that isn't completed
    for (const item of pathProgress) {
      if (!item.completed) {
        return item;
      }
    }

    // If all are completed, return the last one
    return pathProgress[pathProgress.length - 1];
  }

  /**
   * Calculates user's performance metrics
   * @param {string} userId - User ID
   * @returns {Object} Performance metrics
   */
  async getPerformanceMetrics(userId) {
    const userProfile = await this.userProfileService.getProfile(userId);
    if (!userProfile || !userProfile.progress) {
      return null;
    }

    const progressValues = Object.values(userProfile.progress);
    const completedCount = progressValues.filter(p => p === 100).length;
    const totalProgress = progressValues.reduce((sum, progress) => sum + progress, 0);

    // Calculate learning velocity (modules completed per time period)
    const now = new Date();
    const startDate = new Date(userProfile.createdAt);
    const daysActive = Math.max(1, Math.ceil((now - startDate) / (1000 * 60 * 60 * 24)));

    return {
      userId,
      completedModules: completedCount,
      totalProgress: totalProgress,
      averageProgress: progressValues.length > 0 ? Math.round(totalProgress / progressValues.length) : 0,
      learningVelocity: Math.round(completedCount / daysActive * 100) / 100, // modules per day
      consistency: this._calculateConsistency(userProfile.progress),
      skillGrowth: this._calculateSkillGrowth(userProfile)
    };
  }

  /**
   * Calculates learning consistency
   * @private
   */
  _calculateConsistency(progress) {
    // This would analyze the pattern of progress updates over time
    // For now, we'll return a simple consistency score
    const progressValues = Object.values(progress);
    if (progressValues.length === 0) return 0;

    // Calculate variance in progress values as a simple consistency measure
    const mean = progressValues.reduce((sum, val) => sum + val, 0) / progressValues.length;
    const variance = progressValues.reduce((sum, val) => sum + Math.pow(val - mean, 2), 0) / progressValues.length;

    // Lower variance means more consistent progress
    return Math.max(0, 100 - variance);
  }

  /**
   * Calculates skill growth based on module completion patterns
   * @private
   */
  _calculateSkillGrowth(userProfile) {
    // This would analyze the progression through modules of increasing difficulty
    // For now, we'll return a simple growth score
    return 50; // Neutral growth score
  }

  /**
   * Generates a progress report for a user
   * @param {string} userId - User ID
   * @returns {Object} Progress report
   */
  async generateProgressReport(userId) {
    const overallProgress = await this.getOverallProgress(userId);
    const learningPathProgress = await this.getLearningPathProgress(userId);
    const performanceMetrics = await this.getPerformanceMetrics(userId);

    return {
      userId,
      reportDate: new Date().toISOString(),
      overallProgress,
      learningPathProgress,
      performanceMetrics,
      recommendations: await this._generateRecommendations(userId)
    };
  }

  /**
   * Generates personalized recommendations based on progress
   * @private
   */
  async _generateRecommendations(userId) {
    const userProfile = await this.userProfileService.getProfile(userId);
    const overallProgress = await this.getOverallProgress(userId);

    const recommendations = [];

    // Suggest modules based on incomplete prerequisites
    // This would be more sophisticated in a real implementation

    // Suggest modules based on user interests
    if (userProfile.interests && userProfile.interests.length > 0) {
      recommendations.push({
        type: 'interest',
        message: 'Based on your interests, you might enjoy these modules',
        modules: userProfile.interests.slice(0, 2).map(interest =>
          `module-about-${interest.replace(/\s+/g, '-').toLowerCase()}`
        )
      });
    }

    // Suggest modules based on skill level
    if (userProfile.skillLevel) {
      recommendations.push({
        type: 'skill',
        message: 'Modules that match your skill level',
        modules: [userProfile.skillLevel]
      });
    }

    return recommendations;
  }
}

export { ProgressTrackingService };