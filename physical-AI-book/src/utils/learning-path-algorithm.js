/**
 * Learning Path Algorithm
 * Implements the personalized learning path algorithm
 */

class LearningPathAlgorithm {
  /**
   * Generates a personalized learning path based on user profile
   * @param {Array<Object>} modules - All available modules
   * @param {Object} userProfile - User's profile with preferences
   * @returns {Array<Object>} Ordered array of modules for the user
   */
  static generatePersonalizedPath(modules, userProfile) {
    if (!Array.isArray(modules) || !userProfile) {
      return modules || [];
    }

    // Start with all modules
    let path = [...modules];

    // Filter modules based on user's skill level
    path = this._filterBySkillLevel(path, userProfile);

    // Sort modules by prerequisites and skill progression
    path = this._sortModulesByPrerequisites(path);

    // Adjust order based on user interests
    path = this._adjustOrderByInterests(path, userProfile);

    // Adjust order based on user progress (show incomplete modules first)
    path = this._adjustOrderByProgress(path, userProfile);

    // Add adaptive difficulty based on user performance
    path = this._adjustByPerformance(path, userProfile);

    return path;
  }

  /**
   * Filters modules based on user's skill level
   * @private
   */
  static _filterBySkillLevel(modules, userProfile) {
    if (!userProfile.skillLevel) return modules;

    // For beginners, only show modules at their level or lower
    if (userProfile.skillLevel === 'beginner') {
      return modules.filter(module =>
        module.requiredSkillLevel === 'beginner' ||
        module.requiredSkillLevel === 'intro'
      );
    }

    // For intermediate, show beginner and intermediate
    if (userProfile.skillLevel === 'intermediate') {
      return modules.filter(module =>
        ['beginner', 'intro', 'intermediate'].includes(module.requiredSkillLevel)
      );
    }

    // For advanced, show all levels
    return modules;
  }

  /**
   * Sorts modules by prerequisites to ensure proper learning order
   * @private
   */
  static _sortModulesByPrerequisites(modules) {
    // Create a map for quick lookup
    const moduleMap = new Map();
    modules.forEach(module => {
      moduleMap.set(module.id, module);
    });

    // Topological sort to handle prerequisites
    const sorted = [];
    const visited = new Set();
    const visiting = new Set();

    const visit = (moduleId) => {
      if (visited.has(moduleId)) return;
      if (visiting.has(moduleId)) {
        console.warn('Circular dependency detected in module prerequisites');
        return;
      }

      visiting.add(moduleId);
      const module = moduleMap.get(moduleId);

      if (module && module.prerequisites) {
        for (const prereqId of module.prerequisites) {
          if (moduleMap.has(prereqId)) {
            visit(prereqId);
          }
        }
      }

      visiting.delete(moduleId);
      visited.add(moduleId);
      sorted.push(module);
    };

    for (const module of modules) {
      if (!visited.has(module.id)) {
        visit(module.id);
      }
    }

    return sorted;
  }

  /**
   * Adjusts module order based on user interests
   * @private
   */
  static _adjustOrderByInterests(modules, userProfile) {
    if (!userProfile.interests || userProfile.interests.length === 0) {
      return modules;
    }

    return modules.sort((a, b) => {
      const aInterestMatch = userProfile.interests.some(interest =>
        a.title.toLowerCase().includes(interest.toLowerCase()) ||
        a.description.toLowerCase().includes(interest.toLowerCase()) ||
        (a.tags && a.tags.some(tag => tag.toLowerCase().includes(interest.toLowerCase())))
      );
      const bInterestMatch = userProfile.interests.some(interest =>
        b.title.toLowerCase().includes(interest.toLowerCase()) ||
        b.description.toLowerCase().includes(interest.toLowerCase()) ||
        (b.tags && b.tags.some(tag => tag.toLowerCase().includes(interest.toLowerCase())))
      );

      // Prioritize modules that match user interests
      if (aInterestMatch && !bInterestMatch) return -1;
      if (!aInterestMatch && bInterestMatch) return 1;
      return 0;
    });
  }

  /**
   * Adjusts module order based on user progress (incomplete first)
   * @private
   */
  static _adjustOrderByProgress(modules, userProfile) {
    if (!userProfile.progress) {
      return modules;
    }

    return modules.sort((a, b) => {
      const aProgress = userProfile.progress[a.id] || 0;
      const bProgress = userProfile.progress[b.id] || 0;

      // Show incomplete modules first
      if (aProgress < 100 && bProgress === 100) return -1;
      if (aProgress === 100 && bProgress < 100) return 1;

      // Then sort by progress (modules with less progress first)
      return aProgress - bProgress;
    });
  }

  /**
   * Adjusts module order based on user performance
   * @private
   */
  static _adjustByPerformance(modules, userProfile) {
    if (!userProfile.performance) {
      return modules;
    }

    // If user is struggling with certain topics, adjust accordingly
    // This would be based on assessment scores, time spent, etc.
    return modules;
  }

  /**
   * Calculates the estimated time to complete a learning path
   * @param {Array<Object>} path - Learning path
   * @param {Object} userProfile - User's profile
   * @returns {Object} Time estimates
   */
  static calculateTimeEstimates(path, userProfile) {
    const avgReadingSpeed = userProfile.readingSpeed || 200; // words per minute
    const timePerModule = [];

    for (const module of path) {
      // Estimate reading time based on content length
      const wordCount = module.content ? module.content.split(/\s+/).length : 0;
      const readingTime = Math.ceil(wordCount / avgReadingSpeed);

      // Add additional time for exercises and assessments
      const exerciseTime = (module.exercises?.length || 0) * 15; // 15 min per exercise
      const assessmentTime = module.hasAssessment ? 30 : 0; // 30 min for assessment

      timePerModule.push({
        moduleId: module.id,
        estimatedMinutes: readingTime + exerciseTime + assessmentTime,
        readingTime,
        exerciseTime,
        assessmentTime
      });
    }

    const totalMinutes = timePerModule.reduce((sum, item) => sum + item.estimatedMinutes, 0);

    return {
      timePerModule,
      totalMinutes,
      totalHours: Math.ceil(totalMinutes / 60),
      totalTimeByCategory: this._groupTimeByCategory(timePerModule, path)
    };
  }

  /**
   * Groups time estimates by category
   * @private
   */
  static _groupTimeByCategory(timePerModule, modules) {
    const categoryMap = new Map();

    for (let i = 0; i < modules.length; i++) {
      const module = modules[i];
      const timeInfo = timePerModule[i];
      const category = module.category || 'uncategorized';

      if (!categoryMap.has(category)) {
        categoryMap.set(category, {
          modules: [],
          totalMinutes: 0
        });
      }

      const categoryData = categoryMap.get(category);
      categoryData.modules.push({
        moduleId: module.id,
        title: module.title,
        minutes: timeInfo.estimatedMinutes
      });
      categoryData.totalMinutes += timeInfo.estimatedMinutes;
    }

    return Object.fromEntries(categoryMap);
  }

  /**
   * Generates recommendations for additional learning
   * @param {Array<Object>} completedPath - Modules in the completed path
   * @param {Object} userProfile - User's profile
   * @returns {Array<Object>} Recommended modules
   */
  static generateRecommendations(completedPath, userProfile) {
    // This would implement more sophisticated recommendation logic
    // For now, return modules that are related to the completed modules
    const recommendations = [];

    // Find modules related to the ones the user has completed
    for (const module of completedPath) {
      if (module.relatedModules) {
        recommendations.push(...module.relatedModules);
      }
    }

    // Filter out modules the user has already completed
    return recommendations.filter(rec =>
      !completedPath.some(m => m.id === rec.id)
    );
  }
}

export { LearningPathAlgorithm };