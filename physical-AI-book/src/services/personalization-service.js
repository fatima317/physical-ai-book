/**
 * PersonalizationService
 * Handles content personalization based on user profiles and preferences
 */

import LearningModule from '../models/learning-module.js';
import UserProfile from '../models/user-profile.js';

class PersonalizationService {
  constructor() {
    // In a real implementation, this might connect to a recommendation engine
    // For now, we'll implement basic personalization logic
  }

  /**
   * Personalizes a single learning module based on user profile
   * @param {LearningModule} module - The module to personalize
   * @param {UserProfile} userProfile - User's profile with preferences
   * @returns {LearningModule} Personalized module
   */
  async personalizeModule(module, userProfile) {
    if (!module || !userProfile) {
      return module; // Return original if no personalization data
    }

    // Create a copy of the module to avoid modifying the original
    const personalizedModule = new LearningModule({ ...module });

    // Adjust content based on skill level
    if (userProfile.skillLevel) {
      personalizedModule.content = this._adjustContentBySkillLevel(
        module.content,
        userProfile.skillLevel,
        module.requiredSkillLevel
      );
    }

    // Modify difficulty indicators based on user skill level
    if (userProfile.skillLevel) {
      personalizedModule.difficulty = this._adjustDifficulty(
        module.difficulty,
        userProfile.skillLevel,
        module.requiredSkillLevel
      );
    }

    // Filter or highlight relevant content based on user interests
    if (userProfile.interests && Array.isArray(userProfile.interests)) {
      personalizedModule.content = this._highlightRelevantContent(
        module.content,
        userProfile.interests
      );
    }

    // Adjust examples based on user background
    if (userProfile.background) {
      personalizedModule.content = this._adjustExamplesByBackground(
        module.content,
        userProfile.background
      );
    }

    // Add personalized recommendations
    personalizedModule.personalizedRecommendations =
      this._generateRecommendations(module, userProfile);

    return personalizedModule;
  }

  /**
   * Personalizes multiple learning modules based on user profile
   * @param {Array<LearningModule>} modules - Array of modules to personalize
   * @param {UserProfile} userProfile - User's profile with preferences
   * @returns {Array<LearningModule>} Array of personalized modules
   */
  async personalizeModules(modules, userProfile) {
    if (!Array.isArray(modules) || !userProfile) {
      return modules;
    }

    const personalizedModules = [];
    for (const module of modules) {
      const personalized = await this.personalizeModule(module, userProfile);
      personalizedModules.push(personalized);
    }

    return personalizedModules;
  }

  /**
   * Generates a personalized learning path based on user profile
   * @param {Array<LearningModule>} allModules - All available modules
   * @param {UserProfile} userProfile - User's profile with preferences
   * @returns {Array<LearningModule>} Ordered array of modules for the user
   */
  async generatePersonalizedLearningPath(allModules, userProfile) {
    if (!Array.isArray(allModules) || !userProfile) {
      return allModules || [];
    }

    // Filter modules based on user's skill level
    let filteredModules = allModules.filter(module => {
      if (!userProfile.skillLevel) return true;

      // For beginners, only show modules at their level or lower
      if (userProfile.skillLevel === 'beginner') {
        return module.requiredSkillLevel === 'beginner';
      }

      // For intermediate, show beginner and intermediate
      if (userProfile.skillLevel === 'intermediate') {
        return ['beginner', 'intermediate'].includes(module.requiredSkillLevel);
      }

      // For advanced, show all levels
      return true;
    });

    // Sort modules by prerequisites and skill progression
    filteredModules = this._sortModulesByPrerequisites(filteredModules);

    // Reorder based on user interests
    if (userProfile.interests && userProfile.interests.length > 0) {
      filteredModules.sort((a, b) => {
        const aInterestMatch = userProfile.interests.some(interest =>
          a.title.toLowerCase().includes(interest.toLowerCase()) ||
          a.description.toLowerCase().includes(interest.toLowerCase())
        );
        const bInterestMatch = userProfile.interests.some(interest =>
          b.title.toLowerCase().includes(interest.toLowerCase()) ||
          b.description.toLowerCase().includes(interest.toLowerCase())
        );

        // Prioritize modules that match user interests
        if (aInterestMatch && !bInterestMatch) return -1;
        if (!aInterestMatch && bInterestMatch) return 1;
        return 0;
      });
    }

    return filteredModules;
  }

  /**
   * Adjusts content difficulty based on user skill level
   * @private
   */
  _adjustContentBySkillLevel(content, userSkillLevel, requiredSkillLevel) {
    if (!content) return content;

    // For beginners, simplify complex concepts and add more explanations
    if (userSkillLevel === 'beginner') {
      // Add more explanatory text for complex concepts
      let adjustedContent = content;

      // Replace complex terminology with simpler explanations where possible
      adjustedContent = adjustedContent.replace(/\b(abstraction|encapsulation|inheritance|algorithm|framework|library|dependency|configuration|simulation|perception|planning|execution|navigation|controller|actuator|sensor|ros|gazebo|jetson|nx|nvidia|vslam|whisper|llm|ai|ml|neural\s+network)\b/gi, (match) => {
        const lowerMatch = match.toLowerCase();
        switch(lowerMatch) {
          case 'abstraction':
            return `${match} (hiding complex details to make things simpler)`;
          case 'encapsulation':
            return `${match} (bundling data and functions together)`;
          case 'inheritance':
            return `${match} (getting features from a parent concept)`;
          case 'algorithm':
            return `${match} (a step-by-step procedure for solving a problem)`;
          case 'framework':
            return `${match} (a reusable set of tools and guidelines)`;
          case 'library':
            return `${match} (a collection of pre-written code)`;
          case 'dependency':
            return `${match} (something that one piece of code needs to work)`;
          case 'configuration':
            return `${match} (settings that control how something works)`;
          case 'simulation':
            return `${match} (a computer model that acts like the real thing)`;
          case 'perception':
            return `${match} (how a robot understands its environment)`;
          case 'planning':
            return `${match} (deciding what to do next)`;
          case 'execution':
            return `${match} (carrying out the planned actions)`;
          case 'navigation':
            return `${match} (finding the way from one place to another)`;
          case 'controller':
            return `${match} (a device that manages how something works)`;
          case 'actuator':
            return `${match} (a device that makes something move)`;
          case 'sensor':
            return `${match} (a device that detects things in the environment)`;
          case 'ros':
            return `${match} (Robot Operating System - software tools for robots)`;
          case 'gazebo':
            return `${match} (a simulation environment for robots)`;
          case 'jetson':
            return `${match} (a computer board for AI and robotics)`;
          case 'nx':
            return `${match} (a model of the Jetson computer)`;
          case 'nvidia':
            return `${match} (a company that makes computer chips)`;
          case 'vslam':
            return `${match} (Visual Simultaneous Localization and Mapping - how robots see and map)`;
          case 'whisper':
            return `${match} (a system that understands spoken words)`;
          case 'llm':
            return `${match} (Large Language Model - AI that understands and generates text)`;
          case 'ai':
            return `${match} (Artificial Intelligence - smart computer systems)`;
          case 'ml':
            return `${match} (Machine Learning - AI that learns from data)`;
          case 'neural network':
            return `${match} (AI modeled after the human brain)`;
          default:
            return match;
        }
      });

      // Add beginner-friendly examples
      adjustedContent = this._addBeginnerExamples(adjustedContent);

      return adjustedContent;
    }
    // For intermediate users, provide moderate detail
    else if (userSkillLevel === 'intermediate') {
      // In a real implementation, this might adjust the level of detail
      return content;
    }
    // For advanced users, potentially add more depth
    else if (userSkillLevel === 'advanced') {
      // In a real implementation, this might add more advanced content
      return content;
    }

    return content;
  }

  /**
   * Adds beginner-friendly examples to content
   * @private
   */
  _addBeginnerExamples(content) {
    // This would add more concrete examples for beginners
    // For now, we'll just return the content as is
    return content;
  }

  /**
   * Adjusts difficulty rating based on user skill level
   * @private
   */
  _adjustDifficulty(moduleDifficulty, userSkillLevel, requiredSkillLevel) {
    // If user skill level is lower than required, increase perceived difficulty
    if (userSkillLevel === 'beginner' && requiredSkillLevel !== 'beginner') {
      return 'challenging';
    }

    // If user skill level matches or exceeds required, difficulty is appropriate
    return moduleDifficulty;
  }

  /**
   * Highlights content relevant to user interests
   * @private
   */
  _highlightRelevantContent(content, interests) {
    if (!content || !Array.isArray(interests)) return content;

    let highlightedContent = content;
    for (const interest of interests) {
      // Simple highlighting by adding emphasis around matching terms
      const regex = new RegExp(`\\b(${interest})\\b`, 'gi');
      highlightedContent = highlightedContent.replace(regex, '**$1**');
    }

    return highlightedContent;
  }

  /**
   * Adjusts examples based on user background
   * @private
   */
  _adjustExamplesByBackground(content, background) {
    if (!content || !background) return content;

    // In a real implementation, this would customize examples based on background
    // For example, if background is "hardware", emphasize hardware examples
    return content;
  }

  /**
   * Generates personalized recommendations for a module
   * @private
   */
  _generateRecommendations(module, userProfile) {
    const recommendations = [];

    // Add prerequisite recommendations
    if (module.prerequisites && module.prerequisites.length > 0) {
      recommendations.push({
        type: 'prerequisite',
        message: 'It is recommended to complete the following modules first:',
        modules: module.prerequisites
      });
    }

    // Add related module recommendations based on user interests
    if (userProfile.interests && userProfile.interests.length > 0) {
      recommendations.push({
        type: 'related',
        message: 'You might also be interested in these related topics:',
        modules: userProfile.interests.slice(0, 2).map(interest =>
          `module-about-${interest.replace(/\s+/g, '-').toLowerCase()}`
        )
      });
    }

    return recommendations;
  }

  /**
   * Sorts modules by prerequisites to ensure proper learning order
   * @private
   */
  _sortModulesByPrerequisites(modules) {
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
        throw new Error('Circular dependency detected in module prerequisites');
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
   * Calculates a user's readiness for a specific module
   * @param {LearningModule} module - The module to assess readiness for
   * @param {UserProfile} userProfile - User's profile
   * @returns {Object} Readiness assessment
   */
  async assessReadiness(module, userProfile) {
    if (!module || !userProfile) {
      return { ready: false, score: 0, reasons: ['Missing required data'] };
    }

    let score = 0;
    const reasons = [];

    // Check skill level match
    if (userProfile.skillLevel) {
      if (module.requiredSkillLevel === 'beginner') {
        score += userProfile.skillLevel === 'beginner' ? 100 :
                 userProfile.skillLevel === 'intermediate' ? 100 :
                 userProfile.skillLevel === 'advanced' ? 100 : 0;
      } else if (module.requiredSkillLevel === 'intermediate') {
        score += userProfile.skillLevel === 'intermediate' ? 100 :
                 userProfile.skillLevel === 'advanced' ? 100 :
                 userProfile.skillLevel === 'beginner' ? 0 : 50;
      } else if (module.requiredSkillLevel === 'advanced') {
        score += userProfile.skillLevel === 'advanced' ? 100 :
                 userProfile.skillLevel === 'intermediate' ? 50 : 0;
      }
    }

    // Check if prerequisites are met
    if (module.prerequisites && module.prerequisites.length > 0) {
      const completedPrerequisites = module.prerequisites.filter(prereqId =>
        userProfile.progress && userProfile.progress[prereqId] === 100
      );

      const prerequisiteScore = completedPrerequisites.length / module.prerequisites.length * 100;
      score = (score * 0.7) + (prerequisiteScore * 0.3); // Weight prerequisites less heavily
    }

    // Check interest alignment
    if (userProfile.interests && userProfile.interests.length > 0 && module.title) {
      const interestMatch = userProfile.interests.some(interest =>
        module.title.toLowerCase().includes(interest.toLowerCase()) ||
        module.description.toLowerCase().includes(interest.toLowerCase())
      );

      if (interestMatch) {
        score += 10; // Small boost for interest alignment
      }
    }

    // Cap the score at 100
    score = Math.min(100, score);

    // Determine readiness based on score
    const ready = score >= 70;

    if (ready) {
      reasons.push('You meet the skill requirements for this module');
    } else {
      reasons.push('Consider building foundational skills before starting this module');
    }

    if (module.prerequisites && module.prerequisites.length > 0) {
      const completedCount = module.prerequisites.filter(prereqId =>
        userProfile.progress && userProfile.progress[prereqId] === 100
      ).length;
      reasons.push(`You have completed ${completedCount} of ${module.prerequisites.length} prerequisites`);
    }

    return {
      ready,
      score: Math.round(score),
      reasons
    };
  }

  /**
   * Updates personalization preferences for a user
   * @param {UserProfile} userProfile - User's profile
   * @param {Object} preferences - New personalization preferences
   * @returns {UserProfile} Updated user profile
   */
  async updatePersonalizationPreferences(userProfile, preferences) {
    if (!userProfile || !preferences) {
      return userProfile;
    }

    // Update preferences in the user profile
    Object.keys(preferences).forEach(key => {
      userProfile.personalizationPreferences[key] = preferences[key];
    });

    userProfile.updatedAt = new Date().toISOString();
    return userProfile;
  }

  /**
   * Personalizes specific content type based on user profile
   * @param {any} content - Content to personalize
   * @param {UserProfile} userProfile - User's profile with preferences
   * @param {string} contentType - Type of content ('text', 'code', 'exercises', etc.)
   * @returns {any} Personalized content
   */
  async personalizeContent(content, userProfile, contentType) {
    if (!content || !userProfile) {
      return content;
    }

    switch (contentType) {
      case 'content':
      case 'text':
        return this._adjustContentBySkillLevel(content, userProfile.skillLevel);
      case 'code':
      case 'codeExamples':
        return this._personalizeCodeContent(content, userProfile);
      case 'exercises':
      case 'exercises':
        return this._personalizeExerciseContent(content, userProfile);
      case 'title':
      case 'description':
        return this._adjustContentBySkillLevel(content, userProfile.skillLevel);
      default:
        // For unknown content types, apply general personalization
        return this._adjustContentBySkillLevel(content, userProfile.skillLevel);
    }
  }

  /**
   * Personalizes code content based on user profile
   * @private
   */
  _personalizeCodeContent(codeContent, userProfile) {
    if (!codeContent || !userProfile) return codeContent;

    // For beginners, add more comments and simpler examples
    if (userProfile.skillLevel === 'beginner') {
      // This would add explanatory comments to code examples
      // For now, we'll return the content as is
      return codeContent;
    }

    return codeContent;
  }

  /**
   * Personalizes exercise content based on user profile
   * @private
   */
  _personalizeExerciseContent(exerciseContent, userProfile) {
    if (!exerciseContent || !userProfile) return exerciseContent;

    // Adjust exercise difficulty based on user skill level
    if (userProfile.skillLevel === 'beginner') {
      // This would simplify exercises for beginners
      // For now, we'll return the content as is
      return exerciseContent;
    } else if (userProfile.skillLevel === 'advanced') {
      // This would add more challenging exercises
      // For now, we'll return the content as is
      return exerciseContent;
    }

    return exerciseContent;
  }

  /**
   * Adjusts content difficulty adaptively based on user performance
   * @param {string} content - Original content
   * @param {UserProfile} userProfile - User's profile with performance data
   * @param {string} moduleId - Module ID for context
   * @returns {string} Adjusted content
   */
  async adaptContentDifficulty(content, userProfile, moduleId) {
    if (!content || !userProfile) {
      return content;
    }

    // Calculate user's performance for this module or similar modules
    const performanceScore = await this._calculatePerformanceScore(userProfile, moduleId);

    // Adjust content based on performance
    if (performanceScore < 30) {
      // User is struggling, simplify content
      return this._simplifyContent(content);
    } else if (performanceScore > 80) {
      // User is doing well, add more challenges
      return this._enhanceContent(content);
    }

    // Performance is in the middle range, return as is
    return content;
  }

  /**
   * Calculates performance score based on user's progress and interactions
   * @private
   */
  async _calculatePerformanceScore(userProfile, moduleId) {
    if (!userProfile.progress || !moduleId) {
      return 50; // Default neutral score
    }

    // Get progress for this specific module
    const moduleProgress = userProfile.progress[moduleId];
    if (moduleProgress === undefined) {
      return 50; // Default neutral score
    }

    // Calculate performance based on progress and other factors
    // This is a simplified approach - a real system would consider more factors
    let score = moduleProgress; // Start with progress percentage

    // If user completed the module quickly, they might be advanced
    if (moduleProgress === 100 && userProfile.completionTime && userProfile.completionTime[moduleId]) {
      // Adjust score based on completion time if implemented
    }

    // Ensure score is between 0 and 100
    return Math.max(0, Math.min(100, score));
  }

  /**
   * Simplifies content for users who are struggling
   * @private
   */
  _simplifyContent(content) {
    if (!content) return content;

    // Add more explanations for complex concepts
    let simplifiedContent = content;

    // For struggling users, add even more explanatory text
    simplifiedContent = simplifiedContent.replace(/\b(algorithm|framework|library|dependency|configuration|simulation|perception|planning|execution|navigation|controller|actuator|sensor|ros|gazebo|jetson|vslam|whisper|llm|ai|ml|neural\s+network)\b/gi, (match) => {
      const lowerMatch = match.toLowerCase();
      switch(lowerMatch) {
        case 'algorithm':
          return `${match} (think of it as a recipe for solving a problem - it's a list of steps to follow)`;
        case 'framework':
          return `${match} (like a toolkit that gives you pre-built pieces to make building easier)`;
        case 'library':
          return `${match} (a collection of code that others wrote that you can use)`;
        case 'dependency':
          return `${match} (something your code needs to borrow from another place to work)`;
        case 'configuration':
          return `${match} (settings that tell a program how to behave)`;
        case 'simulation':
          return `${match} (like a video game that acts like the real world for testing)`;
        case 'perception':
          return `${match} (how a robot uses its "eyes" and "ears" to understand the world)`;
        case 'planning':
          return `${match} (the robot thinking about what to do next)`;
        case 'execution':
          return `${match} (the robot actually doing what it planned)`;
        case 'navigation':
          return `${match} (how a robot finds its way around, like using GPS)`;
        case 'controller':
          return `${match} (the brain of the robot that decides what to do)`;
        case 'actuator':
          return `${match} (the muscles of the robot that make it move)`;
        case 'sensor':
          return `${match} (the senses of the robot like cameras or touch sensors)`;
        case 'ros':
          return `${match} (Robot Operating System - think of it as Windows but for robots)`;
        case 'gazebo':
          return `${match} (a virtual world where you can test robots safely)`;
        case 'jetson':
          return `${match} (a powerful computer that fits in your hand, made for AI)`;
        case 'vslam':
          return `${match} (how robots see the world and remember where things are)`;
        case 'whisper':
          return `${match} (a system that can understand what people say)`;
        case 'llm':
          return `${match} (a smart computer program that understands and writes text)`;
        case 'ai':
          return `${match} (Artificial Intelligence - computer programs that can think)`;
        case 'ml':
          return `${match} (Machine Learning - how computers learn from examples)`;
        case 'neural network':
          return `${match} (a computer system inspired by how human brains work)`;
        default:
          return match;
      }
    });

    return simplifiedContent;
  }

  /**
   * Enhances content for users who are doing well
   * @private
   */
  _enhanceContent(content) {
    if (!content) return content;

    // For advanced users, add more depth and challenges
    let enhancedContent = content;

    // Add more technical depth
    enhancedContent = enhancedContent.replace(/\b(implementation|architecture|optimization|scalability|performance)\b/gi, (match) => {
      const lowerMatch = match.toLowerCase();
      switch(lowerMatch) {
        case 'implementation':
          return `${match} (the specific way the solution is coded and deployed)`;
        case 'architecture':
          return `${match} (the high-level design and structure of the system)`;
        case 'optimization':
          return `${match} (techniques to make the system run faster or more efficiently)`;
        case 'scalability':
          return `${match} (how well the system handles growing amounts of work)`;
        case 'performance':
          return `${match} (measurable characteristics of speed, responsiveness, and efficiency)`;
        default:
          return match;
      }
    });

    // Add advanced concepts or challenges
    enhancedContent += '\n\n> **Advanced Challenge**: Consider how this concept might be extended or optimized in a production environment.';

    return enhancedContent;
  }
}

export { PersonalizationService };