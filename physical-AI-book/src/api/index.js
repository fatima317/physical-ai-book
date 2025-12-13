/**
 * Main API Router
 * Handles API endpoints for the Physical AI & Humanoid Robotics book
 */

// Import required modules
import express from 'express';
import cors from 'cors';
import helmet from 'helmet';
import rateLimit from 'express-rate-limit';

// Import service modules
import { UserProfileService } from '../services/user-profile-service.js';
import { LearningModuleService } from '../services/learning-module-service.js';
import { PersonalizationService } from '../services/personalization-service.js';
import { TranslationService } from '../services/translation-service.js';
import { SubagentService } from '../services/subagent-service.js';
import { SkillService } from '../services/skill-service.js';
import { ValidationService } from '../services/validation-service.js';
import { ProgressTrackingService } from '../services/progress-tracking-service.js';

// Create Express router
const router = express.Router();

// Initialize services
const userProfileService = new UserProfileService();
const learningModuleService = new LearningModuleService();
const personalizationService = new PersonalizationService();
const translationService = new TranslationService();
const subagentService = new SubagentService();
const skillService = new SkillService();
const validationService = new ValidationService();
const progressTrackingService = new ProgressTrackingService(userProfileService, learningModuleService);

// Security middleware
router.use(helmet());
router.use(cors());
router.use(express.json({ limit: '10mb' }));
router.use(express.urlencoded({ extended: true }));

// Rate limiting
const limiter = rateLimit({
  windowMs: 15 * 60 * 1000, // 15 minutes
  max: 100, // Limit each IP to 100 requests per windowMs
  message: 'Too many requests from this IP, please try again later.'
});
router.use(limiter);

// API health check
router.get('/health', (req, res) => {
  res.status(200).json({
    status: 'OK',
    timestamp: new Date().toISOString(),
    service: 'Physical AI Book API'
  });
});

// User Profile API Routes
router.post('/users/profile', async (req, res) => {
  try {
    const profileData = req.body;
    if (!profileData.id) {
      return res.status(400).json({ error: 'User ID is required' });
    }

    const profile = await userProfileService.createProfile(profileData);
    res.status(201).json(profile);
  } catch (error) {
    res.status(500).json({ error: error.message });
  }
});

router.get('/users/profile', async (req, res) => {
  try {
    const userId = req.query.userId;
    if (!userId) {
      return res.status(400).json({ error: 'User ID is required' });
    }

    const profile = await userProfileService.getProfile(userId);
    res.status(200).json(profile);
  } catch (error) {
    res.status(500).json({ error: error.message });
  }
});

router.put('/users/profile', async (req, res) => {
  try {
    const { userId, ...profileData } = req.body;
    if (!userId) {
      return res.status(400).json({ error: 'User ID is required' });
    }

    const updatedProfile = await userProfileService.updateProfile(userId, profileData);
    res.status(200).json(updatedProfile);
  } catch (error) {
    res.status(500).json({ error: error.message });
  }
});

router.delete('/users/profile', async (req, res) => {
  try {
    const userId = req.query.userId;
    if (!userId) {
      return res.status(400).json({ error: 'User ID is required' });
    }

    const deleted = await userProfileService.deleteProfile(userId);
    if (deleted) {
      res.status(200).json({ success: true });
    } else {
      res.status(404).json({ error: 'Profile not found' });
    }
  } catch (error) {
    res.status(500).json({ error: error.message });
  }
});

router.get('/users', async (req, res) => {
  try {
    const { skillLevel, language } = req.query;
    let profiles = await userProfileService.getAllProfiles();

    // Filter by skill level if specified
    if (skillLevel) {
      profiles = await userProfileService.getUsersBySkillLevel(skillLevel);
    }

    // Filter by language if specified
    if (language) {
      profiles = await userProfileService.getUsersByLanguage(language);
    }

    res.status(200).json(profiles);
  } catch (error) {
    res.status(500).json({ error: error.message });
  }
});

// Module Content API Routes
router.get('/modules/:moduleId', async (req, res) => {
  try {
    const { moduleId } = req.params;
    const { userId, language, personalized, translate, sections } = req.query;

    // Get the module
    const module = await learningModuleService.getModule(moduleId);
    if (!module) {
      return res.status(404).json({ error: 'Module not found' });
    }

    let processedModule = { ...module };

    // Apply personalization if requested and user ID is provided
    if ((personalized === 'true' || userId) && userId) {
      const userProfile = await userProfileService.getProfile(userId);
      if (userProfile) {
        processedModule = await personalizationService.personalizeModule(module, userProfile);
      }
    }

    // Apply translation if requested and language is specified
    if ((translate === 'true' || language) && language && language !== 'en') {
      processedModule = await translationService.translateModule(processedModule, language);
    }

    // If specific sections are requested, return only those
    if (sections) {
      const sectionList = sections.split(',');
      const filteredContent = {};

      // This would be more complex in a real implementation
      // For now, we'll just return the full module if sections are requested
      if (sectionList.includes('content')) {
        filteredContent.content = processedModule.content;
      }
      if (sectionList.includes('exercises')) {
        filteredContent.exercises = processedModule.exercises;
      }
      if (sectionList.includes('examples')) {
        filteredContent.examples = processedModule.codeExamples;
      }

      res.status(200).json({
        ...processedModule,
        ...filteredContent
      });
    } else {
      res.status(200).json(processedModule);
    }
  } catch (error) {
    res.status(500).json({ error: error.message });
  }
});

// Dynamic content loading endpoint for specific content types
router.get('/modules/:moduleId/content/:contentType', async (req, res) => {
  try {
    const { moduleId, contentType } = req.params;
    const { userId, language } = req.query;

    // Get the module
    const module = await learningModuleService.getModule(moduleId);
    if (!module) {
      return res.status(404).json({ error: 'Module not found' });
    }

    let content = module[contentType];

    // Apply personalization if user ID is provided
    if (userId) {
      const userProfile = await userProfileService.getProfile(userId);
      if (userProfile) {
        // Personalize specific content type
        content = await personalizationService.personalizeContent(content, userProfile, contentType);
      }
    }

    // Apply translation if language is specified
    if (language && language !== 'en') {
      content = await translationService.translateText(content, language);
    }

    res.status(200).json({ [contentType]: content });
  } catch (error) {
    res.status(500).json({ error: error.message });
  }
});

router.get('/modules', async (req, res) => {
  try {
    const { userId, category } = req.query;

    let modules = await learningModuleService.getAllModules();

    // Filter by category if specified
    if (category) {
      modules = modules.filter(module => module.category.toLowerCase() === category.toLowerCase());
    }

    // Apply personalization if user ID is provided
    if (userId) {
      const userProfile = await userProfileService.getProfile(userId);
      modules = await personalizationService.personalizeModules(modules, userProfile);
    }

    res.status(200).json(modules);
  } catch (error) {
    res.status(500).json({ error: error.message });
  }
});

router.post('/modules', async (req, res) => {
  try {
    const moduleData = req.body;
    if (!moduleData.id) {
      return res.status(400).json({ error: 'Module ID is required' });
    }

    const module = await learningModuleService.createModule(moduleData);
    res.status(201).json(module);
  } catch (error) {
    res.status(500).json({ error: error.message });
  }
});

router.put('/modules/:moduleId', async (req, res) => {
  try {
    const { moduleId } = req.params;
    const moduleData = req.body;

    const updatedModule = await learningModuleService.updateModule(moduleId, moduleData);
    if (!updatedModule) {
      return res.status(404).json({ error: 'Module not found' });
    }

    res.status(200).json(updatedModule);
  } catch (error) {
    res.status(500).json({ error: error.message });
  }
});

router.delete('/modules/:moduleId', async (req, res) => {
  try {
    const { moduleId } = req.params;

    const deleted = await learningModuleService.deleteModule(moduleId);
    if (deleted) {
      res.status(200).json({ success: true });
    } else {
      res.status(404).json({ error: 'Module not found' });
    }
  } catch (error) {
    res.status(500).json({ error: error.message });
  }
});

// Subagent API Routes
router.post('/subagents/:subagentId/execute', async (req, res) => {
  try {
    const { subagentId } = req.params;
    const { input, options } = req.body;

    if (!input) {
      return res.status(400).json({ error: 'Input is required' });
    }

    const result = await subagentService.executeSubagent(subagentId, input, options || {});
    res.status(200).json(result);
  } catch (error) {
    res.status(500).json({ error: error.message });
  }
});

// Skill API Routes
router.post('/skills/:skillId/execute', async (req, res) => {
  try {
    const { skillId } = req.params;
    const { parameters, context } = req.body;

    const result = await skillService.executeSkill(skillId, parameters, context);
    res.status(200).json(result);
  } catch (error) {
    res.status(500).json({ error: error.message });
  }
});

// Validation API Routes
router.post('/validate/hardware', async (req, res) => {
  try {
    const { configurationId, hardwareSpec } = req.body;

    const result = await validationService.validateHardwareConfiguration(configurationId, hardwareSpec);
    res.status(200).json(result);
  } catch (error) {
    res.status(500).json({ error: error.message });
  }
});

// Error handling middleware
router.use((err, req, res, next) => {
  console.error(err.stack);
  res.status(500).json({ error: 'Something went wrong!' });
});

// Progress Tracking API Routes
router.post('/progress/:moduleId', async (req, res) => {
  try {
    const { moduleId } = req.params;
    const { userId, progress, metadata } = req.body;

    if (!userId) {
      return res.status(400).json({ error: 'User ID is required' });
    }

    if (progress === undefined || progress === null) {
      return res.status(400).json({ error: 'Progress is required' });
    }

    const progressRecord = await progressTrackingService.recordProgress(userId, moduleId, progress, metadata);
    res.status(200).json(progressRecord);
  } catch (error) {
    res.status(500).json({ error: error.message });
  }
});

router.get('/progress/:moduleId', async (req, res) => {
  try {
    const { moduleId } = req.params;
    const userId = req.query.userId;

    if (!userId) {
      return res.status(400).json({ error: 'User ID is required' });
    }

    const progress = await progressTrackingService.getProgress(userId, moduleId);
    if (!progress) {
      return res.status(404).json({ error: 'Progress record not found' });
    }

    res.status(200).json(progress);
  } catch (error) {
    res.status(500).json({ error: error.message });
  }
});

router.get('/progress/overall', async (req, res) => {
  try {
    const userId = req.query.userId;

    if (!userId) {
      return res.status(400).json({ error: 'User ID is required' });
    }

    const overallProgress = await progressTrackingService.getOverallProgress(userId);
    res.status(200).json(overallProgress);
  } catch (error) {
    res.status(500).json({ error: error.message });
  }
});

router.get('/progress/path', async (req, res) => {
  try {
    const userId = req.query.userId;

    if (!userId) {
      return res.status(400).json({ error: 'User ID is required' });
    }

    const pathProgress = await progressTrackingService.getLearningPathProgress(userId);
    res.status(200).json(pathProgress);
  } catch (error) {
    res.status(500).json({ error: error.message });
  }
});

router.get('/progress/report', async (req, res) => {
  try {
    const userId = req.query.userId;

    if (!userId) {
      return res.status(400).json({ error: 'User ID is required' });
    }

    const report = await progressTrackingService.generateProgressReport(userId);
    res.status(200).json(report);
  } catch (error) {
    res.status(500).json({ error: error.message });
  }
});

router.delete('/progress/:moduleId', async (req, res) => {
  try {
    const { moduleId } = req.params;
    const userId = req.query.userId;

    if (!userId) {
      return res.status(400).json({ error: 'User ID is required' });
    }

    const success = await progressTrackingService.resetProgress(userId, moduleId);
    if (success) {
      res.status(200).json({ success: true });
    } else {
      res.status(404).json({ error: 'Progress record not found' });
    }
  } catch (error) {
    res.status(500).json({ error: error.message });
  }
});

// 404 handler
router.use('*', (req, res) => {
  res.status(404).json({ error: 'Route not found' });
});

export default router;