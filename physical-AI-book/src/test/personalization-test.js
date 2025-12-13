/**
 * Personalization Flow Test
 * Tests the personalization functionality with different skill levels
 */

import { UserProfileService } from '../services/user-profile-service.js';
import { LearningModuleService } from '../services/learning-module-service.js';
import { PersonalizationService } from '../services/personalization-service.js';
import { UserProfile } from '../models/user-profile.js';
import { LearningModule } from '../models/learning-module.js';

class PersonalizationTest {
  constructor() {
    this.userProfileService = new UserProfileService();
    this.learningModuleService = new LearningModuleService();
    this.personalizationService = new PersonalizationService();
  }

  async runTests() {
    console.log('🧪 Starting Personalization Flow Tests...\n');

    // Test 1: Create users with different skill levels
    await this.testUserCreation();

    // Test 2: Create learning modules
    await this.testModuleCreation();

    // Test 3: Test personalization for different skill levels
    await this.testPersonalizationBySkillLevel();

    // Test 4: Test adaptive content difficulty
    await this.testAdaptiveDifficulty();

    // Test 5: Test personalized learning paths
    await this.testPersonalizedLearningPath();

    console.log('\n✅ All Personalization Flow Tests Completed!');
  }

  async testUserCreation() {
    console.log('📝 Test 1: Creating users with different skill levels...');

    const users = [
      {
        id: 'user-beginner',
        name: 'Alice Beginner',
        skillLevel: 'beginner',
        languagePreference: 'en',
        interests: ['robotics', 'ai']
      },
      {
        id: 'user-intermediate',
        name: 'Bob Intermediate',
        skillLevel: 'intermediate',
        languagePreference: 'en',
        interests: ['robotics', 'programming']
      },
      {
        id: 'user-advanced',
        name: 'Carol Advanced',
        skillLevel: 'advanced',
        languagePreference: 'en',
        interests: ['ai', 'machine learning']
      }
    ];

    for (const userData of users) {
      try {
        const user = await this.userProfileService.createProfile(userData);
        console.log(`   ✅ Created user: ${user.name} (Skill: ${user.skillLevel})`);
      } catch (error) {
        console.log(`   ❌ Error creating user ${userData.name}: ${error.message}`);
      }
    }

    console.log('   ✅ User creation test completed\n');
  }

  async testModuleCreation() {
    console.log('📚 Test 2: Creating learning modules...');

    const modules = [
      {
        id: 'module-intro',
        title: 'Introduction to Physical AI',
        content: 'This module introduces Physical AI concepts. You will learn about sensors, actuators, and basic robotics principles.',
        category: 'introduction',
        requiredSkillLevel: 'beginner',
        difficulty: 'easy',
        prerequisites: [],
        learningObjectives: [
          'Understand basic Physical AI concepts',
          'Identify robot components'
        ]
      },
      {
        id: 'module-ros',
        title: 'ROS 2 Fundamentals',
        content: 'This module covers Robot Operating System (ROS) 2 basics. You will learn about nodes, topics, services, and actions.',
        category: 'programming',
        requiredSkillLevel: 'intermediate',
        difficulty: 'medium',
        prerequisites: ['module-intro'],
        learningObjectives: [
          'Create ROS 2 nodes',
          'Implement publishers and subscribers'
        ]
      },
      {
        id: 'module-advanced',
        title: 'Advanced Perception Systems',
        content: 'This module dives deep into computer vision and sensor fusion for robotics. You will implement VSLAM algorithms.',
        category: 'perception',
        requiredSkillLevel: 'advanced',
        difficulty: 'hard',
        prerequisites: ['module-intro', 'module-ros'],
        learningObjectives: [
          'Implement VSLAM systems',
          'Fuse sensor data effectively'
        ]
      }
    ];

    for (const moduleData of modules) {
      try {
        const module = await this.learningModuleService.createModule(moduleData);
        console.log(`   ✅ Created module: ${module.title} (Level: ${module.requiredSkillLevel})`);
      } catch (error) {
        console.log(`   ❌ Error creating module ${moduleData.title}: ${error.message}`);
      }
    }

    console.log('   ✅ Module creation test completed\n');
  }

  async testPersonalizationBySkillLevel() {
    console.log('🔄 Test 3: Testing personalization for different skill levels...');

    // Get all modules
    const allModules = await this.learningModuleService.getAllModules();
    const introModule = allModules.find(m => m.id === 'module-intro');

    if (!introModule) {
      console.log('   ❌ Could not find introduction module for testing');
      return;
    }

    // Test personalization for each skill level
    const skillLevels = ['beginner', 'intermediate', 'advanced'];

    for (const skillLevel of skillLevels) {
      // Create a mock user profile for testing
      const mockUserProfile = new UserProfile({
        id: `test-user-${skillLevel}`,
        skillLevel: skillLevel,
        languagePreference: 'en',
        interests: ['robotics']
      });

      try {
        const personalizedModule = await this.personalizationService.personalizeModule(
          introModule,
          mockUserProfile
        );

        console.log(`   ✅ Personalized module for ${skillLevel} user`);
        console.log(`      Original difficulty: ${introModule.difficulty}`);
        console.log(`      Adjusted for ${skillLevel}: ${personalizedModule.difficulty}`);

        // Check if content was adjusted (basic check)
        const contentChanged = personalizedModule.content !== introModule.content;
        console.log(`      Content adjusted: ${contentChanged ? 'Yes' : 'No'}`);

      } catch (error) {
        console.log(`   ❌ Error personalizing for ${skillLevel}: ${error.message}`);
      }
    }

    console.log('   ✅ Personalization by skill level test completed\n');
  }

  async testAdaptiveDifficulty() {
    console.log('⚖️  Test 4: Testing adaptive content difficulty...');

    const testContent = "Understanding neural networks in robotics requires knowledge of algorithms and mathematical concepts.";

    // Test with different user profiles representing different performance levels
    const testProfiles = [
      {
        id: 'struggling-user',
        skillLevel: 'beginner',
        progress: { 'test-module': 20 }, // Low progress = struggling
        interests: ['robotics']
      },
      {
        id: 'advanced-user',
        skillLevel: 'advanced',
        progress: { 'test-module': 95 }, // High progress = advanced
        interests: ['ai', 'ml']
      }
    ];

    for (const profileData of testProfiles) {
      try {
        const profile = await this.userProfileService.createProfile(profileData);
        const adjustedContent = await this.personalizationService.adaptContentDifficulty(
          testContent,
          profile,
          'test-module'
        );

        console.log(`   ✅ Adapted content for ${profileData.id}`);
        console.log(`      Original: ${testContent.substring(0, 50)}...`);
        console.log(`      Adjusted: ${adjustedContent.substring(0, 50)}...`);

      } catch (error) {
        console.log(`   ❌ Error adapting difficulty for ${profileData.id}: ${error.message}`);
      }
    }

    console.log('   ✅ Adaptive difficulty test completed\n');
  }

  async testPersonalizedLearningPath() {
    console.log('🗺️  Test 5: Testing personalized learning paths...');

    // Get all modules
    const allModules = await this.learningModuleService.getAllModules();

    // Test with different user profiles
    const testUsers = [
      { id: 'beginner-user', skillLevel: 'beginner' },
      { id: 'advanced-user', skillLevel: 'advanced' }
    ];

    for (const userData of testUsers) {
      try {
        // Create user profile
        const userProfile = await this.userProfileService.createProfile({
          id: userData.id,
          skillLevel: userData.skillLevel,
          interests: ['robotics', 'ai']
        });

        // Generate personalized learning path
        const learningPath = await this.personalizationService.generatePersonalizedLearningPath(
          allModules,
          userProfile
        );

        console.log(`   ✅ Generated learning path for ${userData.skillLevel} user`);
        console.log(`      Path length: ${learningPath.length} modules`);
        learningPath.forEach((module, index) => {
          console.log(`        ${index + 1}. ${module.title}`);
        });

      } catch (error) {
        console.log(`   ❌ Error generating learning path for ${userData.skillLevel}: ${error.message}`);
      }
    }

    console.log('   ✅ Personalized learning path test completed\n');
  }
}

// Run the tests if this file is executed directly
if (import.meta.url === `file://${process.argv[1]}`) {
  const test = new PersonalizationTest();
  test.runTests().catch(console.error);
}

export { PersonalizationTest };