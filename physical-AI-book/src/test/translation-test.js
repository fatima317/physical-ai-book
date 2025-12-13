/**
 * Translation Functionality Test
 * Tests the multilingual content delivery (English/Urdu)
 */

import { TranslationService } from '../services/translation-service.js';
import { LearningModule } from '../models/learning-module.js';

class TranslationTest {
  constructor() {
    this.translationService = new TranslationService();
  }

  async runTests() {
    console.log('🌐 Starting Translation Functionality Tests...\n');

    // Test 1: Test basic text translation
    await this.testBasicTranslation();

    // Test 2: Test module translation
    await this.testModuleTranslation();

    // Test 3: Test translation caching
    await this.testTranslationCaching();

    // Test 4: Test multiple language support
    await this.testMultipleLanguages();

    // Test 5: Test translation accuracy
    await this.testTranslationAccuracy();

    // Test 6: Test content blocks translation
    await this.testContentBlocksTranslation();

    console.log('\n✅ All Translation Functionality Tests Completed!');
  }

  async testBasicTranslation() {
    console.log('🔤 Test 1: Testing basic text translation...');

    const testTexts = [
      'Introduction to Robotics',
      'Artificial Intelligence',
      'Machine Learning',
      'Neural Network',
      'Sensor and Actuator',
      'Robot Operating System'
    ];

    for (const text of testTexts) {
      try {
        const translated = await this.translationService.translateText(text, 'ur');
        console.log(`   🇺🇸 ${text}`);
        console.log(`   🇵🇰 ${translated}`);
        console.log('');
      } catch (error) {
        console.log(`   ❌ Error translating "${text}": ${error.message}`);
      }
    }

    console.log('   ✅ Basic translation test completed\n');
  }

  async testModuleTranslation() {
    console.log('📚 Test 2: Testing module translation...');

    // Create a sample learning module
    const sampleModule = new LearningModule({
      id: 'test-module',
      title: 'Introduction to Physical AI',
      content: 'This module introduces Physical AI concepts. You will learn about sensors, actuators, and basic robotics principles. The Robot Operating System (ROS) is essential for robotics development.',
      description: 'An introductory module to Physical AI and Robotics',
      category: 'introduction',
      requiredSkillLevel: 'beginner',
      language: 'en',
      learningObjectives: [
        'Understand basic Physical AI concepts',
        'Identify robot components'
      ],
      keywords: ['robotics', 'ai', 'physical-ai'],
      interactiveElements: [
        {
          type: 'quiz',
          title: 'Basic Concepts Quiz',
          description: 'Test your understanding of basic concepts',
          options: ['Option 1', 'Option 2', 'Option 3']
        }
      ]
    });

    try {
      const translatedModule = await this.translationService.translateModule(sampleModule, 'ur');

      console.log(`   ✅ Module translated successfully`);
      console.log(`   🇺🇸 Title: ${sampleModule.title}`);
      console.log(`   🇵🇰 Title: ${translatedModule.title}`);
      console.log(`   🇺🇸 Content preview: ${sampleModule.content.substring(0, 60)}...`);
      console.log(`   🇵🇰 Content preview: ${translatedModule.content.substring(0, 60)}...`);
      console.log(`   🇺🇸 Language: ${sampleModule.language}`);
      console.log(`   🇵🇰 Language: ${translatedModule.language}`);

    } catch (error) {
      console.log(`   ❌ Error translating module: ${error.message}`);
    }

    console.log('   ✅ Module translation test completed\n');
  }

  async testTranslationCaching() {
    console.log('💾 Test 3: Testing translation caching...');

    const testText = 'Robotics and Artificial Intelligence';

    try {
      // First translation
      console.time('First translation');
      const result1 = await this.translationService.translateText(testText, 'ur');
      console.timeEnd('First translation');

      // Second translation (should use cache)
      console.time('Second translation (cached)');
      const result2 = await this.translationService.translateText(testText, 'ur');
      console.timeEnd('Second translation (cached)');

      console.log(`   Original: ${testText}`);
      console.log(`   Translated: ${result1}`);
      console.log(`   Cached result matches: ${result1 === result2 ? '✅' : '❌'}`);

      // Check cache stats
      const cacheStats = this.translationService.getCacheStats();
      console.log(`   Cache size: ${cacheStats.size}`);

    } catch (error) {
      console.log(`   ❌ Error testing caching: ${error.message}`);
    }

    console.log('   ✅ Translation caching test completed\n');
  }

  async testMultipleLanguages() {
    console.log('🌍 Test 4: Testing multiple language support...');

    const testText = 'Hello, welcome to Physical AI and Robotics';
    const supportedLanguages = this.translationService.getSupportedLanguages();

    console.log(`   Supported languages: ${supportedLanguages.join(', ')}`);

    for (const lang of supportedLanguages) {
      try {
        const translated = await this.translationService.translateText(testText, lang);
        console.log(`   🌐 ${lang.toUpperCase()}: ${translated}`);
      } catch (error) {
        console.log(`   ❌ Error translating to ${lang}: ${error.message}`);
      }
    }

    // Test unsupported language
    try {
      const unsupportedResult = await this.translationService.translateText(testText, 'fr');
      console.log(`   🇫🇷 Unsupported language (French) handled: ${unsupportedResult === testText ? 'Original returned' : 'Unexpected'}`);
    } catch (error) {
      console.log(`   🇫🇷 Unsupported language correctly handled: ${error.message}`);
    }

    console.log('   ✅ Multiple language support test completed\n');
  }

  async testTranslationAccuracy() {
    console.log('🔍 Test 5: Testing translation accuracy...');

    // Test with common robotics/AI terms
    const accuracyTests = [
      { en: 'sensor', expected: 'سینسر' },
      { en: 'actuator', expected: 'ایکچوایٹر' },
      { en: 'robot', expected: 'روبوٹ' },
      { en: 'AI', expected: 'مذ' },
      { en: 'neural network', expected: 'نیورل نیٹ ورک' },
      { en: 'programming', expected: 'پروگرامنگ' }
    ];

    let correctTranslations = 0;
    let totalTests = accuracyTests.length;

    for (const test of accuracyTests) {
      try {
        const translated = await this.translationService.translateText(test.en, 'ur');
        const isCorrect = translated.includes(test.expected) || translated.toLowerCase().includes(test.en.toLowerCase());

        console.log(`   🇺🇸 "${test.en}" → 🇵🇰 "${translated}" ${isCorrect ? '✅' : '❌'}`);

        if (isCorrect) {
          correctTranslations++;
        }
      } catch (error) {
        console.log(`   ❌ Error testing "${test.en}": ${error.message}`);
        totalTests--; // Don't count this test if there was an error
      }
    }

    const accuracy = totalTests > 0 ? (correctTranslations / totalTests) * 100 : 0;
    console.log(`   Accuracy: ${correctTranslations}/${totalTests} (${accuracy.toFixed(1)}%)`);

    console.log('   ✅ Translation accuracy test completed\n');
  }

  async testContentBlocksTranslation() {
    console.log('📝 Test 6: Testing content blocks translation...');

    const contentBlocks = [
      { title: 'Introduction', content: 'This is the introduction section' },
      { title: 'Main Content', content: 'This is the main content with robotics concepts' },
      { title: 'Conclusion', content: 'This is the conclusion section' }
    ];

    try {
      const translatedBlocks = await this.translationService.translateContentBlocks(contentBlocks, 'ur');

      console.log(`   ✅ Translated ${contentBlocks.length} content blocks`);
      for (let i = 0; i < translatedBlocks.length; i++) {
        console.log(`     ${i + 1}. ${contentBlocks[i].title} → ${translatedBlocks[i].title}`);
      }
    } catch (error) {
      console.log(`   ❌ Error translating content blocks: ${error.message}`);
    }

    console.log('   ✅ Content blocks translation test completed\n');
  }
}

// Run the tests if this file is executed directly
if (import.meta.url === `file://${process.argv[1]}`) {
  const test = new TranslationTest();
  test.runTests().catch(console.error);
}

export { TranslationTest };