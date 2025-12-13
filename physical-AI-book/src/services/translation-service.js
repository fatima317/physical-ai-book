/**
 * TranslationService
 * Handles content translation, particularly for Urdu localization
 */

class TranslationService {
  constructor() {
    // In a real implementation, this would connect to translation APIs
    // For now, we'll implement basic translation functionality with Urdu support
    this.supportedLanguages = ['en', 'ur']; // English and Urdu
    this.translationCache = new Map(); // Simple cache for translations
  }

  /**
   * Translates a learning module to the specified language
   * @param {Object} module - The module to translate
   * @param {string} targetLanguage - Target language code (e.g., 'ur' for Urdu)
   * @returns {Object} Translated module
   */
  async translateModule(module, targetLanguage) {
    if (!module || !targetLanguage) {
      return module; // Return original if no translation data
    }

    // Check if target language is supported
    if (!this.supportedLanguages.includes(targetLanguage.toLowerCase())) {
      console.warn(`Language ${targetLanguage} is not supported. Returning original module.`);
      return module;
    }

    // Create a copy of the module to avoid modifying the original
    const translatedModule = { ...module };

    // Translate the content
    translatedModule.content = await this.translateText(
      module.content,
      targetLanguage
    );

    // Translate the title
    translatedModule.title = await this.translateText(
      module.title,
      targetLanguage
    );

    // Translate the description
    translatedModule.description = await this.translateText(
      module.description,
      targetLanguage
    );

    // Translate learning objectives
    if (module.learningObjectives && Array.isArray(module.learningObjectives)) {
      translatedModule.learningObjectives = [];
      for (const objective of module.learningObjectives) {
        translatedModule.learningObjectives.push(
          await this.translateText(objective, targetLanguage)
        );
      }
    }

    // Translate keywords
    if (module.keywords && Array.isArray(module.keywords)) {
      translatedModule.keywords = [];
      for (const keyword of module.keywords) {
        translatedModule.keywords.push(
          await this.translateText(keyword, targetLanguage)
        );
      }
    }

    // Translate interactive elements
    if (module.interactiveElements && Array.isArray(module.interactiveElements)) {
      translatedModule.interactiveElements = [];
      for (const element of module.interactiveElements) {
        const translatedElement = { ...element };

        if (element.title) {
          translatedElement.title = await this.translateText(
            element.title,
            targetLanguage
          );
        }

        if (element.description) {
          translatedElement.description = await this.translateText(
            element.description,
            targetLanguage
          );
        }

        if (element.options && Array.isArray(element.options)) {
          translatedElement.options = [];
          for (const option of element.options) {
            translatedElement.options.push(
              await this.translateText(option, targetLanguage)
            );
          }
        }

        translatedModule.interactiveElements.push(translatedElement);
      }
    }

    // Update language property
    translatedModule.language = targetLanguage.toLowerCase();

    return translatedModule;
  }

  /**
   * Translates plain text to the specified language
   * @param {string} text - Text to translate
   * @param {string} targetLanguage - Target language code
   * @returns {string} Translated text
   */
  async translateText(text, targetLanguage) {
    if (!text || !targetLanguage) {
      return text;
    }

    // Check cache first
    const cacheKey = `${text}_${targetLanguage}`;
    if (this.translationCache.has(cacheKey)) {
      return this.translationCache.get(cacheKey);
    }

    // Check if target language is supported
    if (!this.supportedLanguages.includes(targetLanguage.toLowerCase())) {
      console.warn(`Language ${targetLanguage} is not supported.`);
      return text;
    }

    let translatedText;

    if (targetLanguage.toLowerCase() === 'ur') {
      // Implement Urdu translation
      translatedText = await this._translateToUrdu(text);
    } else {
      // For other languages, return original text
      translatedText = text;
    }

    // Cache the translation
    this.translationCache.set(cacheKey, translatedText);

    return translatedText;
  }

  /**
   * Translates text to Urdu
   * @private
   */
  async _translateToUrdu(text) {
    if (!text) {
      return text;
    }

    // This is a placeholder implementation for Urdu translation
    // In a real implementation, this would connect to a translation API
    // or use a proper translation library

    // For demonstration purposes, we'll implement some basic English to Urdu mappings
    // This is only for common terms in the context of robotics and AI education

    const urduMappings = {
      'introduction': 'تعارف',
      'robotics': 'روبوٹکس',
      'artificial intelligence': 'مصنوعی ذہانت',
      'ai': 'مذ',
      'machine learning': 'مشین لرننگ',
      'deep learning': 'گہرائی سیکھنا',
      'neural network': 'نیورل نیٹ ورک',
      'algorithm': 'الگورتھم',
      'programming': 'پروگرامنگ',
      'code': 'کوڈ',
      'function': 'فنکشن',
      'variable': 'متغیر',
      'loop': 'لوپ',
      'condition': 'حالت',
      'robot': 'روبوٹ',
      'sensor': 'سینسر',
      'actuator': 'ایکچوایٹر',
      'motor': 'موٹر',
      'controller': 'کنٹرولر',
      'navigation': 'نیویگیشن',
      'perception': 'ادراک',
      'planning': 'منصوبہ بندی',
      'execution': 'عمل',
      'simulation': 'سیمولیشن',
      'gazebo': 'گزیبو',
      'ros': 'ROS',
      'ros 2': 'ROS 2',
      'humble': 'Humbe',
      'jetson': 'جیٹسن',
      'nx': 'این ایکس',
      'nvidia': 'این ویڈیا',
      'hardware': 'ہارڈ ویئر',
      'software': 'سافٹ ویئر',
      'development': 'تعمیر',
      'development environment': 'تعمیر کا ماحول',
      'environment': 'ماحول',
      'learning': 'سیکھنا',
      'module': 'ماڈیول',
      'chapter': 'باب',
      'section': 'حصہ',
      'topic': 'موضوع',
      'concept': 'تصور',
      'example': 'مثال',
      'exercise': 'ورکشاپ',
      'practice': 'مشق',
      'skill': 'مہارت',
      'beginner': 'ابتدائی',
      'intermediate': 'درمیانہ',
      'advanced': 'اعلیٰ',
      'difficulty': 'مشکل',
      'level': 'سطح',
      'progress': 'پیشرفت',
      'completion': 'مکمل ہونا',
      'completed': 'مکمل',
      'understanding': 'سمجھ',
      'comprehension': 'ادراک',
      'knowledge': 'علم',
      'expert': 'ماہر',
      'subagent': 'سب ایجنٹ',
      'assistant': 'مددگار',
      'help': 'مدد',
      'guide': 'راہ نما',
      'tutorial': 'سیکھنے کا طریقہ',
      'documentation': 'دستاویزات',
      'reference': 'حوالہ',
      'resource': 'وسائل',
      'tool': 'اوزار',
      'framework': 'چوکھٹ',
      'library': 'لائبریری',
      'package': 'پیکیج',
      'dependency': 'وابستگی',
      'installation': 'تنصیب',
      'setup': 'سیٹ اپ',
      'configuration': 'تشکیل',
      'validation': 'توثیق',
      'verification': 'توثیق',
      'test': 'ٹیسٹ',
      'testing': 'ٹیسٹنگ',
      'debug': 'ڈیبگ',
      'debugging': 'ڈیبگنگ',
      'error': 'غلطی',
      'bug': 'بگ',
      'fix': 'ٹھیک کریں',
      'solution': 'حل',
      'problem': 'مسئلہ',
      'challenge': 'چیلنج',
      'task': 'کام',
      'project': 'پروجیکٹ',
      'application': 'اطلاقیہ',
      'implementation': 'نافذ کرنا',
      'real-world': 'حقیقی دنیا',
      'practical': 'عملی',
      'theory': 'نظریہ',
      'practice': 'مشق',
      'hands-on': 'ہاتھوں سے',
      'interactive': 'مinteractive',
      'personalization': 'ذاتی نوعیت',
      'adaptive': 'مطابقت پذیر',
      'customization': 'اپنی مرضی کے مطابق',
      'preference': 'ترجیح',
      'user': 'صارف',
      'profile': 'پروفائل',
      'language': 'زبان',
      'english': 'انگریزی',
      'urdu': 'اردو',
      'begin': 'شروع کریں',
      'start': 'شروع',
      'continue': 'جاری رکھیں',
      'next': 'اگلا',
      'previous': 'پچھلا',
      'end': 'ختم',
      'finish': 'ختم کریں',
      'complete': 'مکمل',
      'done': 'ہو گیا',
      'success': 'کامیابی',
      'achieve': 'حاصل کریں',
      'goal': 'ہدف',
      'objective': 'مقصد',
      'target': 'ہدف',
      'aim': 'ہدف',
      'purpose': 'مقصد',
      'reason': 'وجہ',
      'why': 'کیوں',
      'how': 'کیسے',
      'what': 'کیا',
      'where': 'کہاں',
      'when': 'کب',
      'who': 'کون',
      'which': 'کون سا',
      'can': 'کر سکتا ہے',
      'will': 'کرے گا',
      'should': 'چاہیے',
      'must': 'ضرور',
      'may': 'شاید',
      'might': 'شاید',
      'could': 'کر سکتا تھا',
      'would': 'کرتا',
      'if': 'اگر',
      'then': 'پھر',
      'else': 'ورنہ',
      'for': 'کے لیے',
      'to': 'کو',
      'of': 'کا',
      'and': 'اور',
      'or': 'یا',
      'not': 'نہیں',
      'is': 'ہے',
      'are': 'ہیں',
      'was': 'تھا',
      'were': 'تھے',
      'be': 'ہونا',
      'been': 'ہوا',
      'being': 'ہوتے ہوئے',
      'have': 'رکھتا ہے',
      'has': 'رکھتا ہے',
      'had': 'رکھا',
      'do': 'کرنا',
      'does': 'کرتا ہے',
      'did': 'کیا',
      'say': 'کہنا',
      'says': 'کہتا ہے',
      'said': 'کہا',
      'get': 'حاصل کرنا',
      'got': 'حاصل کیا',
      'go': 'جانا',
      'goes': 'جاتا ہے',
      'went': 'گیا',
      'make': 'بنانا',
      'makes': 'بنا دیتا ہے',
      'made': 'بنایا',
      'take': 'لینا',
      'takes': 'لیتا ہے',
      'took': 'لیا',
      'see': 'دیکھنا',
      'sees': 'دیکھتا ہے',
      'saw': 'دیکھا',
      'come': 'آنا',
      'comes': 'آتا ہے',
      'came': 'آیا',
      'think': 'سوچنا',
      'thinks': 'سوچتا ہے',
      'thought': 'سوچا',
      'look': 'دیکھنا',
      'looks': 'دیکھتا ہے',
      'looked': 'دیکھا',
      'want': 'چاہنا',
      'wants': 'چاہتا ہے',
      'wanted': 'چاہتا تھا',
      'give': 'دینا',
      'gives': 'دیتا ہے',
      'gave': 'دیا',
      'use': 'استعمال کرنا',
      'uses': 'استعمال کرتا ہے',
      'used': 'استعمال کیا',
      'find': 'ڈھونڈنا',
      'finds': 'ڈھونڈتا ہے',
      'found': 'ڈھونڈ لیا',
      'tell': 'بتانا',
      'tells': 'بتاتا ہے',
      'told': 'بتایا',
      'ask': 'پوچھنا',
      'asks': 'پوچھتا ہے',
      'asked': 'پوچھا',
      'work': 'کام',
      'works': 'کام کرتا ہے',
      'worked': 'کام کیا',
      'seem': 'لگتا ہے',
      'seems': 'لگتا ہے',
      'seemed': 'لگا',
      'feel': 'محسوس کرنا',
      'feels': 'محسوس کرتا ہے',
      'felt': 'محسوس کیا',
      'try': 'کوشش کرنا',
      'tries': 'کوشش کرتا ہے',
      'tried': 'کوشش کی',
      'leave': 'چھوڑنا',
      'leaves': 'چھوڑ دیتا ہے',
      'left': 'چھوڑ دیا',
      'call': 'کال کرنا',
      'calls': 'کال کرتا ہے',
      'called': 'کال کی',
      'need': 'ضرورت ہے',
      'needs': 'ضرورت ہے',
      'needed': 'ضرورت تھی',
      'become': 'بننا',
      'becomes': 'بنتا ہے',
      'became': 'بن گیا',
      'happen': 'ہونا',
      'happens': 'ہوتا ہے',
      'happened': 'ہوا',
      'show': 'دکھانا',
      'shows': 'دکھاتا ہے',
      'showed': 'دکھایا',
      'mean': 'مطلب',
      'means': 'مطلب ہے',
      'meant': 'مطلب تھا',
      'put': 'ڈالنا',
      'set': 'سیٹ کرنا',
      'sets': 'سیٹ کرتا ہے',
      'set': 'سیٹ',
      'help': 'مدد',
      'helps': 'مدد کرتا ہے',
      'helped': 'مدد کی',
      'play': 'کھیلنا',
      'plays': 'کھیلتا ہے',
      'played': 'کھیلا',
      'run': 'چلنا',
      'runs': 'چلتا ہے',
      'ran': 'چلا',
      'move': 'منتقل ہونا',
      'moves': 'منتقل ہوتا ہے',
      'moved': 'منتقل ہو گیا',
      'live': 'رہنا',
      'lives': 'رہتا ہے',
      'lived': 'رہا',
      'believe': 'یقین کرنا',
      'believes': 'یقین رکھتا ہے',
      'believed': 'یقین کیا',
      'hold': 'پکڑنا',
      'holds': 'پکڑتا ہے',
      'held': 'پکڑا',
      'bring': 'لانا',
      'brings': 'لاتا ہے',
      'brought': 'لایا',
      'happen': 'ہونا',
      'happens': 'ہوتا ہے',
      'happened': 'ہوا',
      'must': 'ضرور',
      'should': 'چاہیے',
      'would': 'کرے گا',
      'could': 'کر سکتا ہے',
      'can': 'کر سکتا ہے',
      'will': 'کرے گا',
      'shall': 'کرے گا',
      'may': 'شاید',
      'might': 'شاید',
      'yes': 'ہاں',
      'no': 'نہیں',
      'not': 'نہیں',
      'ok': 'ٹھیک ہے',
      'okay': 'ٹھیک ہے',
      'good': 'اچھا',
      'great': 'عظیم',
      'excellent': 'شاندار',
      'amazing': 'اچھا',
      'wonderful': 'اچھا',
      'fantastic': 'شاندار',
      'perfect': 'کامل',
      'right': 'صحیح',
      'correct': 'صحیح',
      'wrong': 'غلط',
      'bad': 'برا',
      'terrible': 'بُرا',
      'awful': 'بُرا',
      'poor': 'غریب',
      'rich': 'امیر',
      'big': 'بڑا',
      'large': 'بڑا',
      'huge': 'بہت بڑا',
      'small': 'چھوٹا',
      'little': 'چھوٹا',
      'tiny': 'ننھا',
      'short': 'چھوٹا',
      'long': 'لمبا',
      'tall': 'لمبا',
      'high': 'ँچا',
      'low': 'کم',
      'deep': 'گہرا',
      'shallow': 'چھرا',
      'wide': 'چوڑا',
      'narrow': 'تنگ',
      'thick': 'موٹا',
      'thin': 'پتلی',
      'heavy': 'بھاری',
      'light': 'ہلکا',
      'fast': 'تیز',
      'quick': 'تیز',
      'slow': 'سست',
      'late': 'دیر',
      'early': 'جلد',
      'new': 'نیا',
      'old': 'پرانا',
      'young': 'نوجوان',
      'ancient': 'قدیم',
      'modern': 'جدید',
      'future': 'مستقبل',
      'past': 'ماضی',
      'present': 'حال',
      'now': 'اب',
      'then': 'پھر',
      'today': 'آج',
      'tomorrow': 'کل',
      'yesterday': 'کل',
      'morning': 'صبح',
      'afternoon': 'دوپہر',
      'evening': 'شام',
      'night': 'رات',
      'day': 'دن',
      'week': 'ہفتہ',
      'month': 'مہینہ',
      'year': 'سال',
      'time': 'وقت',
      'moment': 'لمحہ',
      'hour': 'گھنٹہ',
      'minute': 'منٹ',
      'second': 'سیکنڈ',
      'first': 'پہلا',
      'second': 'دوسرا',
      'third': 'تیسرا',
      'last': 'آخری',
      'next': 'اگلا',
      'previous': 'پچھلا',
      'one': 'ایک',
      'two': 'دو',
      'three': 'تین',
      'four': 'چار',
      'five': 'پانچ',
      'six': 'چھ',
      'seven': 'سات',
      'eight': 'آٹھ',
      'nine': 'نو',
      'ten': 'دس',
      'eleven': 'گیارہ',
      'twelve': 'بارہ',
      'thirteen': 'تیرہ',
      'fourteen': 'چودہ',
      'fifteen': 'پندرہ',
      'sixteen': 'سولہ',
      'seventeen': 'سترہ',
      'eighteen': 'اٹھارہ',
      'nineteen': 'انیس',
      'twenty': 'بیس',
      'thirty': 'تیس',
      'forty': 'چالیس',
      'fifty': 'پچاس',
      'sixty': 'سٹھ',
      'seventy': 'ستر',
      'eighty': 'اسی',
      'ninety': 'نوے',
      'hundred': 'سو',
      'thousand': 'ہزار',
      'million': 'ملین',
      'billion': 'بلین',
      'more': 'مزید',
      'less': 'کم',
      'many': 'بہت',
      'much': 'بہت',
      'few': 'کم',
      'several': 'کئی',
      'all': 'سب',
      'every': 'ہر',
      'each': 'ہر ایک',
      'some': 'کچھ',
      'any': 'کوئی',
      'none': 'کوئی نہیں',
      'nothing': 'کچھ نہیں',
      'everything': 'سب کچھ',
      'something': 'کچھ',
      'anything': 'کوئی چیز',
      'here': 'یہاں',
      'there': 'وہاں',
      'where': 'کہاں',
      'this': 'یہ',
      'that': 'وہ',
      'these': 'یہ',
      'those': 'وہ',
      'who': 'کون',
      'what': 'کیا',
      'which': 'کون سا',
      'whose': 'جس کا',
      'whom': 'جسے',
      'when': 'کب',
      'where': 'کہاں',
      'why': 'کیوں',
      'how': 'کیسے',
      'i': 'میں',
      'you': 'آپ',
      'he': 'وہ',
      'she': 'وہ',
      'it': 'یہ',
      'we': 'ہم',
      'they': 'وہ',
      'me': 'مجھے',
      'him': 'اسے',
      'her': 'اسے',
      'us': 'ہمیں',
      'them': 'انہیں',
      'my': 'میرا',
      'your': 'آپ کا',
      'his': 'اس کا',
      'her': 'اس کا',
      'its': 'اس کا',
      'our': 'ہمارا',
      'their': 'ان کا',
      'mine': 'میرا',
      'yours': 'آپ کا',
      'hers': 'اس کا',
      'ours': 'ہمارا',
      'theirs': 'ان کا',
      'myself': 'خود',
      'yourself': 'آپ خود',
      'himself': 'خود',
      'herself': 'خود',
      'itself': 'خود',
      'ourselves': 'خود',
      'yourselves': 'آپ خود',
      'themselves': 'خود'
    };

    // Split text into words and translate common terms
    let translated = text;

    // Sort keys by length in descending order to avoid partial replacements
    const sortedKeys = Object.keys(urduMappings).sort((a, b) => b.length - a.length);

    for (const key of sortedKeys) {
      // Use word boundaries to avoid partial matches within words
      const regex = new RegExp(`\\b${key}\\b`, 'gi');
      translated = translated.replace(regex, urduMappings[key]);
    }

    // For more sophisticated translation, we would use a proper translation API
    // This is a very basic implementation for demonstration purposes
    return translated;
  }

  /**
   * Translates content blocks (multiple text elements at once)
   * @param {Array<Object>} contentBlocks - Array of content blocks with text properties
   * @param {string} targetLanguage - Target language code
   * @returns {Array<Object>} Array of translated content blocks
   */
  async translateContentBlocks(contentBlocks, targetLanguage) {
    if (!Array.isArray(contentBlocks) || !targetLanguage) {
      return contentBlocks;
    }

    const translatedBlocks = [];
    for (const block of contentBlocks) {
      const translatedBlock = { ...block };

      // Translate text properties in the block
      for (const [key, value] of Object.entries(translatedBlock)) {
        if (typeof value === 'string') {
          translatedBlock[key] = await this.translateText(value, targetLanguage);
        }
      }

      translatedBlocks.push(translatedBlock);
    }

    return translatedBlocks;
  }

  /**
   * Translates an array of strings
   * @param {Array<string>} texts - Array of strings to translate
   * @param {string} targetLanguage - Target language code
   * @returns {Array<string>} Array of translated strings
   */
  async translateArray(texts, targetLanguage) {
    if (!Array.isArray(texts) || !targetLanguage) {
      return texts;
    }

    const translated = [];
    for (const text of texts) {
      translated.push(await this.translateText(text, targetLanguage));
    }

    return translated;
  }

  /**
   * Gets supported languages
   * @returns {Array<string>} Array of supported language codes
   */
  getSupportedLanguages() {
    return [...this.supportedLanguages];
  }

  /**
   * Checks if a language is supported
   * @param {string} language - Language code to check
   * @returns {boolean} True if language is supported
   */
  isLanguageSupported(language) {
    return this.supportedLanguages.includes(language.toLowerCase());
  }

  /**
   * Pre-translates common educational terms to improve performance
   * @param {string} targetLanguage - Target language code
   */
  async pretranslateCommonTerms(targetLanguage) {
    // This would be used to pre-populate the cache with common terms
    // to improve performance for frequently used terms
    const commonTerms = [
      'introduction', 'robotics', 'artificial intelligence', 'ai', 'machine learning',
      'deep learning', 'neural network', 'algorithm', 'programming', 'code',
      'function', 'variable', 'loop', 'condition', 'robot', 'sensor', 'actuator',
      'motor', 'controller', 'navigation', 'perception', 'planning', 'execution',
      'simulation', 'gazebo', 'ros', 'ros 2', 'humble', 'jetson', 'nx', 'nvidia',
      'hardware', 'software', 'development', 'development environment', 'environment',
      'learning', 'module', 'chapter', 'section', 'topic', 'concept', 'example',
      'exercise', 'practice', 'skill', 'beginner', 'intermediate', 'advanced',
      'difficulty', 'level', 'progress', 'completion', 'completed', 'understanding',
      'comprehension', 'knowledge', 'expert', 'subagent', 'assistant', 'help',
      'guide', 'tutorial', 'documentation', 'reference', 'resource', 'tool',
      'framework', 'library', 'package', 'dependency', 'installation', 'setup',
      'configuration', 'validation', 'verification', 'test', 'testing', 'debug',
      'debugging', 'error', 'bug', 'fix', 'solution', 'problem', 'challenge',
      'task', 'project', 'application', 'implementation', 'real-world', 'practical',
      'theory', 'practice', 'hands-on', 'interactive', 'personalization',
      'adaptive', 'customization', 'preference', 'user', 'profile', 'language',
      'english', 'urdu'
    ];

    for (const term of commonTerms) {
      await this.translateText(term, targetLanguage);
    }
  }

  /**
   * Clears the translation cache
   */
  clearCache() {
    this.translationCache.clear();
  }

  /**
   * Gets cache statistics
   * @returns {Object} Cache statistics
   */
  getCacheStats() {
    return {
      size: this.translationCache.size,
      keys: Array.from(this.translationCache.keys())
    };
  }
}

export { TranslationService };