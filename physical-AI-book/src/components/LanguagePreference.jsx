/**
 * LanguagePreference Component
 * React component for language preference selection
 */

import React, { useState } from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';

const LanguagePreferenceComponent = ({
  onLanguageSelect,
  initialLanguage = 'en',
  supportedLanguages = [
    { code: 'en', name: 'English', nativeName: 'English' },
    { code: 'ur', name: 'Urdu', nativeName: 'اردو' }
  ]
}) => {
  const [selectedLanguage, setSelectedLanguage] = useState(initialLanguage);

  const handleLanguageChange = (languageCode) => {
    setSelectedLanguage(languageCode);
    if (onLanguageSelect) {
      onLanguageSelect(languageCode);
    }
  };

  return (
    <div className="language-preference">
      <h3>Select Your Preferred Language</h3>
      <div className="language-options">
        {supportedLanguages.map(lang => (
          <div
            key={lang.code}
            className={`language-option ${selectedLanguage === lang.code ? 'selected' : ''}`}
            onClick={() => handleLanguageChange(lang.code)}
          >
            <div className="language-radio">
              <input
                type="radio"
                id={`lang-${lang.code}`}
                name="language"
                value={lang.code}
                checked={selectedLanguage === lang.code}
                onChange={() => handleLanguageChange(lang.code)}
              />
            </div>
            <label htmlFor={`lang-${lang.code}`}>
              <div className="language-name">{lang.name}</div>
              <div className="language-native-name">{lang.nativeName}</div>
            </label>
          </div>
        ))}
      </div>
      <div className="language-info">
        {selectedLanguage === 'en' ? (
          <p>You have selected English as your preferred language. Content will be displayed in English.</p>
        ) : selectedLanguage === 'ur' ? (
          <p>آپ نے اردو کو اپنی ترجیحی زبان کے طور پر منتخب کیا ہے۔ مواد اردو میں ظاہر ہوگا۔</p>
        ) : (
          <p>Selected language: {supportedLanguages.find(lang => lang.code === selectedLanguage)?.name}</p>
        )}
      </div>
    </div>
  );
};

// Wrapper component for browser-only rendering
const LanguagePreference = (props) => {
  return (
    <BrowserOnly>
      {() => <LanguagePreferenceComponent {...props} />}
    </BrowserOnly>
  );
};

export default LanguagePreference;