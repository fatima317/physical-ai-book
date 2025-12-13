/**
 * SkillAssessment Component
 * React component for skill level assessment interface
 */

import React, { useState } from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';

const SkillAssessmentComponent = ({
  onAssessmentComplete,
  initialSkillLevel = 'beginner',
  showResults = true
}) => {
  const [currentQuestion, setCurrentQuestion] = useState(0);
  const [answers, setAnswers] = useState({});
  const [skillLevel, setSkillLevel] = useState(initialSkillLevel);
  const [assessmentComplete, setAssessmentComplete] = useState(false);
  const [showAssessment, setShowAssessment] = useState(true);

  // Sample skill assessment questions
  const assessmentQuestions = [
    {
      id: 1,
      category: 'programming',
      question: 'How comfortable are you with programming concepts?',
      options: [
        { value: 'beginner', label: 'I am just starting to learn programming' },
        { value: 'intermediate', label: 'I have some experience with programming' },
        { value: 'advanced', label: 'I am experienced with programming' }
      ]
    },
    {
      id: 2,
      category: 'robotics',
      question: 'What is your experience with robotics?',
      options: [
        { value: 'beginner', label: 'I have no experience with robotics' },
        { value: 'intermediate', label: 'I have some experience with robotics' },
        { value: 'advanced', label: 'I am experienced with robotics' }
      ]
    },
    {
      id: 3,
      category: 'ai',
      question: 'How familiar are you with Artificial Intelligence concepts?',
      options: [
        { value: 'beginner', label: 'I am new to AI concepts' },
        { value: 'intermediate', label: 'I have some knowledge of AI' },
        { value: 'advanced', label: 'I am familiar with advanced AI concepts' }
      ]
    },
    {
      id: 4,
      category: 'math',
      question: 'What is your comfort level with mathematics (linear algebra, calculus)?',
      options: [
        { value: 'beginner', label: 'Basic math knowledge' },
        { value: 'intermediate', label: 'Some knowledge of calculus and linear algebra' },
        { value: 'advanced', label: 'Strong background in mathematics' }
      ]
    },
    {
      id: 5,
      category: 'development',
      question: 'How experienced are you with development tools and environments?',
      options: [
        { value: 'beginner', label: 'Limited experience with development tools' },
        { value: 'intermediate', label: 'Some experience with IDEs and development tools' },
        { value: 'advanced', label: 'Very experienced with development environments' }
      ]
    }
  ];

  const handleAnswer = (questionId, answer) => {
    setAnswers(prev => ({
      ...prev,
      [questionId]: answer
    }));
  };

  const handleNext = () => {
    if (currentQuestion < assessmentQuestions.length - 1) {
      setCurrentQuestion(currentQuestion + 1);
    } else {
      calculateSkillLevel();
    }
  };

  const handlePrevious = () => {
    if (currentQuestion > 0) {
      setCurrentQuestion(currentQuestion - 1);
    }
  };

  const calculateSkillLevel = () => {
    // Simple algorithm to determine skill level based on answers
    const beginnerCount = Object.values(answers).filter(a => a === 'beginner').length;
    const intermediateCount = Object.values(answers).filter(a => a === 'intermediate').length;
    const advancedCount = Object.values(answers).filter(a => a === 'advanced').length;

    let calculatedLevel = 'beginner';
    if (advancedCount > intermediateCount && advancedCount > beginnerCount) {
      calculatedLevel = 'advanced';
    } else if (intermediateCount > beginnerCount) {
      calculatedLevel = 'intermediate';
    }

    setSkillLevel(calculatedLevel);
    setAssessmentComplete(true);
    setShowAssessment(false);

    if (onAssessmentComplete) {
      onAssessmentComplete(calculatedLevel, answers);
    }
  };

  const resetAssessment = () => {
    setCurrentQuestion(0);
    setAnswers({});
    setAssessmentComplete(false);
    setShowAssessment(true);
  };

  const currentQ = assessmentQuestions[currentQuestion];

  if (!showAssessment) {
    return (
      <div className="skill-assessment-results">
        <h3>Your Assessed Skill Level: {skillLevel.charAt(0).toUpperCase() + skillLevel.slice(1)}</h3>
        <p>Based on your responses, we've determined your skill level in robotics and AI concepts.</p>
        <button onClick={resetAssessment}>Retake Assessment</button>
      </div>
    );
  }

  return (
    <div className="skill-assessment">
      <div className="assessment-progress">
        Question {currentQuestion + 1} of {assessmentQuestions.length}
      </div>
      <div className="assessment-question">
        <h4>{currentQ.question}</h4>
        <div className="assessment-options">
          {currentQ.options.map(option => (
            <div key={option.value} className="assessment-option">
              <input
                type="radio"
                id={`q${currentQ.id}_${option.value}`}
                name={`question_${currentQ.id}`}
                value={option.value}
                checked={answers[currentQ.id] === option.value}
                onChange={() => handleAnswer(currentQ.id, option.value)}
              />
              <label htmlFor={`q${currentQ.id}_${option.value}`}>
                {option.label}
              </label>
            </div>
          ))}
        </div>
      </div>
      <div className="assessment-navigation">
        <button
          onClick={handlePrevious}
          disabled={currentQuestion === 0}
        >
          Previous
        </button>
        <button
          onClick={handleNext}
          disabled={!answers[currentQ.id]}
        >
          {currentQuestion === assessmentQuestions.length - 1 ? 'Finish' : 'Next'}
        </button>
      </div>
    </div>
  );
};

// Wrapper component for browser-only rendering
const SkillAssessment = (props) => {
  return (
    <BrowserOnly>
      {() => <SkillAssessmentComponent {...props} />}
    </BrowserOnly>
  );
};

export default SkillAssessment;