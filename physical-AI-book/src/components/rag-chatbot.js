/**
 * Client-side script for embedding RAG chatbot in Docusaurus book
 * Implements anonymous, read-only educational chat functionality
 */

// Configuration for backend API endpoints
const API_CONFIG = {
  BASE_URL: 'http://localhost:8000', // This should be updated to the actual backend URL in production
  ENDPOINTS: {
    QUERY: '/api/v1/query',
    SELECTED_TEXT_QUERY: '/api/v1/query-selected-text'
  }
};

/**
 * Function to call the existing FastAPI /api/v1/query endpoint with user query
 * @param {string} query - The user's query text
 * @returns {Promise<Object>} - Response from the backend
 */
async function callQueryEndpoint(query) {
  const response = await fetch(`${API_CONFIG.BASE_URL}${API_CONFIG.ENDPOINTS.QUERY}`, {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json'
    },
    body: JSON.stringify({
      query: query,
      max_results: 5
    })
  });

  if (!response.ok) {
    throw new Error(`API request failed with status ${response.status}`);
  }

  return await response.json();
}

/**
 * Function to call the existing FastAPI /api/v1/query-selected-text endpoint with selected text
 * @param {string} query - The user's query text
 * @param {string} selectedText - The selected text from the page
 * @returns {Promise<Object>} - Response from the backend
 */
async function callSelectedTextEndpoint(query, selectedText) {
  const response = await fetch(`${API_CONFIG.BASE_URL}${API_CONFIG.ENDPOINTS.SELECTED_TEXT_QUERY}`, {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json'
    },
    body: JSON.stringify({
      query: query,
      selected_text: selectedText
    })
  });

  if (!response.ok) {
    throw new Error(`API request failed with status ${response.status}`);
  }

  return await response.json();
}

/**
 * Function to capture selected text from book pages
 * @returns {string} - The currently selected text, or empty string if no selection
 */
function getSelectedText() {
  const selection = window.getSelection();
  return selection.toString().trim();
}

/**
 * Function to display response text from the backend
 * @param {string} responseText - The response text to display
 */
function displayResponse(responseText) {
  // Find or create the response display element
  let responseElement = document.getElementById('rag-response-display');

  if (!responseElement) {
    // Create response display element if it doesn't exist
    responseElement = document.createElement('div');
    responseElement.id = 'rag-response-display';
    responseElement.style.marginTop = '10px';
    responseElement.style.padding = '10px';
    responseElement.style.border = '1px solid #ccc';
    responseElement.style.backgroundColor = '#f9f9f9';
    responseElement.style.borderRadius = '4px';

    // Find a suitable place to insert the response display
    const mainContent = document.querySelector('main') || document.body;
    mainContent.appendChild(responseElement);
  }

  // Update the content of the response element
  responseElement.innerHTML = `<p><strong>Response:</strong> ${responseText}</p>`;
}

// Export functions for use in Docusaurus
if (typeof module !== 'undefined' && module.exports) {
  module.exports = {
    callQueryEndpoint,
    callSelectedTextEndpoint,
    getSelectedText,
    displayResponse
  };
}