"""
Utility functions for the Educational RAG Chatbot
Provides helper functions for RAG operations and validation.
"""
from typing import List, Dict, Any, Optional
import re


def validate_educational_query(query: str) -> bool:
    """
    Validate that a query is appropriate for educational purposes.

    Args:
        query: The user's query to validate

    Returns:
        True if query is appropriate, False otherwise
    """
    if not query or len(query.strip()) == 0:
        return False

    # Check for minimum length
    if len(query.strip()) < 3:
        return False

    # Check for potentially inappropriate content
    # This is a basic implementation - in a real system, this would be more sophisticated
    query_lower = query.lower()

    # Check for non-educational requests
    non_educational_patterns = [
        r"write.*code",
        r"generate.*code",
        r"create.*program",
        r"build.*application",
        r"make.*software"
    ]

    for pattern in non_educational_patterns:
        if re.search(pattern, query_lower):
            return False

    return True


def format_citation(source_metadata: Dict[str, Any]) -> str:
    """
    Format a citation from source metadata.

    Args:
        source_metadata: Metadata about the source

    Returns:
        Formatted citation string
    """
    if not source_metadata:
        return ""

    # Extract relevant information for citation
    source_path = source_metadata.get("source_path", "Unknown source")
    heading = source_metadata.get("heading_level_1") or source_metadata.get("heading_level_2", "")

    if heading:
        return f"Citation: {source_path} - {heading}"
    else:
        return f"Citation: {source_path}"


def truncate_text(text: str, max_length: int = 200) -> str:
    """
    Truncate text to a maximum length with ellipsis.

    Args:
        text: Text to truncate
        max_length: Maximum length of the text

    Returns:
        Truncated text with ellipsis if needed
    """
    if len(text) <= max_length:
        return text

    return text[:max_length-3] + "..."


def sanitize_response(response: str) -> str:
    """
    Sanitize response to ensure it meets educational standards.

    Args:
        response: Response to sanitize

    Returns:
        Sanitized response
    """
    if not response:
        return response

    # Remove any potentially problematic content
    sanitized = response

    # Ensure response is grounded in educational content
    if not sanitized.lower().startswith("based on"):
        sanitized = f"Based on educational content: {sanitized}"

    return sanitized


def calculate_similarity_score(text1: str, text2: str) -> float:
    """
    Calculate a basic similarity score between two texts.
    Note: This is a simplified implementation. A real system would use embeddings.

    Args:
        text1: First text
        text2: Second text

    Returns:
        Similarity score between 0 and 1
    """
    if not text1 or not text2:
        return 0.0

    # Convert to sets of words for basic similarity
    words1 = set(text1.lower().split())
    words2 = set(text2.lower().split())

    if not words1 and not words2:
        return 1.0
    if not words1 or not words2:
        return 0.0

    # Calculate Jaccard similarity
    intersection = words1.intersection(words2)
    union = words1.union(words2)

    return len(intersection) / len(union)


def validate_answer_grounding(answer: str, source_texts: List[str], threshold: float = 0.3) -> bool:
    """
    Validate that an answer is properly grounded in source texts.

    Args:
        answer: The answer to validate
        source_texts: List of source texts that should support the answer
        threshold: Minimum similarity threshold for validation

    Returns:
        True if answer is properly grounded, False otherwise
    """
    if not answer or not source_texts:
        return False

    # Check if answer has sufficient similarity to any of the source texts
    max_similarity = 0.0
    for source_text in source_texts:
        similarity = calculate_similarity_score(answer, source_text)
        max_similarity = max(max_similarity, similarity)

    return max_similarity >= threshold


def format_educational_response(answer: str, sources: List[Dict[str, Any]]) -> str:
    """
    Format an educational response with proper citations.

    Args:
        answer: The answer to format
        sources: List of sources for the answer

    Returns:
        Formatted response with citations
    """
    if not sources:
        return answer

    formatted_answer = answer

    # Add citations if they're not already present
    if not formatted_answer.lower().startswith("based on"):
        formatted_answer = f"Based on the educational content: {formatted_answer}"

    # Add source citations
    if sources:
        formatted_answer += "\n\nSources:"
        for i, source in enumerate(sources[:3]):  # Limit to first 3 sources
            citation = format_citation(source)
            if citation:
                formatted_answer += f"\n- {citation}"

    return formatted_answer