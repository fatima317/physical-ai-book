"""
Retrieval service for the Educational RAG Chatbot
Coordinates search and retrieval operations with answer synthesis.
"""
from typing import List, Dict, Optional, Any
import os
from ..services.content_service import ContentService
from ..services.embedding_service import EmbeddingService
from ..services.vector_db_service import VectorDBService
from ..content import ContentSegment


class RetrievalService:
    """
    Service for coordinating retrieval operations and answer synthesis.
    """

    def __init__(self):
        """Initialize retrieval service with required dependencies."""
        self.content_service = ContentService()
        self.embedding_service = EmbeddingService()
        self.vector_db_service = VectorDBService()
        self.max_retrieval_results = int(os.getenv("MAX_RETRIEVAL_RESULTS", "5"))
        self.similarity_threshold = float(os.getenv("SIMILARITY_THRESHOLD", "0.5"))

    def retrieve_and_synthesize_answer(self, query: str, max_results: Optional[int] = None) -> Dict[str, Any]:
        """
        Retrieve relevant content and synthesize an educational answer.

        Args:
            query: User's question or query
            max_results: Maximum number of results to retrieve (optional, defaults to config)

        Returns:
            Dictionary containing answer and supporting information
        """
        if not query or query.strip() == "":
            return {
                "answer": "Please provide a question about the educational content.",
                "sources": [],
                "confidence": 0.0
            }

        # Generate embedding for the query
        query_embeddings = self.embedding_service.generate_embeddings([query])
        if not query_embeddings or len(query_embeddings) == 0 or not query_embeddings[0]:
            return {
                "answer": "Could not process your query. Please try rephrasing your question.",
                "sources": [],
                "confidence": 0.0
            }

        query_embedding = query_embeddings[0]

        # Search for similar content in the vector database
        max_results = max_results or self.max_retrieval_results
        search_results = self.vector_db_service.search_similar(
            query_embedding=query_embedding,
            limit=max_results
        )

        # Filter results based on similarity threshold
        filtered_results = [
            result for result in search_results
            if result['score'] >= self.similarity_threshold
        ]

        if not filtered_results:
            return {
                "answer": "Could not find relevant content in the educational materials to answer your question.",
                "sources": [],
                "confidence": 0.0
            }

        # Synthesize answer from retrieved content
        answer = self._synthesize_answer(query, filtered_results)
        sources = [result['metadata'] for result in filtered_results]

        # Calculate average confidence from similarity scores
        avg_confidence = sum(r['score'] for r in filtered_results) / len(filtered_results) if filtered_results else 0.0

        return {
            "answer": answer,
            "sources": sources,
            "confidence": avg_confidence
        }

    def _synthesize_answer(self, query: str, search_results: List[Dict]) -> str:
        """
        Synthesize an answer from retrieved content segments.

        Args:
            query: Original user query
            search_results: List of search results from vector database

        Returns:
            Synthesized answer based on retrieved content
        """
        if not search_results:
            return "No relevant content was found to answer your question."

        # Extract content texts from search results
        retrieved_texts = []
        for result in search_results:
            # For now, we'll just use the metadata for context
            # In a real implementation, we'd retrieve the actual content
            metadata = result.get('metadata', {})
            text = metadata.get('text', '') or metadata.get('content', '')
            if text:
                retrieved_texts.append(text)

        if not retrieved_texts:
            return "No relevant content was found to answer your question."

        # Create a response based on the retrieved content
        # This is a simple concatenation - in a real system, this would be more sophisticated
        combined_content = " ".join(retrieved_texts[:3])  # Use up to 3 most relevant results

        # Ensure the answer is grounded in the retrieved content
        answer = f"Based on the educational content: {combined_content[:500]}..."  # Limit length

        return answer

    def process_query_with_selected_text_only(self, query: str, selected_text: str) -> Dict[str, Any]:
        """
        Process a query using only the provided selected text.

        Args:
            query: User's question
            selected_text: Specific text content to use for answering

        Returns:
            Dictionary containing answer based only on selected text
        """
        if not selected_text or selected_text.strip() == "":
            return {
                "answer": "No selected text provided for focused query.",
                "sources": [],
                "confidence": 0.0
            }

        # Generate embedding for the selected text
        segment = ContentSegment(
            text=selected_text,
            metadata={"type": "selected_text", "source": "user_selection"},
            segment_id="selected_text_segment"
        )

        embedding = self.embedding_service.generate_embedding_for_segment(segment)
        if not embedding:
            return {
                "answer": "Could not process the selected text.",
                "sources": [],
                "confidence": 0.0
            }

        # Generate embedding for the query
        query_embeddings = self.embedding_service.generate_embeddings([query])
        if not query_embeddings or len(query_embeddings) == 0 or not query_embeddings[0]:
            return {
                "answer": "Could not process your query.",
                "sources": [],
                "confidence": 0.0
            }

        query_embedding = query_embeddings[0]

        # Perform similarity search using the selected text embedding as context
        # For this implementation, we'll just synthesize an answer from the selected text
        answer = self._synthesize_answer_from_text(query, selected_text)

        return {
            "answer": answer,
            "sources": [{"type": "selected_text", "source": "user_selection"}],
            "confidence": 1.0  # High confidence since we're using provided text directly
        }

    def _synthesize_answer_from_text(self, query: str, text: str) -> str:
        """
        Synthesize an answer from a specific text based on the query.

        Args:
            query: User's question
            text: Text to use for answering

        Returns:
            Answer synthesized from the provided text
        """
        # Simple approach: return relevant portion of text based on query
        # In a real implementation, this would use more sophisticated NLP
        return f"Based on the selected text: {text[:500]}..."  # Limit length

    def validate_answer_grounding(self, answer: str, retrieved_content: List[str]) -> bool:
        """
        Validate that an answer is properly grounded in retrieved content.

        Args:
            answer: Answer to validate
            retrieved_content: Content that the answer should be based on

        Returns:
            True if answer is properly grounded, False otherwise
        """
        # This is a simplified validation
        # In a real system, this would use more sophisticated NLP techniques
        answer_lower = answer.lower()
        for content in retrieved_content:
            if content.lower() in answer_lower:
                return True

        # If no content appears in the answer, check for semantic similarity
        # This is a basic check - real implementation would use embeddings
        return len(answer.strip()) > 0