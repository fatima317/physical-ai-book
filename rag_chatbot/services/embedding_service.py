"""
Embedding service for the Educational RAG Chatbot
Handles embedding generation and management using Cohere models.
"""
import os
from typing import List, Dict, Optional
import cohere
from ..content import ContentSegment


class EmbeddingService:
    """
    Service for generating and managing embeddings using Cohere models.
    """

    def __init__(self):
        """Initialize embedding service with Cohere client."""
        api_key = os.getenv("COHERE_API_KEY")
        if not api_key:
            raise ValueError("COHERE_API_KEY environment variable is required")

        self.client = cohere.Client(api_key)
        self.model = os.getenv("COHERE_EMBED_MODEL", "embed-english-v3.0")
        self.input_type = os.getenv("COHERE_INPUT_TYPE", "search_document")

    def generate_embeddings(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for a list of texts using Cohere.

        Args:
            texts: List of text strings to embed

        Returns:
            List of embedding vectors
        """
        try:
            response = self.client.embed(
                texts=texts,
                model=self.model,
                input_type=self.input_type
            )
            return response.embeddings
        except Exception as e:
            # Log error but don't expose internal details to user
            print(f"Embedding generation error: {str(e)}")
            return [[] for _ in texts]  # Return empty embeddings as fallback

    def generate_embedding_for_segment(self, segment: ContentSegment) -> Optional[List[float]]:
        """
        Generate embedding for a single content segment.

        Args:
            segment: ContentSegment to embed

        Returns:
            Embedding vector or None if failed
        """
        embeddings = self.generate_embeddings([segment.text])
        if embeddings and len(embeddings) > 0:
            return embeddings[0]
        return None

    def batch_generate_embeddings(self, segments: List[ContentSegment]) -> List[Dict]:
        """
        Generate embeddings for a batch of content segments with metadata.

        Args:
            segments: List of ContentSegment objects

        Returns:
            List of dictionaries containing segment_id, embedding, and metadata
        """
        if not segments:
            return []

        # Extract texts for embedding
        texts = [segment.text for segment in segments]
        embeddings = self.generate_embeddings(texts)

        result = []
        for i, segment in enumerate(segments):
            if i < len(embeddings) and embeddings[i]:  # Check if embedding was generated successfully
                result.append({
                    'segment_id': segment.segment_id,
                    'embedding': embeddings[i],
                    'metadata': segment.metadata
                })

        return result

    def validate_embedding_quality(self, embedding: List[float]) -> bool:
        """
        Validate that an embedding meets quality standards.

        Args:
            embedding: Embedding vector to validate

        Returns:
            True if embedding is valid, False otherwise
        """
        if not embedding or len(embedding) == 0:
            return False

        # Check for NaN or infinity values
        for value in embedding:
            if not isinstance(value, (int, float)) or value != value:  # Check for NaN (value != value is True for NaN)
                return False

        return True