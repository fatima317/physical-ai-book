"""
Vector database service for the Educational RAG Chatbot
Handles vector storage and retrieval operations using Qdrant.
"""
import os
from typing import List, Dict, Optional, Any
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import Distance, VectorParams


class VectorDBService:
    """
    Service for managing vector storage and retrieval using Qdrant.
    """

    def __init__(self, collection_name: str = "educational_content"):
        """Initialize vector database service with Qdrant client."""
        # Get Qdrant configuration from environment variables
        url = os.getenv("QDRANT_URL")  # Use QDRANT_URL for cloud
        api_key = os.getenv("QDRANT_API_KEY")

        # Default to host/port if URL is not provided (for local development)
        if url:
            self.client = QdrantClient(
                url=url,
                api_key=api_key,
                prefer_grpc=True
            )
        else:
            # Fallback to host/port configuration
            host = os.getenv("QDRANT_HOST", "localhost")
            port = int(os.getenv("QDRANT_PORT", "6333"))

            if api_key:
                self.client = QdrantClient(
                    url=host,
                    port=port,
                    api_key=api_key,
                    prefer_grpc=True
                )
            else:
                self.client = QdrantClient(
                    host=host,
                    port=port
                )

        # Get embedding dimension from environment, default to 1024
        self.embedding_dim = int(os.getenv("EMBEDDING_DIM", "1024"))
        self.collection_name = collection_name
        self._ensure_collection_exists()

    def _ensure_collection_exists(self):
        """Ensure the collection exists in Qdrant with proper configuration."""
        try:
            # Try to get collection info to check if it exists
            self.client.get_collection(self.collection_name)
        except Exception:
            # Collection doesn't exist, create it with proper embedding dimension
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=VectorParams(size=self.embedding_dim, distance=Distance.COSINE)
            )

    def store_embedding(self, segment_id: str, embedding: List[float], metadata: Dict[str, Any]) -> bool:
        """
        Store a single embedding in the vector database.

        Args:
            segment_id: Unique identifier for the content segment
            embedding: Embedding vector to store
            metadata: Metadata associated with the segment

        Returns:
            True if successful, False otherwise
        """
        try:
            # Prepare the point for Qdrant
            points = [
                models.PointStruct(
                    id=segment_id,
                    vector=embedding,
                    payload={
                        "segment_id": segment_id,
                        "metadata": metadata
                    }
                )
            ]

            # Upsert the point into the collection
            self.client.upsert(
                collection_name=self.collection_name,
                points=points
            )
            return True
        except Exception as e:
            # Log error but don't expose internal details to user
            print(f"Error storing embedding: {str(e)}")
            return False

    def batch_store_embeddings(self, embeddings_data: List[Dict]) -> bool:
        """
        Store multiple embeddings in the vector database.

        Args:
            embeddings_data: List of dictionaries containing segment_id, embedding, and metadata

        Returns:
            True if successful, False otherwise
        """
        try:
            points = []
            for item in embeddings_data:
                point = models.PointStruct(
                    id=item['segment_id'],
                    vector=item['embedding'],
                    payload={
                        "segment_id": item['segment_id'],
                        "metadata": item['metadata']
                    }
                )
                points.append(point)

            # Upsert all points into the collection
            self.client.upsert(
                collection_name=self.collection_name,
                points=points
            )
            return True
        except Exception as e:
            # Log error but don't expose internal details to user
            print(f"Error batch storing embeddings: {str(e)}")
            return False

    def search_similar(self, query_embedding: List[float], limit: int = 5) -> List[Dict[str, Any]]:
        """
        Search for similar embeddings in the vector database.

        Args:
            query_embedding: Query embedding vector to find similar items
            limit: Maximum number of results to return

        Returns:
            List of similar items with metadata
        """
        try:
            # Perform search in Qdrant
            search_results = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                limit=limit,
                with_payload=True
            )

            results = []
            for hit in search_results:
                result = {
                    'segment_id': hit.id,
                    'score': hit.score,
                    'metadata': hit.payload.get('metadata', {}),
                    'payload': hit.payload
                }
                results.append(result)

            return results
        except Exception as e:
            # Log error but don't expose internal details to user
            print(f"Error searching embeddings: {str(e)}")
            return []

    def get_by_id(self, segment_id: str) -> Optional[Dict[str, Any]]:
        """
        Retrieve a specific embedding by its ID.

        Args:
            segment_id: Unique identifier for the content segment

        Returns:
            Dictionary with segment data or None if not found
        """
        try:
            points = self.client.retrieve(
                collection_name=self.collection_name,
                ids=[segment_id],
                with_payload=True
            )

            if points:
                point = points[0]
                return {
                    'segment_id': point.id,
                    'metadata': point.payload.get('metadata', {}),
                    'payload': point.payload
                }

            return None
        except Exception as e:
            # Log error but don't expose internal details to user
            print(f"Error retrieving embedding by ID: {str(e)}")
            return None

    def delete_by_id(self, segment_id: str) -> bool:
        """
        Delete a specific embedding by its ID.

        Args:
            segment_id: Unique identifier for the content segment

        Returns:
            True if successful, False otherwise
        """
        try:
            self.client.delete(
                collection_name=self.collection_name,
                points_selector=models.PointIdsList(
                    points=[segment_id]
                )
            )
            return True
        except Exception as e:
            # Log error but don't expose internal details to user
            print(f"Error deleting embedding by ID: {str(e)}")
            return False