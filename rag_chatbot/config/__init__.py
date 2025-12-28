"""
Configuration for the Educational RAG Chatbot
Handles application settings and environment configuration.
"""
import os
from typing import Optional


class Config:
    """
    Configuration class for the Educational RAG Chatbot.
    """

    # Content configuration
    BOOK_CONTENT_PATH: str = os.getenv("BOOK_CONTENT_PATH", "./book_content")

    # RAG configuration
    MAX_RETRIEVAL_RESULTS: int = int(os.getenv("MAX_RETRIEVAL_RESULTS", "5"))
    SIMILARITY_THRESHOLD: float = float(os.getenv("SIMILARITY_THRESHOLD", "0.5"))
    EMBEDDING_DIM: int = int(os.getenv("EMBEDDING_DIM", "1024"))

    # Cohere configuration
    COHERE_API_KEY: Optional[str] = os.getenv("COHERE_API_KEY")
    COHERE_EMBED_MODEL: str = os.getenv("COHERE_EMBED_MODEL", "embed-english-v3.0")
    COHERE_INPUT_TYPE: str = os.getenv("COHERE_INPUT_TYPE", "search_document")

    # Qdrant configuration
    QDRANT_URL: Optional[str] = os.getenv("QDRANT_URL")
    QDRANT_HOST: str = os.getenv("QDRANT_HOST", "localhost")
    QDRANT_PORT: int = int(os.getenv("QDRANT_PORT", "6333"))
    QDRANT_API_KEY: Optional[str] = os.getenv("QDRANT_API_KEY")
    QDRANT_COLLECTION_NAME: str = os.getenv("QDRANT_COLLECTION_NAME", "educational_content")

    # Database configuration
    DATABASE_URL: Optional[str] = os.getenv("DATABASE_URL")

    # Application settings
    DEBUG: bool = os.getenv("DEBUG", "false").lower() == "true"
    LOG_LEVEL: str = os.getenv("LOG_LEVEL", "info")

    # Educational-specific settings
    EDUCATIONAL_MODE: bool = True  # Always True for this application
    ALLOW_SELECTED_TEXT_QUERIES: bool = True  # Allow "answer from selected text only" functionality

    @classmethod
    def validate_config(cls) -> bool:
        """
        Validate that required configuration values are present.

        Returns:
            True if configuration is valid, False otherwise
        """
        required_fields = []

        # Check if Cohere API key is provided
        if not cls.COHERE_API_KEY:
            print("Warning: COHERE_API_KEY is not set. Embedding functionality may not work.")

        # Check if Qdrant is properly configured
        if not cls.QDRANT_URL and not cls.QDRANT_API_KEY:
            print("Warning: QDRANT_URL or QDRANT_API_KEY is not set. Vector database functionality may not work.")

        # Validate ranges for numeric values
        if cls.MAX_RETRIEVAL_RESULTS <= 0 or cls.MAX_RETRIEVAL_RESULTS > 50:
            print("Warning: MAX_RETRIEVAL_RESULTS should be between 1 and 50.")
            return False

        if cls.SIMILARITY_THRESHOLD < 0.0 or cls.SIMILARITY_THRESHOLD > 1.0:
            print("Warning: SIMILARITY_THRESHOLD should be between 0.0 and 1.0.")
            return False

        if cls.EMBEDDING_DIM <= 0:
            print("Warning: EMBEDDING_DIM should be greater than 0.")
            return False

        return True


# Initialize configuration
config = Config()


def get_config() -> Config:
    """
    Get the application configuration.

    Returns:
        Config instance
    """
    return config