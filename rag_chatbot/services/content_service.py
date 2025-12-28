"""
Content service for the Educational RAG Chatbot
Handles access to book content with read-only operations.
"""
import os
from typing import List, Dict, Optional
from pathlib import Path


class ContentService:
    """
    Service for accessing and processing book content in read-only mode.
    """

    def __init__(self, content_path: Optional[str] = None):
        """
        Initialize content service with path to book content.

        Args:
            content_path: Path to book content directory (defaults to environment variable)
        """
        self.content_path = content_path or os.getenv("BOOK_CONTENT_PATH", "./book_content")
        self.content_dir = Path(self.content_path)

    def get_content_by_path(self, content_path: str) -> Optional[str]:
        """
        Retrieve content from a specific path within the book content directory.

        Args:
            content_path: Relative path to the content file

        Returns:
            Content as string or None if not found
        """
        full_path = self.content_dir / content_path
        if full_path.exists() and full_path.is_file():
            try:
                with open(full_path, 'r', encoding='utf-8') as file:
                    return file.read()
            except Exception:
                # Log error but don't expose internal details to user
                return None
        return None

    def get_all_content_paths(self) -> List[str]:
        """
        Get all content file paths in the book content directory.

        Returns:
            List of relative paths to content files
        """
        paths = []
        if self.content_dir.exists():
            for file_path in self.content_dir.rglob("*"):
                if file_path.is_file():
                    relative_path = file_path.relative_to(self.content_dir)
                    paths.append(str(relative_path))
        return paths

    def search_content_metadata(self, query: str) -> List[Dict[str, str]]:
        """
        Search for content based on metadata (chapters, sections, etc.).

        Args:
            query: Search query for content metadata

        Returns:
            List of content metadata matching the query
        """
        # This would be implemented with actual metadata search
        # For now, return empty list as placeholder
        return []