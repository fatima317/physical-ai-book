"""
Content processing for the Educational RAG Chatbot
Handles content parsing and segmentation for optimal retrieval.
"""
import re
from typing import List, Dict, Tuple
from dataclasses import dataclass


@dataclass
class ContentSegment:
    """Represents a segmented piece of content with metadata."""
    text: str
    metadata: Dict[str, str]
    segment_id: str


class ContentProcessor:
    """
    Processes book content into segments suitable for embedding and retrieval.
    """

    def __init__(self):
        """Initialize content processor."""
        pass

    def segment_content(self, content: str, max_chunk_size: int = 512) -> List[ContentSegment]:
        """
        Split content into segments of appropriate size for embedding.

        Args:
            content: The content to segment
            max_chunk_size: Maximum size of each segment in characters

        Returns:
            List of ContentSegment objects with text and metadata
        """
        if not content:
            return []

        # Split content by paragraphs first
        paragraphs = [p.strip() for p in content.split('\n\n') if p.strip()]

        segments = []
        segment_id_counter = 0

        for paragraph in paragraphs:
            if len(paragraph) <= max_chunk_size:
                # Paragraph fits in a single segment
                segment = ContentSegment(
                    text=paragraph,
                    metadata={"type": "paragraph"},
                    segment_id=f"seg_{segment_id_counter}"
                )
                segments.append(segment)
                segment_id_counter += 1
            else:
                # Paragraph is too long, split into smaller chunks
                sub_chunks = self._split_long_content(paragraph, max_chunk_size)
                for chunk in sub_chunks:
                    segment = ContentSegment(
                        text=chunk,
                        metadata={"type": "chunk"},
                        segment_id=f"seg_{segment_id_counter}"
                    )
                    segments.append(segment)
                    segment_id_counter += 1

        return segments

    def _split_long_content(self, content: str, max_chunk_size: int) -> List[str]:
        """
        Split long content into smaller chunks while preserving sentence boundaries.

        Args:
            content: Content to split
            max_chunk_size: Maximum size for each chunk

        Returns:
            List of content chunks
        """
        sentences = re.split(r'(?<=[.!?])\s+', content)
        chunks = []
        current_chunk = ""

        for sentence in sentences:
            if len(current_chunk) + len(sentence) <= max_chunk_size:
                current_chunk += sentence + " "
            else:
                if current_chunk:
                    chunks.append(current_chunk.strip())
                current_chunk = sentence + " "

        if current_chunk:
            chunks.append(current_chunk.strip())

        # Handle case where a single sentence is longer than max_chunk_size
        final_chunks = []
        for chunk in chunks:
            if len(chunk) <= max_chunk_size:
                final_chunks.append(chunk)
            else:
                # Force split very long chunks by character count
                for i in range(0, len(chunk), max_chunk_size):
                    final_chunks.append(chunk[i:i+max_chunk_size])

        return final_chunks

    def extract_content_metadata(self, content: str, source_path: str) -> Dict[str, str]:
        """
        Extract metadata from content such as chapter, section, etc.

        Args:
            content: The content to extract metadata from
            source_path: Path to the source file

        Returns:
            Dictionary of metadata
        """
        metadata = {
            "source_path": source_path,
            "length": str(len(content)),
            "type": "text"
        }

        # Try to extract chapter/section from content
        # This is a simplified implementation - in a real system, this would be more sophisticated
        lines = content.split('\n')[:5]  # Look at first 5 lines for headings
        for line in lines:
            line = line.strip()
            if line.startswith('#') or line.startswith('##') or line.startswith('###'):
                # Found a heading, likely a section or chapter title
                level = len(line) - len(line.lstrip('#'))
                heading = line.lstrip('#').strip()
                metadata[f"heading_level_{level}"] = heading
                break

        return metadata