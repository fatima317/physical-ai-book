"""
Chat-related data models for the Educational RAG Chatbot
Defines request/response models for educational communication.
"""
from pydantic import BaseModel, Field
from typing import List, Dict, Optional


class QueryRequest(BaseModel):
    """
    Request model for educational queries.
    """
    query: str = Field(..., description="The user's question about educational content", min_length=1, max_length=1000)
    max_results: Optional[int] = Field(default=5, description="Maximum number of results to retrieve", ge=1, le=20)


class SelectedTextQueryRequest(BaseModel):
    """
    Request model for queries based on selected text.
    """
    query: str = Field(..., description="The user's question about the selected text", min_length=1, max_length=1000)
    selected_text: str = Field(..., description="The specific text content to focus on", min_length=1, max_length=5000)


class QueryResponse(BaseModel):
    """
    Response model for educational queries.
    """
    answer: str = Field(..., description="The educational answer based on book content")
    sources: List[Dict] = Field(default_factory=list, description="List of sources for the answer")
    confidence: float = Field(default=0.0, description="Confidence score for the answer", ge=0.0, le=1.0)


class HealthResponse(BaseModel):
    """
    Response model for health check endpoint.
    """
    status: str = Field(..., description="Service status")
    service: str = Field(..., description="Service identifier")


class ServiceInfoResponse(BaseModel):
    """
    Response model for service information endpoint.
    """
    name: str = Field(..., description="Name of the service")
    description: str = Field(..., description="Description of the service")
    version: str = Field(..., description="Version of the service")
    purpose: str = Field(..., description="Purpose of the service")