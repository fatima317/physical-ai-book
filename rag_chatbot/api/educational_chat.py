"""
Educational chat API endpoints for the RAG Chatbot
Handles educational query processing and response delivery.
"""
from fastapi import APIRouter, HTTPException, Depends
from typing import Dict, Any, Optional
from ..models.chat import QueryRequest, QueryResponse, SelectedTextQueryRequest
from ..services.retrieval_service import RetrievalService


router = APIRouter()


def get_retrieval_service() -> RetrievalService:
    """Dependency to get retrieval service instance."""
    return RetrievalService()


@router.post("/query", response_model=QueryResponse)
async def process_educational_query(
    request: QueryRequest,
    retrieval_service: RetrievalService = Depends(get_retrieval_service)
) -> QueryResponse:
    """
    Process an educational query and return a response based on book content.

    Args:
        request: Query request containing the user's question
        retrieval_service: Service to handle retrieval and answer synthesis

    Returns:
        QueryResponse with answer and sources
    """
    try:
        result = retrieval_service.retrieve_and_synthesize_answer(
            query=request.query,
            max_results=request.max_results
        )

        return QueryResponse(
            answer=result["answer"],
            sources=result["sources"],
            confidence=result["confidence"]
        )
    except Exception as e:
        # Log error but don't expose internal details to user
        raise HTTPException(
            status_code=500,
            detail="An error occurred while processing your query. Please try again."
        )


@router.post("/query-selected-text", response_model=QueryResponse)
async def process_selected_text_query(
    request: SelectedTextQueryRequest,
    retrieval_service: RetrievalService = Depends(get_retrieval_service)
) -> QueryResponse:
    """
    Process a query based only on the provided selected text.

    Args:
        request: Query request containing the user's question and selected text
        retrieval_service: Service to handle retrieval and answer synthesis

    Returns:
        QueryResponse with answer and sources
    """
    try:
        result = retrieval_service.process_query_with_selected_text_only(
            query=request.query,
            selected_text=request.selected_text
        )

        return QueryResponse(
            answer=result["answer"],
            sources=result["sources"],
            confidence=result["confidence"]
        )
    except Exception as e:
        # Log error but don't expose internal details to user
        raise HTTPException(
            status_code=500,
            detail="An error occurred while processing your query. Please try again."
        )


@router.get("/health")
async def health_check() -> Dict[str, str]:
    """
    Health check endpoint to verify the service is running.

    Returns:
        Dictionary with status information
    """
    return {"status": "healthy", "service": "educational-rag-chatbot"}


@router.get("/info")
async def service_info() -> Dict[str, str]:
    """
    Information endpoint to provide details about the service.

    Returns:
        Dictionary with service information
    """
    return {
        "name": "Educational RAG Chatbot API",
        "description": "Retrieval-Augmented Generation chatbot for educational robotics book",
        "version": "0.1.0",
        "purpose": "Educational reference assistance for published book content"
    }