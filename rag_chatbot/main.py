"""
FastAPI application for the Educational RAG Chatbot
This serves as the main entry point for the backend service.
"""
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

# Import API routers
from rag_chatbot.api.educational_chat import router as educational_chat_router
from rag_chatbot.config import get_config

# Get configuration
config = get_config()

app = FastAPI(
    title="Educational RAG Chatbot API",
    description="Retrieval-Augmented Generation chatbot for educational robotics book",
    version="0.1.0"
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, configure specific origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include API routers
app.include_router(educational_chat_router, prefix="/api/v1", tags=["educational-chat"])

@app.get("/")
def read_root():
    return {"message": "Educational RAG Chatbot API is running"}

@app.get("/health")
def health_check():
    return {"status": "healthy"}