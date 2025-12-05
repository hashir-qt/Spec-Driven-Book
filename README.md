# Physical AI Humanoid Robotics Book with RAG Chatbot

## Project Overview

This project aims to create a comprehensive educational platform that includes:
1.  A **Static Documentation Site** (Docusaurus) with course content on Physical AI and Humanoid Robotics.
2.  An **Embedded RAG Chatbot** that provides AI-powered assistance by answering questions based on the book's content.

## Prerequisites

To set up and run this project, you will need the following installed on your system:

*   **Node.js**: Version 18+ (LTS recommended)
*   **Python**: Version 3.12+
*   **Git**: Version control system
*   **npm**: Node Package Manager (comes with Node.js)
*   **pip**: Python Package Installer (comes with Python)

## Setup Instructions

Follow these steps to get the project running locally:

1.  **Clone the Repository:**
    ```bash
    git clone https://github.com/your-username/Spec-Driven-Book.git
    cd Spec-Driven-Book
    ```

2.  **Frontend Setup (Docusaurus):**
    Navigate to the `docs` directory, install dependencies, and create a `.env` file.
    ```bash
    cd docs
    npm install
    cp .env.example .env
    # No environment variables needed for frontend currently, but keep .env file for consistency.
    cd .. # Go back to root directory
    ```

3.  **Backend Setup (FastAPI):**
    Navigate to the `backend` directory, create a virtual environment, activate it, install dependencies, and create a `.env` file.
    ```bash
    cd backend
    python -m venv .venv
    # On Windows:
    .venv\Scripts\activate
    # On macOS/Linux:
    # source .venv/bin/activate
    pip install -r requirements.txt # We will create this in the next step
    cp .env.example .env
    # Edit .env and add your GEMINI_API_KEY, GEMINI_BASE_URL, QDRANT_URL, QDRANT_API_KEY, NEON_DATABASE_URL, JWT_SECRET_KEY
    cd .. # Go back to root directory
    ```

4.  **Install Backend Dependencies and Verify Connections:**
    Make sure your virtual environment is active (from step 3).
    ```bash
    # Install dependencies required for the connection tests
    .venv\Scripts\pip.exe install openai qdrant-client "psycopg2-binary" python-dotenv
    # Run connection tests
    .venv\Scripts\python.exe backend/scripts/test_connections.py
    ```
    Ensure all connections (Gemini, Qdrant, Neon Postgres) report "✅ Successful". If not, double-check your `.env` file and API keys/URLs.

## Available Scripts

### Frontend (Docusaurus)

From the `docs` directory:

*   `npm start`: Starts the development server.
*   `npm run build`: Builds the static files for deployment.
*   `npm run serve`: Serves the built static files.

### Backend (FastAPI)

From the `backend` directory (with virtual environment activated):

*   `uvicorn app.main:app --reload`: Starts the FastAPI development server with auto-reloading.

## Troubleshooting Common Issues

*   **`OPENAI_API_KEY` or `GEMINI_API_KEY` not found:** Ensure you have created a `.env` file (not `.env.example`) in the `backend` directory and that your API key is correctly specified and uncommented.
*   **Connection errors to external services:** Double-check your API keys, base URLs, and database connection strings in your `backend/.env` file. Ensure your network allows connections to these services.
*   **Node.js or Python version issues:** Make sure you have the required versions installed as per the "Prerequisites" section. Use tools like `nvm` (Node Version Manager) or `pyenv` (Python Version Manager) if you need to manage multiple versions.
*   **Dependency installation failures:** Ensure your internet connection is stable. For Python, check if your virtual environment is activated. For Node.js, try clearing npm cache (`npm cache clean --force`).
