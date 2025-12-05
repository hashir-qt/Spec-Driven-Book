# Contributing to Physical AI Humanoid Robotics Book with RAG Chatbot

We welcome contributions to this project! To ensure a smooth collaboration, please follow these guidelines.

## Code of Conduct

Please note that this project is released with a Contributor Code of Conduct. By participating in this project, you agree to abide by its terms.

## How to Contribute

1.  **Fork the repository.**
2.  **Clone your forked repository:**
    ```bash
    git clone https://github.com/your-username/Spec-Driven-Book.git
    cd Spec-Driven-Book
    ```
3.  **Set up your development environment** as described in the `README.md`.
4.  **Create a new branch** for your feature or bug fix. Follow the branch naming conventions below.
    ```bash
    git checkout -b feature/your-feature-name
    ```
5.  **Make your changes.** Ensure your code adheres to the Code Style Guide and passes all tests.
6.  **Commit your changes.** Follow the Commit Message Convention below.
7.  **Push your branch** to your forked repository.
    ```bash
    git push origin feature/your-feature-name
    ```
8.  **Create a Pull Request** to the `develop` branch of the original repository.

## Git Workflow

*   **Main branch**: `main` (Production-ready code only)
*   **Develop branch**: `develop` (Integration branch for features)
*   **Feature branches**: `feature/your-feature-name`, `bugfix/issue-description`
*   **Hotfix branches**: `hotfix/critical-issue`

All feature development should happen on feature branches branched off `develop`. Pull Requests should target the `develop` branch.

## Commit Message Convention

We follow the Conventional Commits specification. This helps in generating changelogs, understanding the history, and automating releases.

```
<type>(<scope>): <subject>

<body>

<footer>
```

*   **type**: Must be one of the following:
    *   `feat`: A new feature
    *   `fix`: A bug fix
    *   `docs`: Documentation only changes
    *   `style`: Changes that do not affect the meaning of the code (white-space, formatting, missing semi-colons, etc.)
    *   `refactor`: A code change that neither fixes a bug nor adds a feature
    *   `perf`: A code change that improves performance
    *   `test`: Adding missing tests or correcting existing tests
    *   `chore`: Changes to the build process or auxiliary tools and libraries such as documentation generation

*   **scope**: Optional, but recommended. Describes the part of the codebase affected (e.g., `frontend`, `backend`, `docs`, `chatbot`, `auth`).

*   **subject**: A very brief summary of the change.

*   **body**: Optional. Provides additional contextual information about the code changes.

*   **footer**: Optional. Can contain information like breaking changes, references to issues (e.g., `Closes #123`).

**Examples**:

```
feat(chatbot): implement RAG pipeline with Qdrant vector search
```

```
fix(auth): prevent token leakage in error responses

Previously, 500 errors from auth endpoints would include JWT token
in error payload. Now sanitizing all error responses.

Security-Impact: High
```

## Code Style Guide

*   **Python (Backend)**:
    *   Follow PEP 8.
    *   Use `Black` for auto-formatting.
    *   Use `isort` for sorting imports.
    *   Type hints for all function signatures and return types.

*   **TypeScript/React (Frontend)**:
    *   Follow Airbnb React/TypeScript style guide.
    *   Use `ESLint` for linting.
    *   Use `Prettier` for formatting.
    *   Strict TypeScript mode enabled.

## Testing Requirements

*   **Backend**: Minimum 80% code coverage.
    *   Unit tests for all service functions.
    *   Integration tests for critical API flows.
*   **Frontend**: Minimum 70% component coverage.
    *   Unit tests for utility functions and custom hooks.
    *   Component tests for user interactions (React Testing Library).
    *   E2E tests for critical user journeys (Playwright).

All Pull Requests must pass all CI/CD checks (linting, type-checking, tests) before merging.
