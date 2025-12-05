# Tasks: Initial Setup

**Input**: Design documents from `/specs/initial-setup/`
**Prerequisites**: plan.md, spec.md

## Phase 1: User Story 1 - Run the Docusaurus Frontend (Priority: P1) 🎯 MVP

**Goal**: As a developer, I want to run the Docusaurus frontend to see the initial book structure.

**Independent Test**: The Docusaurus development server starts successfully and the default page is accessible in a web browser.

### Implementation for User Story 1

-   [ ] T001 [US1] Navigate to the `docs` directory.
-   [ ] T002 [US1] Run `npm start` to start the Docusaurus development server.
-   [ ] T003 [US1] Open `http://localhost:3000` in a web browser to verify the welcome page is displayed.

## Phase 2: User Story 2 - Run the FastAPI Backend (Priority: P1)

**Goal**: As a developer, I want to run the FastAPI backend to verify the basic API is working.

**Independent Test**: The FastAPI development server starts successfully and the root endpoint is accessible.

### Implementation for User Story 2

-   [ ] T004 [US2] Navigate to the `backend` directory.
-   [ ] T005 [US2] Run `uvicorn main:app --reload` to start the FastAPI development server.
-   [ ] T006 [US2] Open `http://localhost:8000` in a web browser to verify the `{"Hello": "World"}` JSON response is displayed.
