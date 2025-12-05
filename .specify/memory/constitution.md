# Project Constitution: Physical AI Humanoid Robotics Book with RAG Chatbot

**Version**: 1.0.0  
**Created**: 2024-12-06  
**Status**: Active  
**Scope**: All features, all phases, all code  

---

## Purpose

This constitution defines **immutable quality standards** that apply to every line of code, every specification, every architectural decision, and every user interaction in this project. These are not guidelines—they are requirements.

This project has two parallel deliverables:
1. **Docusaurus Book** - Educational content on Physical AI and Humanoid Robotics
2. **RAG Chatbot System** - Embedded AI assistant that answers questions about book content

Both must meet professional production standards suitable for academic and industry use.

---

## Core Principles

### 1. Educational Excellence
**Standard**: Content must teach effectively, not just present information.

- Every chapter follows learning science principles (cognitive load theory, scaffolding, worked examples)
- Complex concepts broken into digestible chunks with clear progression
- Code examples are complete, runnable, and annotated
- Technical accuracy verified against official documentation (via Context7 MCP)

**Rationale**: This book trains engineers for Physical AI careers. Poor teaching wastes students' time and damages credibility.

### 2. Production-Ready Architecture
**Standard**: All code must be deployment-ready, not prototype quality.

- Clean separation of concerns (frontend/backend/data layers)
- Environment-based configuration (dev/staging/production)
- Error handling at every API boundary
- Logging, monitoring, and observability built-in from day one
- Security hardened (OWASP Top 10 mitigations)

**Rationale**: Students learn architecture patterns from this codebase. Teaching bad patterns is educational malpractice.

### 3. Developer Experience (DX)
**Standard**: Setup and development must be frictionless.

- Single-command setup: `npm install && npm start` or `poetry install && poetry run dev`
- Clear README with prerequisites, setup steps, and troubleshooting
- Type safety enforced (TypeScript for frontend, Pydantic for backend)
- Comprehensive error messages with actionable solutions
- Hot reload for both frontend and backend during development

**Rationale**: Frustrated developers abandon projects. DX quality directly impacts learning outcomes.

### 4. AI-Augmented Development
**Standard**: Leverage MCP servers and AI tools effectively.

- Context7 MCP server queries official docs before writing integration code
- GitHub MCP server manages commits, branches, and PRs systematically
- Prompts are specific, well-structured, and include success criteria
- AI-generated code always reviewed for correctness and security

**Rationale**: This project demonstrates AI-native development. Use AI tools properly or don't use them.

---

## Quality Standards

### Code Quality

#### Backend (FastAPI)
- **Type Hints**: 100% coverage for function signatures and return types
  - ❌ `def get_user(id): ...`
  - ✅ `def get_user(id: UUID) -> UserSchema: ...`

- **API Design**: RESTful conventions strictly followed
  - Endpoints: `/api/v1/resource` (versioned)
  - Status codes: 200 (success), 201 (created), 400 (client error), 500 (server error)
  - Response format: `{"data": {...}, "error": null}` or `{"data": null, "error": {...}}`

- **Error Handling**: No silent failures
  ```python
  # ❌ Bad
  try:
      user = db.get_user(id)
  except:
      return None
  
  # ✅ Good
  try:
      user = await db.get_user(id)
  except UserNotFoundError as e:
      logger.warning(f"User {id} not found: {e}")
      raise HTTPException(status_code=404, detail=f"User {id} not found")
  except DatabaseError as e:
      logger.error(f"Database error retrieving user {id}: {e}")
      raise HTTPException(status_code=500, detail="Internal server error")
  ```

- **Async Best Practices**: Use `async/await` for I/O operations
  - Database queries: `await db.execute(query)`
  - External APIs: `async with httpx.AsyncClient() as client: ...`
  - No blocking calls in async functions

- **Dependency Injection**: FastAPI's DI system for all shared resources
  ```python
  async def get_chatbot_response(
      message: str,
      db: AsyncSession = Depends(get_db),
      vector_store: QdrantClient = Depends(get_vector_store),
      current_user: User = Depends(get_current_user)
  ) -> ChatResponse:
      ...
  ```

#### Frontend (Docusaurus + React)
- **Component Structure**: Functional components with hooks
  - No class components unless interfacing with legacy libraries
  - Custom hooks for reusable logic (`useAuth`, `useChatbot`, `usePersonalization`)

- **TypeScript**: Strict mode enabled (`strict: true` in tsconfig.json)
  - No `any` types except for legitimate third-party typing gaps
  - Props interfaces defined for all components
  ```typescript
  // ❌ Bad
  function ChatWidget(props: any) { ... }
  
  // ✅ Good
  interface ChatWidgetProps {
    initialMessage?: string;
    onResponse: (response: string) => void;
    userId: string | null;
  }
  function ChatWidget({ initialMessage, onResponse, userId }: ChatWidgetProps) { ... }
  ```

- **State Management**: Context API for global state (auth, theme, personalization)
  - No Redux unless complexity demands it (ChatGPT-style apps rarely need Redux)
  - Local state (`useState`) for component-specific data

- **Performance**: 
  - Code splitting for chatbot component: `React.lazy(() => import('./Chatbot'))`
  - Memoization for expensive renders: `useMemo`, `useCallback`
  - Virtual scrolling for long chat histories (react-window)

- **Accessibility**: WCAG 2.1 AA compliance
  - Semantic HTML (`<button>` not `<div onClick>`)
  - ARIA labels for icon buttons
  - Keyboard navigation for all interactive elements
  - Focus management in modals and dropdowns

#### Database (Neon Postgres + Qdrant)
- **Schema Design**: Normalized structure with clear relationships
  ```sql
  -- Users table with proper constraints
  CREATE TABLE users (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    email VARCHAR(255) UNIQUE NOT NULL,
    hashed_password VARCHAR(255) NOT NULL,
    software_background JSONB,
    hardware_background JSONB,
    created_at TIMESTAMP DEFAULT NOW(),
    updated_at TIMESTAMP DEFAULT NOW()
  );
  
  -- Personalization preferences
  CREATE TABLE user_preferences (
    user_id UUID REFERENCES users(id) ON DELETE CASCADE,
    chapter_id VARCHAR(100),
    personalization_level VARCHAR(50),
    preferred_language VARCHAR(10) DEFAULT 'en',
    PRIMARY KEY (user_id, chapter_id)
  );
  ```

- **Migrations**: Alembic for schema versioning
  - Every schema change requires a migration
  - Migrations are reversible (`upgrade` and `downgrade` functions)
  - Never edit migration files after they're committed

- **Vector Store (Qdrant)**: Proper collection management
  - Collections named semantically: `book_chapters_en`, `book_chapters_ur`
  - Vector dimensions match embedding model (OpenAI text-embedding-3-small: 1536 dimensions)
  - Metadata includes: chapter_id, section_title, content_hash, language

### Security Standards

#### Authentication (Better-auth)
- **Password Requirements**: 
  - Minimum 12 characters
  - Must include: uppercase, lowercase, number, special character
  - Rate limiting: 5 failed attempts → 15-minute lockout

- **Session Management**:
  - JWT tokens with 15-minute expiry (access) + 7-day refresh token
  - HttpOnly cookies for token storage (no localStorage for tokens)
  - CSRF protection enabled for all state-changing operations

- **Authorization**:
  - Role-based access control (RBAC): `user`, `admin`
  - Endpoint protection: `@requires_auth` decorator on all private routes
  - Row-level security: Users can only access their own personalization data

#### API Security
- **Input Validation**: Pydantic models validate all request bodies
  ```python
  class ChatRequest(BaseModel):
      message: str = Field(..., min_length=1, max_length=2000)
      context: Optional[str] = Field(None, max_length=10000)
      
      @validator('message')
      def sanitize_message(cls, v):
          # Remove potential injection attempts
          return v.strip()
  ```

- **Rate Limiting**: 
  - Chatbot endpoint: 20 requests/minute per user
  - Auth endpoints: 5 requests/minute per IP
  - Use `slowapi` library with Redis backend

- **CORS**: Restrict origins in production
  ```python
  app.add_middleware(
      CORSMiddleware,
      allow_origins=["https://yourdomain.com"],  # No wildcards in production
      allow_credentials=True,
      allow_methods=["GET", "POST", "PUT", "DELETE"],
      allow_headers=["*"],
  )
  ```

- **Content Security Policy (CSP)**: Prevent XSS attacks
  ```typescript
  // In Docusaurus config
  headers: {
    "Content-Security-Policy": "default-src 'self'; script-src 'self' 'unsafe-inline'; ..."
  }
  ```

### Testing Standards

#### Coverage Requirements
- **Backend**: Minimum 80% code coverage
  - Unit tests: All service functions
  - Integration tests: Critical API flows (auth, chatbot, personalization)
  - Use `pytest` with `pytest-asyncio` for async tests

- **Frontend**: Minimum 70% component coverage
  - Unit tests: Utility functions, custom hooks
  - Component tests: User interactions (React Testing Library)
  - E2E tests: Critical user journeys (Playwright)

#### Test Structure
```python
# Backend test example
@pytest.mark.asyncio
async def test_chatbot_responds_to_question(
    async_client: AsyncClient,
    mock_vector_store: Mock,
    mock_openai: Mock
):
    """
    Given: A user asks a question about ROS 2 nodes
    When: POST /api/v1/chat with the question
    Then: Response includes relevant chapter excerpt and AI answer
    """
    # Arrange
    mock_vector_store.search.return_value = [
        {"text": "ROS 2 nodes are independent processes...", "chapter": "ros2-fundamentals"}
    ]
    mock_openai.chat.completions.create.return_value = Mock(
        choices=[Mock(message=Mock(content="ROS 2 nodes communicate via topics..."))]
    )
    
    # Act
    response = await async_client.post(
        "/api/v1/chat",
        json={"message": "What are ROS 2 nodes?"}
    )
    
    # Assert
    assert response.status_code == 200
    data = response.json()
    assert "ROS 2 nodes" in data["answer"]
    assert data["source_chapter"] == "ros2-fundamentals"
```

### Documentation Standards

#### Code Documentation
- **Docstrings**: Google style for Python, JSDoc for TypeScript
  ```python
  def generate_personalized_content(
      chapter_content: str,
      user_background: UserBackground,
      personalization_level: str
  ) -> str:
      """Generate personalized chapter content based on user background.
      
      Args:
          chapter_content: Original chapter markdown content
          user_background: User's software/hardware experience levels
          personalization_level: One of ['beginner', 'intermediate', 'advanced']
      
      Returns:
          Personalized markdown content with adjusted examples and explanations
      
      Raises:
          ValueError: If personalization_level is invalid
          OpenAIError: If LLM request fails
      
      Example:
          >>> content = "# ROS 2 Nodes\nNodes are..."
          >>> background = UserBackground(software_exp="intermediate", hardware_exp="beginner")
          >>> personalized = generate_personalized_content(content, background, "intermediate")
      """
  ```

- **API Documentation**: Auto-generated via FastAPI's OpenAPI
  - Endpoint descriptions include examples
  - Request/response schemas documented with field descriptions
  - Error responses documented with example payloads

#### Architecture Documentation
- **ADRs (Architectural Decision Records)**: Required for all significant decisions
  - Why we chose Neon over vanilla Postgres
  - Why we chose Qdrant over Pinecone/Weaviate
  - Why we chose Better-auth over NextAuth
  - Each ADR includes: Context, Decision, Alternatives Considered, Consequences

- **README Files**: Every directory has a README explaining its purpose
  ```
  backend/app/services/
  ├── README.md          # Explains service layer architecture
  ├── chatbot.py         # RAG chatbot logic
  ├── personalization.py # Content adaptation logic
  └── translation.py     # Urdu translation logic
  ```

---

## Performance Standards

### Frontend Performance
- **Lighthouse Scores**: Minimum thresholds
  - Performance: 90+
  - Accessibility: 100
  - Best Practices: 95+
  - SEO: 100

- **Core Web Vitals**:
  - LCP (Largest Contentful Paint): < 2.5s
  - FID (First Input Delay): < 100ms
  - CLS (Cumulative Layout Shift): < 0.1

- **Bundle Size**: 
  - Initial JS bundle: < 200kb (gzipped)
  - Chatbot component (lazy-loaded): < 100kb (gzipped)

### Backend Performance
- **API Response Times**: 95th percentile
  - Simple queries (GET user profile): < 100ms
  - RAG chatbot query: < 2s (includes vector search + LLM inference)
  - Personalization generation: < 5s

- **Database Query Optimization**:
  - All queries use indexes (analyze with `EXPLAIN ANALYZE`)
  - N+1 query prevention (use `selectinload` in SQLAlchemy)
  - Connection pooling: 10 min connections, 20 max connections

- **Caching Strategy**:
  - Chapter content: Redis cache with 1-hour TTL
  - Vector embeddings: Cache in Qdrant collection metadata
  - User preferences: In-memory cache (invalidate on update)

---

## Git and Version Control Standards

### Commit Message Convention
Follow Conventional Commits specification:
```
type(scope): subject

body

footer
```

**Types**: `feat`, `fix`, `docs`, `style`, `refactor`, `test`, `chore`

**Examples**:
```
feat(chatbot): implement RAG pipeline with Qdrant vector search

- Integrate OpenAI embeddings for chapter content
- Add semantic search in Qdrant collection
- Implement context injection for LLM prompts

Closes #12
```

```
fix(auth): prevent token leakage in error responses

Previously, 500 errors from auth endpoints would include JWT token
in error payload. Now sanitizing all error responses.

Security-Impact: High
```

### Branch Strategy
- **Main branch**: Production-ready code only
- **Develop branch**: Integration branch for features
- **Feature branches**: `feature/chatbot-rag`, `feature/better-auth`
- **Hotfix branches**: `hotfix/critical-auth-bug`

**Branch Protection**:
- Main: Requires 1 approval, all tests pass, no force push
- Develop: Requires all tests pass

### Pull Request Requirements
- **PR Title**: Conventional commit format
- **Description Template**:
  ```markdown
  ## What does this PR do?
  [Brief description]
  
  ## How to test?
  [Step-by-step testing instructions]
  
  ## Checklist
  - [ ] Tests added/updated
  - [ ] Documentation updated
  - [ ] No console errors
  - [ ] Lighthouse scores meet standards
  ```

- **Review Process**:
  - AI-generated code: Human must review security and logic
  - Database migrations: Require manual approval
  - Breaking changes: Require team discussion

---

## Deployment Standards

### Environment Configuration
- **Three environments**: Development, Staging, Production
- **Environment Variables**: Never hardcode secrets
  ```python
  # ❌ Bad
  OPENAI_API_KEY = "sk-proj-..."
  
  # ✅ Good
  OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")
  if not OPENAI_API_KEY:
      raise ValueError("OPENAI_API_KEY environment variable not set")
  ```

- **Configuration Management**:
  - Use `.env` files for local development (gitignored)
  - Use Vercel/Railway environment variables for production
  - Document all required env vars in README

### Deployment Checklist
Before deploying to production:
- [ ] All tests pass (backend + frontend)
- [ ] Lighthouse scores meet standards
- [ ] Security audit: `npm audit`, `safety check` (Python)
- [ ] Database migrations run successfully
- [ ] Environment variables set correctly
- [ ] Monitoring/logging configured (Sentry, LogRocket)
- [ ] Backup strategy in place (Neon automatic backups enabled)

### Rollback Plan
- **Frontend**: Revert to previous GitHub Pages deployment
- **Backend**: Railway/Vercel rollback to previous deployment
- **Database**: Revert migration with `alembic downgrade -1`

---

## Monitoring and Observability

### Logging Standards
- **Log Levels**:
  - DEBUG: Detailed diagnostic info (disabled in production)
  - INFO: General informational messages
  - WARNING: Something unexpected but handled
  - ERROR: Serious problem that needs attention
  - CRITICAL: System is unusable

- **Structured Logging**: JSON format for easy parsing
  ```python
  logger.info(
      "Chatbot query processed",
      extra={
          "user_id": user.id,
          "query_length": len(message),
          "response_time_ms": elapsed,
          "sources_used": len(context_chunks)
      }
  )
  ```

### Error Tracking
- **Frontend**: Sentry for JavaScript errors
  - Source maps uploaded for readable stack traces
  - User context attached (user_id, page, action)

- **Backend**: Sentry for Python exceptions
  - Request context attached (endpoint, method, params)
  - Database query logging in development

### Metrics to Monitor
- **Frontend**:
  - Page load times (RUM - Real User Monitoring)
  - Chatbot interaction rate (% of users who open chatbot)
  - Error rate (JS errors per session)

- **Backend**:
  - API response times (p50, p95, p99)
  - Error rate (5xx errors per hour)
  - Database connection pool utilization
  - Qdrant query latency

---

## Accessibility Standards

### WCAG 2.1 AA Compliance
- **Keyboard Navigation**: All interactive elements accessible via keyboard
  - Tab order logical
  - Focus indicators visible
  - No keyboard traps

- **Screen Reader Support**:
  - ARIA labels for icon buttons (`aria-label="Open chatbot"`)
  - ARIA live regions for dynamic content (`aria-live="polite"`)
  - Semantic HTML (`<nav>`, `<main>`, `<article>`)

- **Color Contrast**: Minimum 4.5:1 for normal text, 3:1 for large text
  - Test with Chrome DevTools Lighthouse
  - Provide high-contrast theme option

- **Responsive Design**: Functional on all viewport sizes
  - Mobile-first CSS
  - Touch targets minimum 44x44px
  - No horizontal scrolling

---

## Content Standards (Book Chapters)

### Writing Quality
- **Clarity**: Technical concepts explained without jargon overload
  - Define acronyms on first use: "ROS 2 (Robot Operating System 2)"
  - Analogies for complex topics: "Think of ROS 2 nodes like microservices..."

- **Accuracy**: All technical claims verified against official docs
  - Use Context7 MCP to query official ROS 2, NVIDIA Isaac, Gazebo docs
  - Cite sources for complex topics

- **Pedagogy**: 
  - Learning objectives stated at chapter start
  - Worked examples before exercises
  - Incremental complexity (scaffolding)

### Code Examples
- **Completeness**: Every code example is runnable as-is
  ```python
  # ❌ Bad (missing imports, context)
  node = Node('my_node')
  
  # ✅ Good (complete, runnable)
  #!/usr/bin/env python3
  import rclpy
  from rclpy.node import Node
  
  def main():
      rclpy.init()
      node = Node('my_node')
      node.get_logger().info('Hello from ROS 2!')
      rclpy.spin(node)
      node.destroy_node()
      rclpy.shutdown()
  
  if __name__ == '__main__':
      main()
  ```

- **Explanation**: Code annotated with inline comments for learning
  ```python
  # Subscribe to the /cmd_vel topic (velocity commands)
  self.subscription = self.create_subscription(
      Twist,                    # Message type
      '/cmd_vel',               # Topic name
      self.velocity_callback,   # Callback function
      10                        # QoS (queue size)
  )
  ```

### Multimedia
- **Diagrams**: Use Mermaid.js for architecture diagrams (editable)
- **Videos**: Embed demo videos (< 2 minutes each)
- **Screenshots**: Annotated with arrows/highlights for clarity

---

## Internationalization (i18n) Standards

### Urdu Translation (Bonus Feature)
- **Translation Quality**: Professional, not machine-translated
  - Use OpenAI GPT-4 with specialized prompts for technical translation
  - Maintain technical terms in English (e.g., "ROS 2 node" not "ROS 2 نوڈ")
  - Right-to-left (RTL) layout for Urdu content

- **Consistency**: 
  - Glossary of translated terms (maintain across chapters)
  - Same translator/prompt for all content

- **Testing**: Native Urdu speakers review translated chapters

---

## Success Criteria for Constitution Compliance

This constitution is met when:
- [ ] All backend endpoints have type hints and error handling
- [ ] All frontend components have TypeScript interfaces
- [ ] Test coverage meets minimums (80% backend, 70% frontend)
- [ ] Lighthouse scores meet minimums (90+ performance, 100 accessibility)
- [ ] All commits follow Conventional Commits format
- [ ] All PRs pass CI/CD checks (tests, linting, type-checking)
- [ ] Security audit tools report 0 critical/high vulnerabilities
- [ ] Documentation exists for all major features (ADRs, READMEs)
- [ ] Monitoring configured for production (Sentry, metrics)
- [ ] Deployment checklist completed before production launch

---

## Revision History

| Version | Date       | Changes                          | Author |
|---------|------------|----------------------------------|--------|
| 1.0.0   | 2024-12-06 | Initial constitution created     | AI     |

---

## Enforcement

- **Automated Checks**: CI/CD pipeline enforces standards (linting, type-checking, tests)
- **Code Review**: Reviewers verify constitution compliance before approving PRs
- **AI Collaboration**: Gemini CLI prompts include constitution reference
- **Exceptions**: Require documented justification in ADR format

This constitution is a living document. Update it when standards change, but never lower quality bars without team consensus.