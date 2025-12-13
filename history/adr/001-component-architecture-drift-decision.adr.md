# ADR-001: Component Architecture Decision - MDX vs JSX Components

## Status
Accepted

## Date
2025-12-11

## Context
The original plan specified React .jsx interactive utilities under `src/components/` but implementation drifted to MDX playground files in the same directory structure. The specification called for interactive elements like personalization and translation hooks, but the implementation resulted in both .js and .mdx versions of components like PersonalizeChapter and TranslateToUrdu, causing potential conflicts.

## Decision
We will standardize on JSX components for interactive elements to maintain cleaner separation between content and presentation layers. The JSX format provides better component lifecycle management and clearer separation of concerns. We will remove the MDX versions to avoid import conflicts and ensure consistent component architecture.

Specifically:
- Keep JSX components: `PersonalizeChapter.jsx`, `TranslateToUrdu.jsx`, etc.
- Remove MDX component versions: `PersonalizeChapter.mdx`, `TranslateToUrdu.mdx`, etc.
- Maintain the component structure under `physical-AI-book/src/components/` as originally planned
- Use JSX format with Docusaurus BrowserOnly wrapper for client-side rendering needs

## Consequences

### Positive
- Cleaner separation between content (MDX/Markdown) and interactive components (JSX)
- Better component lifecycle management and React patterns
- Improved performance compared to MDX components
- Clearer separation of concerns between presentation and content
- Single source of truth for components (no duplicate versions)

### Negative
- Requires BrowserOnly wrapper for client-side rendering needs
- Slightly more complex setup for Docusaurus integration
- Need to import components separately rather than inline in MDX content

## Alternatives Considered

### Alternative 1: MDX Components
Use MDX format for interactive elements to blend content and functionality
- Pros: Better integration with Docusaurus documentation platform, ability to mix content and interactivity
- Cons: More complex component structure, potential performance overhead, harder to maintain separation of concerns

### Alternative 2: Hybrid Approach
Mix JSX and MDX components based on functionality
- Pros: Could optimize for specific use cases
- Cons: Would maintain the confusion and import conflicts that already existed

## References
- `/specs/001-physical-ai-book/plan.md` - Project structure specification
- `/specs/001-physical-ai-book/contracts/api-contracts.md` - Interactive element specifications
- `/specs/001-physical-ai-book/data-model.md` - Interactive Element entity specification