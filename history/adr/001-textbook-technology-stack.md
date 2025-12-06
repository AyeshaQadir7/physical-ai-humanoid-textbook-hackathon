# ADR-001: Textbook Technology Stack

## Status
Proposed

## Date
2025-12-06

## Context
The Physical AI & Humanoid Robotics textbook needs to be built using a technology stack that supports:
- Educational content delivery
- Interactive documentation
- GitHub Pages deployment
- Integration with Spec-Kit Plus workflow
- Support for code samples, diagrams, and multimedia

## Decision
We will use the following technology stack:
- **Framework**: Docusaurus v3 as the static site generator
- **Deployment**: GitHub Pages for hosting
- **Documentation Format**: Markdown with Docusaurus extensions
- **Build Tool**: npm with standard Node.js ecosystem
- **Integration**: Spec-Kit Plus for planning and generation workflow

This stack provides a proven solution for technical documentation with excellent support for code samples, versioning, and responsive design.

## Alternatives
- **VuePress + Netlify**: Alternative static site generator with different ecosystem
- **GitBook + GitBook hosting**: Purpose-built for books but less flexible
- **Custom React app with Next.js**: More control but higher maintenance overhead
- **Sphinx + Read the Docs**: Traditional for technical documentation but Python-focused

## Consequences
**Positive:**
- Docusaurus has excellent support for technical documentation with code highlighting
- GitHub Pages integration is straightforward
- Strong community support and documentation
- Built-in search functionality
- Responsive design out of the box
- Good support for versioned documentation if needed later

**Negative:**
- Additional dependency on Node.js ecosystem
- Potential build complexity as content grows
- Possible performance issues with very large documentation sets
- Lock-in to Docusaurus-specific markdown extensions

## References
- plan.md: Sections 1.2, 1.3, 5.3
- spec.md: Section 4 (Docusaurus Structure)