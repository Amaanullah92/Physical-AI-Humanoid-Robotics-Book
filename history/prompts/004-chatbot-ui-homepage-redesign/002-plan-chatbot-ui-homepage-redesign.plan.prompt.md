---
id: 002
title: Plan Chatbot UI and Homepage Redesign
stage: plan
date: 2025-12-18
surface: {{SURFACE}}
model: {{MODEL}}
feature: 004-chatbot-ui-homepage-redesign
branch: 004-chatbot-ui-homepage-redesign
user: {{USER}}
command: {{COMMAND}}
labels: [{{LABELS}}]
links:
  spec: {{LINKS_SPEC}}
  ticket: {{LINKS_TICKET}}
  adr: {{LINKS_ADR}}
  pr: {{LINKS_PR}}
files:
{{FILES_YAML}}
tests:
{{TESTS_YAML}}
---

## Prompt

/sp.plan

Create:
- Architecture sketch for floating button and mini chat window
- Section structure:
  1. Floating button placement and styling
  2. Mini chat window design and animation (open/close)
  3. Homepage layout improvements: hero section, featured content, navigation
  4. Responsive design adjustments for mobile and tablet

Decisions needing documentation:
- Position of floating button (bottom-right vs bottom-left)
- Size and style of chat window (mini vs full-screen)
- Homepage sections: which elements to feature prominently
- Color scheme and alignment with book theme
- Component structure: standalone ChatWidget wrapper vs integration into layout

Testing strategy:
- Verify floating button appears on all book pages without blocking content
- Click button opens mini chat window and allows interactions
- Closing/minimizing works correctly
- Homepage displays correctly on desktop and mobile
- Check for layout breakages and overlapping elements

Technical details:
- Use React dynamic import for ChatWidget to avoid SSR issues
- Use CSS fixed positioning and z-index for floating button
- Use Docusaurus Layout component or custom wrapper for homepage redesign
- Use media queries or Docusaurus built-in responsiveness for mobile
- Follow SDD sequential phase: Floating button → Mini window → Homepage redesign → Testing

## Response snapshot

Implementation plan created for chatbot UI enhancements and Docusaurus homepage redesign.

## Outcome

- âœ… Impact: {{OUTCOME_IMPACT}}
- ðŸ§ª Tests: {{TESTS_SUMMARY}}
- ðŸ“ Files: {{FILES_SUMMARY}}
- ðŸ” Next prompts: {{NEXT_PROMPTS}}
- ðŸ§  Reflection: {{REFLECTION_NOTE}}

## Evaluation notes (flywheel)

- Failure modes observed: {{FAILURE_MODES}}
- Graders run and results (PASS/FAIL): {{GRADER_RESULTS}}
- Prompt variant (if applicable): {{PROMPT_VARIANT_ID}}
- Next experiment (smallest change to try): {{NEXT_EXPERIMENT}}
