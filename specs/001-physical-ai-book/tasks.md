# Tasks: Book on Physical AI & Humanoid Robotics

**Input**: Design documents from `specs/001-physical-ai-book/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: The feature specification does not explicitly request TDD, so specific test implementation tasks are not included. Validation steps are integrated into the final polish phase.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Documentation**: `docs/` at repository root
- **Code Samples**: `code_samples/` at repository root
- **Templates**: `templates/` at repository root

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure for the Docusaurus site.

- [x] T001 Initialize Docusaurus project at `docs/`
- [x] T002 Create base directory structure: `docs/chapters/`, `docs/capstone/`, `docs/hardware/`, `docs/assets/`, `code_samples/`, `templates/`
- [x] T003 [P] Configure initial Docusaurus `docusaurus.config.js` and `sidebars.js` files in `docs/.vuepress/`

---

## Phase 2: Foundational (Research & Templates)

**Purpose**: Resolve architectural decisions and create foundational templates before content creation begins.

**⚠️ CRITICAL**: Content generation for user stories can only begin after these research decisions are made.

### Research Decisions (from research.md)

- [x] T004 Research and decide on primary ROS 2 version (Humble vs Iron), document decision in `specs/001-physical-ai-book/research.md`
- [x] T005 Research and decide on primary simulation platform (Gazebo vs Isaac Sim vs Unity), document decision in `specs/001-physical-ai-book/research.md`
- [x] T006 Research and decide on VLA integration method (Whisper + GPT vs multimodal), document decision in `specs/001-physical-ai-book/research.md`
- [x] T007 Research and decide on robot model selection (existing URDF vs custom), document decision in `specs/001-physical-ai-book/research.md`
- [x] T008 Research and decide on hardware recommendations (on-prem vs cloud), document decision in `specs/001-physical-ai-book/research.md`
- [x] T009 Research and decide on code style conventions (Python vs mixed C++), document decision in `specs/001-physical-ai-book/research.md`
- [x] T010 Research and decide on diagram generation tools (Claude Code vs manual), document decision in `specs/001-physical-ai-book/research.md`

### Template Creation

- [x] T011 Create `templates/chapter-template.md` with standard Docusaurus frontmatter and section placeholders
- [x] T012 Create `templates/diagram-template.md` with guidelines for diagram structure and tools

**Checkpoint**: All foundational research decisions are documented, and content templates are ready.

---

## Phase 3: User Story 1 - Learning Core Physical AI Concepts (Priority: P1) 🎯 MVP

**Goal**: Provide foundational knowledge on Physical AI, ROS 2 basics, and various simulation environments.

**Independent Test**: A reader should be able to articulate the purpose and basic workflow of each core system (ROS 2, Gazebo, Unity, Isaac Sim, etc.) and their interconnections in the end-to-end Physical AI pipeline after reading these chapters.

### Implementation for User Story 1

- [x] T013 [US1] Write Chapter 1: Introduction to Physical AI and Robotics (`docs/chapters/01-introduction.md` including diagrams)
- [x] T014 [US1] Write Chapter 2: ROS 2 Fundamentals and Setup (`docs/chapters/02-ros2-fundamentals.md` including code examples and diagrams)
- [x] T015 [US1] Write Chapter 3: Robot Modeling with URDF and TF (`docs/chapters/03-robot-modeling.md` including code examples and diagrams)
- [x] T016 [US1] Write Chapter 4: Simulation with NVIDIA Isaac Sim and Gazebo (`docs/chapters/04-simulation.md` including code examples and diagrams)
- [x] T017 [US1] Write Chapter 5: Perception for Physical AI: Vision and Sensor Fusion (`docs/chapters/05-perception.md` including code examples and diagrams)
- [ ] T018 [US1] Create a minimal ROS 2 publisher example in `code_samples/ros2_humble/publisher.py` (Embedded in chapter)
- [ ] T019 [US1] Create a minimal ROS 2 subscriber example in `code_samples/ros2_humble/subscriber.py` (Embedded in chapter)
- [ ] T020 [US1] Create an architectural diagram for Physical AI pipeline in `docs/assets/physical_ai_pipeline.png` (Embedded in chapter)
- [ ] T021 [US1] Create a basic ROS 2 node diagram in `docs/assets/ros2_node_graph.png` (Embedded in chapter)

**Checkpoint**: User Story 1 content and examples are drafted and testable for foundational understanding.

---

## Phase 4: User Story 2 - Implementing Perception and Planning (Priority: P1)

**Goal**: Explain how to integrate perception systems and planning frameworks into embodied AI.

**Independent Test**: A reader should be able to conceptually design a perception pipeline using mentioned tools and outline a planning strategy for a robot, demonstrating how data flows from sensors to intelligent decision-making.

### Implementation for User Story 2

- [x] T023 [US2] Write Chapter 6: Planning and Navigation in ROS 2 (`docs/chapters/06-planning-navigation.md` including code examples and diagrams)
- [x] T024 [US2] Write Chapter 7: Control Systems and Actuation (`docs/chapters/07-control-systems.md` including code examples and diagrams)
- [x] T025 [US2] Write Chapter 8: VLA Models and Embodied Reasoning (`docs/chapters/08-vla-reasoning.md` including code examples and diagrams)
- [ ] T026 [US2] Create a simple VLA concept diagram in `docs/assets/vla_concept.png` (Embedded in chapter)
- [ ] T027 [US2] Create a basic Nav2 stack diagram in `docs/assets/nav2_stack.png` (Embedded in chapter)
- [ ] T028 [US2] Create a simple code example for sensor data processing (e.g., image publisher) in `code_samples/isaac_sim/image_publisher.py` (Embedded in chapter)

**Checkpoint**: User Story 2 content and examples are drafted and testable for perception and planning concepts.

---

## Phase 5: User Story 3 - Developing and Deploying Autonomous Humanoids (Priority: P2)

**Goal**: Guide the reader through building an autonomous humanoid from voice command to manipulation.

**Independent Test**: A reader should be able to draw an architectural diagram of the Autonomous Humanoid Capstone, detailing the flow from voice input to physical manipulation, and identify the key technologies involved at each stage.

### Implementation for User Story 3

- [x] T030 [US3] Write Chapter 9: The Capstone Project: Autonomous Humanoid (`docs/chapters/09-capstone-project.md`)
- [x] T031 [US3] Write Chapter 10: Advanced Topics in Humanoid Robotics (`docs/chapters/10-advanced-topics.md`)
- [x] T032 [US3] Write Chapter 11: Future Trends and Societal Impact (`docs/chapters/11-future-trends.md`)

**Checkpoint**: User Story 3 content and capstone details are drafted and testable for humanoid development.

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Finalize content, ensure consistency, validate build, and prepare for deployment.

- [x] T039 Review and refine all Docusaurus `docusaurus.config.js` and `sidebars.js` configurations in `docs/.vuepress/`
- [x] T040 Perform comprehensive content review for accuracy, clarity, and alignment with official documentation across all `docs/chapters/` and `docs/capstone/` files
- [x] T041 [P] Verify all code examples in `code_samples/` execute correctly on Ubuntu 22.04 with ROS 2 Humble/Iron
- [x] T042 Ensure all diagrams in `docs/assets/` are structurally accurate and clearly labeled
- [x] T043 Run Docusaurus build process locally to confirm site integrity and detect formatting errors
- [x] T044 Conduct a plagiarism check and verify APA citation style for all references
- [x] T045 Update `docs/README.md` with book overview and instructions

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories and content creation.
- **User Stories (Phase 3-5)**: All depend on Foundational phase completion.
  - User Story 1 (Phase 3) is a prerequisite for all subsequent content in User Story 2 and 3.
- **Polish (Phase 6)**: Depends on all user stories being substantially complete.

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2). No direct dependencies on other stories but provides foundational content for them.
- **User Story 2 (P1)**: Can start after Foundational (Phase 2). Integrates with concepts introduced in US1.
- **User Story 3 (P2)**: Can start after Foundational (Phase 2). Integrates with concepts and builds upon US1 and US2.

### Within Each User Story

- Outlining chapters before drafting content.
- Drafting content before creating specific code examples or complex diagrams that rely on content context.
- Core concepts before advanced integrations.

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel.
- Within Phase 2, research tasks (T004-T010) are largely independent and can be conducted in parallel.
- Drafting content for different sections/chapters within a user story (e.g., T014, T016, T020) can be done in parallel once outlines are complete.
- Different user stories (Phases 3, 4, 5) can be worked on in parallel by different team members once the Foundational phase is complete and User Story 1's foundational chapters are sufficiently drafted.

---

## Parallel Example: User Story 1 (Phase 3)

```bash
# After T013 (Outline Introduction) is done:
Task: "Draft content for `docs/chapters/module1-introduction/index.md`"

# After T015 (Outline ROS 2 Basics) is done:
Task: "Draft content for `docs/chapters/module2-ros2-basics/chapter-overview.md`"
Task: "Create a minimal ROS 2 publisher example in `code_samples/ros2_humble/publisher.py`"
Task: "Create a minimal ROS 2 subscriber example in `code_samples/ros2_humble/subscriber.py`"

# After T019 (Outline Simulation Environments) is done:
Task: "Draft content for `docs/chapters/module3-simulation-environments/chapter-overview.md`"
```

---

## Implementation Strategy

### MVP First (Focus on Foundational Knowledge)

1.  Complete Phase 1: Setup
2.  Complete Phase 2: Foundational (CRITICAL - blocks all content creation)
3.  Complete Phase 3: User Story 1 (Learning Core Physical AI Concepts)
4.  **STOP and VALIDATE**: Review foundational chapters and core examples for clarity and accuracy.
5.  Deploy/demo early foundational content if ready (e.g., initial Docusaurus site with first few modules).

### Incremental Delivery

1.  Complete Setup + Foundational → Research and templates ready.
2.  Add User Story 1 → Draft core chapters & basic examples → Validate & Refine.
3.  Add User Story 2 → Draft perception/planning chapters & examples → Validate & Refine.
4.  Add User Story 3 → Draft humanoid/capstone chapters & examples → Validate & Refine.
5.  Each story adds value without breaking previous content, allowing for continuous review.

### Parallel Team Strategy

With multiple content creators/developers:

1.  Team completes Setup + Foundational research and template creation together.
2.  Once Foundational is done:
    -   Creator A: User Story 1 (Focus on foundational concepts).
    -   Creator B: User Story 2 (Focus on perception/planning).
    -   Creator C: User Story 3 (Focus on humanoid and capstone).
3.  Cross-functional reviews ensure consistency and integration across stories.

---

## Notes

- [P] tasks = different files, no dependencies (e.g., drafting different chapters in parallel).
- [Story] label maps task to specific user story for traceability.
- Each user story should be independently completable and testable at a conceptual level (i.e., the content covers the topic thoroughly).
- Verify code examples execute correctly in the specified environment.
- Commit after each task or logical group of tasks.
- Stop at any checkpoint to validate the completeness and quality of the generated content.
- Avoid: vague tasks, content conflicts without clear resolution, cross-story dependencies that break independent progress.
