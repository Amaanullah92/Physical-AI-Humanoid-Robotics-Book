<!-- Sync Impact Report:
Version change: 0.0.0 (template) -> 1.0.0
Modified principles: N/A (initial setup)
Added sections: All sections from user input
Removed sections: All template placeholders
Templates requiring updates:
- .specify/templates/plan-template.md: ✅ updated
- .specify/templates/spec-template.md: ✅ updated
- .specify/templates/tasks-template.md: ✅ updated
- .specify/templates/commands/sp.constitution.md: ✅ updated
Follow-up TODOs: N/A
-->
Project: Book on Physical AI & Humanoid Robotics for Docusaurus

Core principles:
- Technical accuracy based on robotics, AI, simulation, and hardware standards
- Clarity for an academic + engineering audience (undergrad CS/AI/Robotics students)
- Reproducibility: all explanations, code samples, and architectures must be technically implementable
- Alignment with official documentation of ROS 2, Gazebo, Unity, NVIDIA Isaac, and Vision-Language-Action systems
- Conceptual consistency: maintain a unified terminology for Physical AI, embodied intelligence, humanoid robotics, and digital twins
- Zero hallucinations: all system descriptions must reflect real capabilities of ROS 2, Isaac Sim, SLAM, VLA models, and Jetson hardware

Key standards:
- All technical claims must be verifiable against authoritative sources (ROS 2 docs, NVIDIA Isaac docs, research papers)
- Diagrams must reflect real system architectures (ROS graphs, Isaac pipelines, sensor flow)
- Writing clarity: must match a university-level robotics textbook tone (Flesch-Kincaid grade 10–12)
- All modules must follow the quarter structure: ROS 2 → Gazebo/Unity → Isaac → VLA → Capstone
- Use consistent naming for modules, weeks, systems, APIs, and hardware
- Examples must be runnable on Ubuntu 22.04, ROS 2 Humble or Iron, and Isaac Sim

Constraints:
- Book structure must follow Docusaurus best practices (sidebars, indexes, versioning)
- Minimum 12 chapters (covering all modules and weekly breakdown)
- Include diagrams, code snippets, configuration examples, launch files, and architecture sketches
- No fictional hardware; only actual devices (Jetson Orin, RealSense D435i, Unitree Go2/G1, etc.)
- All technical descriptions must avoid speculation about unverified features
- Provide step-by-step guides where needed (ROS 2 nodes, URDF, Isaac Sim setup)
- Maintain strict separation between concepts, tutorials, and assessments

Success criteria:
- Book accurately explains the entire Physical AI course and weekly curriculum
- ROS 2, Gazebo, Isaac, and VLA concepts are technically correct and reproducible
- Hardware requirements section is precise, realistic, and matched to current market specs
- Capstone humanoid robot pipeline is coherent, technically feasible, and aligned with course goals
- Readers can understand, simulate, and deploy a humanoid robot digital twin by the end
- All content fits into a consistent Docusaurus site, deployable on GitHub Pages without errors

## Governance
Constitution supersedes all other practices; Amendments require documentation, approval, migration plan; All PRs/reviews must verify compliance; Complexity must be justified.

**Version**: 1.0.0 | **Ratified**: 2025-12-06 | **Last Amended**: 2025-12-06