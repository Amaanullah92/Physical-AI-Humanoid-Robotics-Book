# Research Plan: Physical AI Book

**Feature Branch**: `001-physical-ai-book`
**Created**: 2025-12-06

This document outlines key decisions requiring research and the alternatives to consider for the "Book on Physical AI & Humanoid Robotics."

## Decisions Needing Documentation and Research Tasks

### 1. Choice of ROS 2 Version
- **Decision**: ROS 2 Humble Hawksbill.
- **Rationale**: Humble Hawksbill is a Long Term Support (LTS) release supported until May 31, 2027. This provides the stability and longevity required for an academic book, ensuring that code examples and tutorials remain relevant and functional for an extended period. ROS 2 Iron Irwini is End-of-Life (EOL) as of December 4, 2024, making it unsuitable for new projects or ongoing development requiring official support and security updates.
- **Alternatives**: ROS 2 Humble Hawksbill, ROS 2 Iron Irwini.
- **Research Task**: Completed. Analyzed release cycles, end-of-life dates, package availability, and community support. Prioritized stability for an academic course.

### 2. Simulation Platform Priority
- **Decision**: NVIDIA Isaac Sim as the primary simulation platform, with significant coverage of Gazebo as an open-source alternative.
- **Rationale**: NVIDIA Isaac Sim is highly optimized for AI training and high-fidelity robotics, leveraging GPU-accelerated physics and RTX-enabled sensor simulations. This aligns perfectly with the book's focus on bridging digital AI (LLMs, perception, simulation) with embodied systems and its explicit mention of NVIDIA Isaac. While Isaac Sim has higher hardware requirements, its advanced capabilities for AI-driven robotics and photorealistic environments are crucial for demonstrating the book's core concepts. Gazebo will be covered as a widely adopted, open-source alternative with strong ROS integration, making the content accessible to a broader audience with varying hardware.
- **Alternatives**: Gazebo (Classic/Ignition/Garden), NVIDIA Isaac Sim (Omniverse), Unity 3D (Robotics Hub).
- **Research Task**: Completed. Compared capabilities, hardware requirements, ROS 2 integration, and accessibility, leading to the decision to prioritize Isaac Sim while including Gazebo.

### 3. VLA Integration Method
- **Decision**: Prioritize integrated multimodal VLA models (e.g., GPT-4o or specialized robotics VLM/VLA models) as the primary integration method, with a discussion of modular approaches (Whisper + GPT) as an alternative.
- **Rationale**: Integrated multimodal models are advancing rapidly, offering unified processing of text, vision, and audio within a single framework. This reduces pipeline complexity and latency, aligning with the book's emphasis on end-to-end Physical AI pipelines and cutting-edge embodied intelligence. GPT-4o demonstrates native audio understanding and efficient multimodal input/output. Modular approaches using Whisper for speech-to-text combined with separate LLMs can be effective but introduce more complexity. The book will focus on the integrated approach as the leading edge, while acknowledging the modular option for specific scenarios or foundational understanding.
- **Alternatives**: Modular approach (Whisper + GPT-series models), Integrated multimodal models (e.g., newer large language-vision models like GPT-4o or robotics-specific VLMs/VLAs).
- **Research Task**: Completed. Investigated current state-of-the-art in VLA for robotics, comparing performance, computational requirements, data annotation needs, and ease of deployment for different integration strategies. Focused on practical applicability for a humanoid robot capstone.

### 4. Robot Model Selection
- **Decision**: Utilize a publicly available humanoid robot URDF (e.g., from Unitree, Agility Robotics, or prominent open-source projects) for the capstone project and examples, focusing on its adaptability for educational simplification.
- **Rationale**: This approach balances providing students with exposure to realistic, industry-relevant robotic systems while ensuring the complexity can be managed for learning. Existing URDFs offer a foundation of detailed kinematics and dynamics, which can then be simplified or modularized for specific educational objectives, such as highlighting core joint kinematics or basic sensor integration. This avoids the overhead of creating a custom model from scratch while still allowing for pedagogical tailoring.
- **Alternatives**: Publicly available humanoid robot URDF (e.g., from Unitree, Agility Robotics, or open-source projects), simplified custom URDF.
- **Research Task**: Completed. Investigated existing humanoid URDF models and their suitability for educational purposes, considering their compatibility with ROS 2 and selected simulation platforms (NVIDIA Isaac Sim, Gazebo). Evaluated options based on realism, complexity, available documentation, and potential for pedagogical simplification.

### 5. Hardware Recommendations
- **Decision**: Recommend a hybrid approach: an NVIDIA RTX workstation (on-premise) for core development and local high-fidelity simulation, supplemented by NVIDIA Omniverse Cloud deployment for scalable training, synthetic data generation, and large-scale simulations.
- **Rationale**: An on-premise RTX workstation (e.g., equipped with NVIDIA RTX PRO Blackwell GPUs in 2025) provides direct control, minimal latency for iterative development, and is suitable for consistent, intensive local workloads. This aligns with the CapEx model for foundational infrastructure. For tasks requiring elastic scalability, such as large-scale synthetic data generation, extensive robot learning, or distributed simulation, leveraging Omniverse Cloud (e.g., on Microsoft Azure, AWS, or Google Cloud) offers OpEx benefits, flexible resource scaling, and access to powerful, managed platforms like NVIDIA DGX Cloud and Isaac Sim on EC2 G6e instances. This hybrid strategy optimizes for both development efficiency and scalable computational needs.
- **Alternatives**: Local development workstation (NVIDIA RTX series GPUs), Cloud-based NVIDIA Omniverse development.
- **Research Task**: Completed. Researched current hardware specifications for NVIDIA RTX GPUs and Jetson platforms relevant for ROS 2 and Isaac Sim. Compared the costs, setup complexity, performance benchmarks, and benefits of local vs cloud development environments for embodied AI projects, leading to a hybrid recommendation.

### 6. Code Style Conventions
- **Decision**: Primarily Python (rclpy) for most examples and high-level logic, with a clear explanation of when and why C++ (rclcpp) is preferred for performance-critical sections. Adhere to PEP8 for Python and the official ROS C++ Style Guide (referencing Google C++ Style Guide) for C++.
- **Rationale**: Prioritizing Python makes the content more accessible to a broader student audience, aligning with its suitability for rapid development in high-level control, planning, and AI within robotics. The book will emphasize idiomatic Python practices using `rclpy`. For components requiring maximum performance or low-level hardware interaction, C++ will be introduced, following `rclcpp` and established ROS 2 C++ style guides (e.g., Google C++ Style Guide). This approach balances ease of learning with practical performance considerations, preparing students for real-world robotics development.
- **Alternatives**: Primarily Python (rclpy) with C++ only for performance-critical sections, balanced mix of Python and C++ examples.
- **Research Task**: Completed. Reviewed common ROS 2 coding practices for both Python and C++. Investigated best practices for teaching Python-first robotics development and when to introduce C++ for performance optimization. Aligned with standard Python (PEP 8) and C++ (ROS 2 C++ Style Guide) conventions.

### 7. Diagram Generation Tools
- **Decision**: Leverage Claude Code's diagram generation capabilities (e.g., Mermaid, draw.io XML) as the primary tool for creating architectural, pipeline, and robot-specific diagrams, supplemented by manual refinement or external tools (like PlantUML, if needed for highly specific representations) for complex scenarios.
- **Rationale**: Claude Code offers significant advantages in speed, consistency, and automated generation of diagrams (flowcharts, sequence diagrams, system architectures) directly from prompts, which ensures synchronization with content and reduces manual effort. It can produce various formats, including Mermaid and draw.io XML, facilitating integration into Docusaurus. This aligns with the need for rapid iteration within the hackathon timeline. For diagrams requiring extremely fine-grained control, very specific layout, or advanced features not yet fully supported by AI generation, manual tools can serve as a fallback, but the primary focus will be on maximizing AI assistance for efficiency.
- **Alternatives**: Claude Code's diagram generation (if available/suitable), external diagramming tools (e.g., draw.io, PlantUML, Mermaid), manual SVG/PNG creation.
- **Research Task**: Completed. Evaluated the current capabilities of Claude Code (or similar AI tools) for generating accurate and structured architectural diagrams, TF trees, and sensor pipelines. Compared this with the control, flexibility, and tooling available in dedicated diagramming software, leading to a decision to prioritize Claude Code for efficiency.

