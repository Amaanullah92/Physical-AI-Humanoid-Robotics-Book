# Physical AI & Humanoid Robotics Book

This repository contains the source content for the "Physical AI & Humanoid Robotics" book, which bridges digital AI concepts (LLMs, perception, simulation) with embodied systems (humanoids, sensors, locomotion). The book is designed for undergraduate students, early-career engineers, and AI developers transitioning into embodied intelligence.

## Table of Contents (Chapters 1-12)

1.  **Introduction to Physical AI and Robotics**: Defines Physical AI, its importance, and key disciplines.
2.  **ROS 2 Fundamentals and Setup**: Covers ROS 2 core concepts, architecture, and installation on Ubuntu 22.04.
3.  **Robot Modeling with URDF and TF**: Explores URDF for robot kinematics/dynamics and the TF system for coordinate frames.
4.  **Simulation with NVIDIA Isaac Sim and Gazebo**: Introduces key simulation platforms for robotics development.
5.  **Perception for Physical AI: Vision and Sensor Fusion**: Focuses on computer vision and sensor fusion techniques.
6.  **Planning and Navigation in ROS 2**: Covers path planning, motion control, and the Nav2 stack.
7.  **Control Systems and Actuation**: Delves into PID controllers and various types of robot actuators.
8.  **VLA Models and Embodied Reasoning**: Explores Vision-Language-Action models for high-level robot intelligence.
9.  **The Capstone Project: Autonomous Humanoid**: Defines a comprehensive project integrating all concepts.
10. **Advanced Topics in Humanoid Robotics**: Discusses whole-body control, compliant locomotion, and HRC.
11. **Future Trends and Societal Impact**: Examines emerging trends, ethical considerations, and regulatory landscapes.
12. **Conclusion and Appendices**: Summarizes key takeaways and provides a glossary, resources, and deployment guide.

## Getting Started

This book is built using [Docusaurus 3](https://docusaurus.io/). To set up and run the book locally:

1.  **Clone the repository**:
    ```bash
    git clone https://github.com/your-organization/physical-ai-book.git
    cd physical-ai-book/docs
    ```
2.  **Install dependencies**:
    ```bash
    npm install
    ```
3.  **Start the development server**:
    ```bash
    npm run start
    ```
    This command starts a local development server and opens up a browser window. Most changes are reflected live without having to restart the server.

4.  **Build the static site**:
    ```bash
    npm run build
    ```
    This command generates static content into the `build` directory and can be served using any static content hosting service.

## Deployment

The book is designed to be deployed to GitHub Pages. Refer to [Appendix C: Docusaurus Deployment Guide](docs/chapters/12-conclusion-appendices.md#appendix-c-docusaurus-deployment-guide) for detailed instructions on setting up a GitHub Actions workflow for deployment.

## Contributions

We welcome contributions to improve the book. Please refer to the `CONTRIBUTING.md` (if available) for guidelines.

## License

This project is licensed under the [MIT License](LICENSE) - see the `LICENSE` file for details.

---

