import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

const sidebars: SidebarsConfig = {
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      collapsed: false,
      items: [
        'chapters/introduction',
        'chapters/ros2-fundamentals',
        'chapters/robot-modeling',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      collapsed: false,
      items: [
        'chapters/simulation',
        'chapters/control-systems', // If this belongs to Module 2
        'chapters/perception',       // Or adjust based on your intended structure
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac)',
      collapsed: false,
      items: [
        'chapters/planning-navigation',
        'chapters/vla-reasoning', // Rename if needed
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Capstone & Advanced Topics',
      collapsed: false,
      items: [
        'chapters/capstone-project',
        'chapters/advanced-topics',
        'chapters/conclusion-appendices',
        'chapters/future-trends',
      ],
    },
  ],
};

export default sidebars;
