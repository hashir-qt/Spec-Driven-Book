import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

const sidebars: SidebarsConfig = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1: ROS 2 Fundamentals',
      items: [
        'module-1-ros2/introduction',
        'module-1-ros2/building-your-first-node',
        'module-1-ros2/topics-and-pub-sub',
        'module-1-ros2/services-and-actions',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Gazebo Simulation',
      items: [
        'module-2-gazebo/gazebo-environment-setup',
        'module-2-gazebo/urdf-robot-modeling',
        'module-2-gazebo/physics-and-sensors',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: NVIDIA Isaac Sim',
      items: [
        'module-3-isaac/isaac-sim-introduction',
        'module-3-isaac/isaac-ros-perception',
        'module-3-isaac/sim-to-real-transfer',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action',
      items: [
        'module-4-vla/voice-commands-whisper-llm',
        'module-4-vla/cognitive-planning',
      ],
    },
  ],
};

export default sidebars;
