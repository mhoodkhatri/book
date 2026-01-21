import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

const sidebars: SidebarsConfig = {
  docs: [
    'intro',
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System',
      collapsed: false,
      link: {type: 'doc', id: 'module-1-ros2/index'},
      items: [
        'module-1-ros2/nodes-topics',
        'module-1-ros2/services-actions',
        'module-1-ros2/rclpy',
        'module-1-ros2/urdf',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin',
      link: {type: 'doc', id: 'module-2-simulation/index'},
      items: [
        'module-2-simulation/gazebo-basics',
        'module-2-simulation/physics-sim',
        'module-2-simulation/unity-integration',
        'module-2-simulation/sensors',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain',
      link: {type: 'doc', id: 'module-3-nvidia-isaac/index'},
      items: [
        'module-3-nvidia-isaac/isaac-sim',
        'module-3-nvidia-isaac/isaac-ros',
        'module-3-nvidia-isaac/vslam',
        'module-3-nvidia-isaac/nav2',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action',
      link: {type: 'doc', id: 'module-4-vla/index'},
      items: [
        'module-4-vla/whisper',
        'module-4-vla/cognitive-planning',
        'module-4-vla/capstone',
      ],
    },
    {
      type: 'category',
      label: 'Hardware',
      items: [
        'hardware/workstation',
        'hardware/edge-kit',
      ],
    },
    {
      type: 'category',
      label: 'Appendix',
      items: [
        'appendix/assessments',
        'appendix/resources',
      ],
    },
  ],
};

export default sidebars;
