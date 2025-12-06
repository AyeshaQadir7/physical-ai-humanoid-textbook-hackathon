import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  // Manual sidebar for our textbook
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Introduction',
      items: ['intro'],
    },
    {
      type: 'category',
      label: 'Week 1: Introduction to Physical AI and Humanoid Robotics',
      items: [
        'week-1/index',
        'week-1/theory',
        'week-1/lab-exercise',
        'week-1/assessment'
      ],
    },
    {
      type: 'category',
      label: 'Week 2: Robot Operating Systems and ROS 2 Fundamentals',
      items: [
        'week-2/index',
        'week-2/theory',
        'week-2/lab-exercise',
        'week-2/assessment'
      ],
    },
    {
      type: 'category',
      label: 'Week 3: Robot Perception and Sensor Integration',
      items: [
        'week-3/index',
        'week-3/theory',
        'week-3/lab-exercise',
        'week-3/assessment'
      ],
    },
    {
      type: 'category',
      label: 'Week 4: Robot Simulation with Gazebo',
      items: [
        'week-4/index',
        'week-4/theory',
        'week-4/lab-exercise',
        'week-4/assessment'
      ],
    },
    {
      type: 'category',
      label: 'Week 5: Advanced Control Systems & Trajectory Planning',
      items: [
        'week-5/index',
        'week-5/theory',
        'week-5/lab-exercise',
        'week-5/assessment'
      ],
    },
    {
      type: 'category',
      label: 'Week 6: Multi-Robot Systems & Coordination',
      items: [
        'week-6/index',
        'week-6/theory',
        'week-6/lab-exercise',
        'week-6/assessment'
      ],
    },
    {
      type: 'category',
      label: 'Week 7: Machine Learning for Robotics',
      items: [
        'week-7/index',
        'week-7/theory',
        'week-7/lab-exercise',
        'week-7/assessment'
      ],
    },
    {
      type: 'category',
      label: 'Week 8: Integration & Deployment',
      items: [
        'week-8/index',
        'week-8/theory',
        'week-8/lab-exercise',
        'week-8/assessment'
      ],
    },
    {
      type: 'category',
      label: 'Week 9: Human-Robot Interaction & Social Robotics',
      items: [
        'week-9/index',
        'week-9/theory',
        'week-9/lab-exercise',
        'week-9/assessment'
      ],
    },
    {
      type: 'category',
      label: 'Week 10: Advanced Topics in Robot Learning',
      items: [
        'week-10/index',
        'week-10/theory',
        'week-10/lab-exercise',
        'week-10/assessment'
      ],
    },
    {
      type: 'category',
      label: 'Week 11: Field Robotics & Real-World Applications',
      items: [
        'week-11/index',
        'week-11/theory',
        'week-11/lab-exercise',
        'week-11/assessment'
      ],
    },
    {
      type: 'category',
      label: 'Week 12: Project Development & Capstone',
      items: [
        'week-12/index',
        'week-12/theory',
        'week-12/lab-exercise',
        'week-12/assessment'
      ],
    },
    {
      type: 'category',
      label: 'Hardware & Safety',
      items: [
        'hardware-requirements',
        'safety-guidelines'
      ],
    },
    {
      type: 'category',
      label: 'Community',
      items: [
        'community/contributing',
        'community/faq'
      ],
    },
  ],
};

export default sidebars;