import type { SidebarsConfig } from "@docusaurus/plugin-content-docs";

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
      type: "category",
      label: "Introduction",
      items: ["intro"],
    },
    {
      type: "category",
      label: "Overview",
      items: [
        "overview/learning-outcomes",
        "overview/why-physical-ai-matters",
      ],
    },
    {
      type: "category",
      label: "Module 1: Introduction to Physical AI and Humanoid Robotics",
      items: [
        "module-1/index",
        "module-1/theory",
        "module-1/lab-exercise",
        "module-1/assessment",
      ],
    },
    {
      type: "category",
      label: "Module 2: Robot Operating Systems and ROS 2 Fundamentals",
      items: [
        "module-2/index",
        "module-2/theory",
        "module-2/lab-exercise",
        "module-2/assessment",
      ],
    },
    {
      type: "category",
      label: "Module 3: Robot Perception and Sensor Integration",
      items: [
        "module-3/index",
        "module-3/theory",
        "module-3/lab-exercise",
        "module-3/assessment",
      ],
    },
    {
      type: "category",
      label: "Module 4: Robot Simulation with Gazebo",
      items: [
        "module-4/index",
        "module-4/theory",
        "module-4/lab-exercise",
        "module-4/assessment",
      ],
    },
    {
      type: "category",
      label: "Module 5: Advanced Control Systems & Trajectory Planning",
      items: [
        "module-5/index",
        "module-5/theory",
        "module-5/lab-exercise",
        "module-5/assessment",
      ],
    },
    {
      type: "category",
      label: "Module 6: Multi-Robot Systems & Coordination",
      items: [
        "module-6/index",
        "module-6/theory",
        "module-6/lab-exercise",
        "module-6/assessment",
      ],
    },
    {
      type: "category",
      label: "Module 7: Machine Learning for Robotics",
      items: [
        "module-7/index",
        "module-7/theory",
        "module-7/lab-exercise",
        "module-7/assessment",
      ],
    },
    {
      type: "category",
      label: "Module 8: Integration & Deployment",
      items: [
        "module-8/index",
        "module-8/theory",
        "module-8/lab-exercise",
        "module-8/assessment",
      ],
    },
    {
      type: "category",
      label: "Module 9: Human-Robot Interaction & Social Robotics",
      items: [
        "module-9/index",
        "module-9/theory",
        "module-9/lab-exercise",
        "module-9/assessment",
      ],
    },
    {
      type: "category",
      label: "Module 10: Advanced Topics in Robot Learning",
      items: [
        "module-10/index",
        "module-10/theory",
        "module-10/lab-exercise",
        "module-10/assessment",
      ],
    },
    {
      type: "category",
      label: "Module 11: Field Robotics & Real-World Applications",
      items: [
        "module-11/index",
        "module-11/theory",
        "module-11/lab-exercise",
        "module-11/assessment",
      ],
    },
    {
      type: "category",
      label: "Module 12: Project Development & Capstone",
      items: [
        "module-12/index",
        "module-12/theory",
        "module-12/lab-exercise",
        "module-12/assessment",
      ],
    },
    {
      type: "category",
      label: "Hardware & Safety",
      items: ["hardware-requirements", "safety-guidelines"],
    },
    {
      type: "category",
      label: "Community",
      items: ["community/contributing", "community/faq"],
    },
  ],
};

export default sidebars;
