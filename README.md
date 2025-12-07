# Physical AI & Humanoid Robotics Textbook

This repository contains a comprehensive, college-level technical textbook that educates students in Physical AI and Humanoid Robotics through hands-on learning experiences. The textbook guides learners through the complete development lifecycle of humanoid robots using modern tools and frameworks including ROS 2, Gazebo, Unity, NVIDIA Isaac, and Vision-Language-Action systems.

## About the Textbook

This curriculum follows a structured weekly module breakdown that progresses from foundational concepts to a capstone autonomous humanoid project. Each week culminates in a functional robot subsystem that contributes to a larger humanoid robot project, incorporating hardware requirements, laboratory exercises, assessments, and visual diagrams to ensure deep understanding and practical competency.

The content is structured as markdown files compatible with Docusaurus for deployment to GitHub Pages, following the principles of open-source and reproducible learning.

## Weekly Structure

The curriculum is organized into 12 weekly modules:

1. **Week 1**: Introduction to Physical AI and Humanoid Robotics
2. **Week 2**: Robot Operating Systems and ROS 2 Fundamentals
3. **Week 3**: Robot Perception and Sensor Integration
4. **Week 4**: Robot Simulation with Gazebo
5. **Week 5**: Motion Planning and Control
6. **Week 6**: Unity Integration for Advanced Visualization
7. **Week 7**: NVIDIA Isaac for Accelerated AI Computing
8. **Week 8**: Vision-Language-Action Systems
9. **Week 9**: Voice-to-Action Integration
10. **Week 10**: Hardware-Software Co-Design
11. **Week 11**: Advanced Control and Learning
12. **Week 12**: Capstone Autonomous Humanoid Project

## Technologies Covered

- **ROS 2**: Robot Operating System for robot software development
- **Gazebo**: Physics-based simulation environment
- **Unity**: Advanced visualization and AR/VR integration
- **NVIDIA Isaac**: GPU-accelerated AI computing for robotics
- **Vision-Language-Action Systems**: Multimodal AI integration
- **Voice-to-Action**: Natural interfaces for robot control

## Documentation Structure

The textbook content is organized as follows:
- `docs/`: Contains all textbook content organized by week
- `physical-ai-textbook/`: Docusaurus project for website generation
- `specs/`: Specification files for the textbook development
- `.github/workflows/`: GitHub Actions for deployment to GitHub Pages

## Deployment

The textbook is automatically deployed to GitHub Pages on every push to the master branch. The live site can be accessed at: https://soft-hands.github.io/hackathon-ai-textbook/

Alternatively, you can deploy this Docusaurus project to Vercel by following these steps:

1. Push this repository to GitHub if you haven't already
2. Go to [Vercel](https://vercel.com) and sign in
3. Click "New Project" and import your repository
4. Vercel will automatically detect this is a Docusaurus project
5. Add the following build settings in your Vercel project:
   - Build Command: `npm run build`
   - Output Directory: `build`
   - Development Command: `npm start`
6. Deploy your project!

## Contributing

This textbook is an open-source project. Contributions are welcome! Please see our [contributing guidelines](docs/community/contributing.md) for more information.

## License

This textbook is licensed under the Creative Commons Attribution 4.0 International License. You are free to share and adapt the material for any purpose, provided you give appropriate credit.

## Acknowledgments

This textbook was developed as part of the Physical AI & Humanoid Robotics Hackathon initiative, designed to provide accessible, comprehensive education in advanced robotics concepts.