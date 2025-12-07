import type { ReactNode } from "react";
import clsx from "clsx";
import Heading from "@theme/Heading";
import styles from "./styles.module.css";

type LearningOutcomeItem = {
  title: string;
  description: ReactNode;
};

const LearningOutcomeList: LearningOutcomeItem[] = [
  {
    title: "Design Robot Control Systems",
    description: (
      <>
        Master ROS 2 architecture with proper communication patterns between
        nodes, topics, services, and actions for humanoid robot development.
      </>
    ),
  },
  {
    title: "Deploy Artificial Intelligence Algorithms",
    description: (
      <>
        Implement NVIDIA Isaac platform with GPU acceleration for real-time
        robotics applications and human-robot interaction.
      </>
    ),
  },
  {
    title: "Create Simulation Environments",
    description: (
      <>
        Build and validate simulation environments using Gazebo and Unity for
        safe and efficient humanoid robot development and testing.
      </>
    ),
  },
];

function LearningOutcome({ title, description }: LearningOutcomeItem) {
  return (
    <div className={clsx("col col--4")}>
      <div className="text--center padding-horiz--md">
        <div className={styles.learningOutcomeCard}>
          <Heading as="h3">{title}</Heading>
          <p>{description}</p>
        </div>
      </div>
    </div>
  );
}

export default function LearningOutcomeSection(): ReactNode {
  return (
    <section className={styles.learningOutcomeSection}>
      <div className="container">
        <div className="row">
          <div className="col col--12">
            <Heading as="h2" className={styles.sectionTitle}>
              Course Learning Outcomes
            </Heading>
            <p className={styles.sectionSubtitle}>
              By the end of this course, students will be able to
            </p>
          </div>
        </div>
        <div className="row">
          {LearningOutcomeList.map((props, idx) => (
            <LearningOutcome key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
