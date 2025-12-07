import type { ReactNode } from "react";
import clsx from "clsx";
import Heading from "@theme/Heading";
import { Bot, BrainCircuit, CircuitBoard } from "lucide-react";
import styles from "./styles.module.css";

type FeatureItem = {
  title: string;
  icon: React.ComponentType<React.ComponentProps<"svg">>;
  description: ReactNode;
};

const FeatureList: FeatureItem[] = [
  {
    title: "Comprehensive Curriculum",
    icon: BrainCircuit,
    description: (
      <>
        12-week curriculum covering fundamental concepts to advanced topics in
        Physical AI and Humanoid Robotics, from kinematics and dynamics to
        machine learning and control systems.
      </>
    ),
  },
  {
    title: "Hands-on Learning",
    icon: CircuitBoard,
    description: (
      <>
        Practical exercises and projects using simulation environments like
        Gazebo and real-world platforms to reinforce theoretical concepts with
        actual implementation.
      </>
    ),
  },
  {
    title: "Cutting-Edge Topics",
    icon: Bot,
    description: (
      <>
        Explore the latest research in humanoid robotics, including locomotion,
        manipulation, perception, and human-robot interaction in real-world
        environments.
      </>
    ),
  },
];

function Feature({ title, icon: Icon, description }: FeatureItem) {
  return (
    <div className={clsx("col col--4")}>
      <div className="text--center">
        <Icon className={styles.featureSvg} strokeWidth={0.75} />
      </div>
      <div className="text--center padding-horiz--md">
        <Heading as="h3">{title}</Heading>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures(): ReactNode {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          <div className="col col--12">
            <Heading as="h2" className={styles.sectionTitle}>
              Course Features
            </Heading>
            <p className={styles.sectionSubtitle}>
              What makes this curriculum exceptional:
            </p>
          </div>
        </div>
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
