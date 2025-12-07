import type { ReactNode } from "react";
import clsx from "clsx";
import Link from "@docusaurus/Link";
import useDocusaurusContext from "@docusaurus/useDocusaurusContext";
import Layout from "@theme/Layout";
import HomepageFeatures from "@site/src/components/HomepageFeatures";
import LearningOutcomeSection from "@site/src/components/LearningOutcomeSection";
import Heading from "@theme/Heading";

import styles from "./index.module.css";

function HomepageHeader() {
  const { siteConfig } = useDocusaurusContext();
  return (
    <header className={clsx("hero hero--primary", styles.heroBanner)}>
      <div className="container">
        <div className={styles.heroContent}>
          <div className={styles.heroText}>
            <Heading as="h1" className="hero__title">
              {siteConfig.title}
            </Heading>
            <p className="hero__subtitle">{siteConfig.tagline}</p>
            <p className={styles.heroDescription}>
              Master the cutting-edge field of Physical AI and Humanoid Robotics
              with this comprehensive textbook designed for advanced
              undergraduate and graduate students.
            </p>
            <div className={styles.buttons}>
              <Link
                className="button button--primary button--lg"
                to="/docs/intro"
              >
                Start Learning
              </Link>
              <Link
                className="button button--primary button--lg margin-left--md"
                to="/docs/module-1"
              >
                Learning Outcomes
              </Link>
            </div>
          </div>
          <div className={styles.heroVisual}>
            <img
              src="img/robot.jpg"
              alt="Humanoid Robot Illustration"
              className={styles.heroImage}
            />
          </div>
        </div>
      </div>
    </header>
  );
}

export default function Home(): ReactNode {
  const { siteConfig } = useDocusaurusContext();
  return (
    <Layout
      title={`${siteConfig.title}`}
      description="Description will go into a meta tag in <head />"
    >
      <HomepageHeader />
      <main>
        <LearningOutcomeSection />
        <HomepageFeatures />
      </main>
    </Layout>
  );
}
