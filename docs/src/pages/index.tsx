import type { ReactNode } from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';
import Heading from '@theme/Heading';

import styles from './index.module.css';

function HomepageHeader() {
  const { siteConfig } = useDocusaurusContext();
  return (
    <header className={clsx('hero', styles.heroBanner)}>
      <div className="container">
        <div className={styles.heroContent}>
          <div className={styles.heroText}>
            <Heading as="h1" className={clsx('hero__title', styles.slideUp)}>
              {siteConfig.title}
            </Heading>
            <p className={clsx('hero__subtitle', styles.slideUp, styles.delay1)}>
              {siteConfig.tagline}
            </p>
            <div className={clsx(styles.buttons, styles.slideUp, styles.delay2)}>
              <Link
                className="button button--primary button--lg"
                to="/docs/intro">
                Start Learning Now
              </Link>
            </div>
          </div>
          <div className={clsx(styles.heroImageContainer, styles.fadeIn)}>
            <img
              src="img/hero_robot.png"
              alt="Futuristic Robot Reading"
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
      title={`Welcome to the ${siteConfig.title}`}
      description="An open-source book on the future of AI and Robotics.">
      <HomepageHeader />
      <main>
        <HomepageFeatures />
      </main>
    </Layout>
  );
}
