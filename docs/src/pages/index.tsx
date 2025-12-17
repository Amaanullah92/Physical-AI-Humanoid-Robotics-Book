import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <h1 className="hero__title">{siteConfig.title}</h1>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
                      <Link
                      className="button button--secondary button--lg"
                      to="./docs/chapters/introduction">
                      Explore the Book
                    </Link>        </div>
      </div>
    </header>
  );
}

export default function Home(): JSX.Element {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Hello from ${siteConfig.title}`}
      description="Description will go into a meta tag in <head />">
      <HomepageHeader />
      <main>
        <div className={clsx('container', styles.homepageNavigation)}>
          <div className="row">
            <div className="col col--6">
              <h2>Explore the Book</h2>
              <ul>
                <li><Link to="./docs/chapters/introduction">Introduction</Link></li>
                <li><Link to="./docs/chapters/ros2-fundamentals">ROS2 Fundamentals</Link></li>
                <li><Link to="./docs/chapters/robot-modeling">Robot Modeling</Link></li>
                <li><Link to="./docs/chapters/simulation">Simulation</Link></li>
              </ul>
            </div>
            <div className="col col--6">
              <h2>Featured Content</h2>
              <p>Discover key topics and latest insights.</p>
              {/* Placeholder for featured content */}
            </div>
          </div>
        </div>
        <HomepageFeatures />
      </main>
    </Layout>
  );
}
