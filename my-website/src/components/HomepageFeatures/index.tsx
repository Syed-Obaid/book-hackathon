import type {ReactNode} from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

type FeatureItem = {
  title: string;
  Svg: React.ComponentType<React.ComponentProps<'svg'>>;
  description: ReactNode;
  link: string;
};

const ModuleList: FeatureItem[] = [
  {
    title: 'Module 1: ROS 2 Foundation',
    Svg: require('@site/static/img/undraw_docusaurus_mountain.svg').default,
    description: (
      <>
        Learn ROS 2 Humble, create workspaces, define URDF robot models, and
        write publisher/subscriber nodes for humanoid control.
      </>
    ),
    link: 'docs/category/module-1-ros-2-foundation',
  },
  {
    title: 'Module 2: Digital Twin Simulation',
    Svg: require('@site/static/img/undraw_docusaurus_tree.svg').default,
    description: (
      <>
        Simulate robots in Gazebo and Unity with realistic physics, sensors
        (IMU, depth camera), and understand the sim-to-real gap.
      </>
    ),
    link: 'docs/category/module-2-digital-twin',
  },
  {
    title: 'Module 3: AI Perception & Navigation',
    Svg: require('@site/static/img/undraw_docusaurus_react.svg').default,
    description: (
      <>
        Train reinforcement learning navigation policies in Isaac Gym, integrate
        object detection (YOLO), and implement SLAM for environment mapping.
      </>
    ),
    link: 'docs/category/module-3-motion-planning',
  },
  {
    title: 'Module 4: VLA Integration',
    Svg: require('@site/static/img/undraw_docusaurus_mountain.svg').default,
    description: (
      <>
        Build Vision-Language-Action pipelines: speech recognition, LLM intent
        parsing, and multi-modal control for natural language robot commands.
      </>
    ),
    link: 'docs/category/module-4-embodied-ai',
  },
  {
    title: 'Module 5: Capstone Project',
    Svg: require('@site/static/img/undraw_docusaurus_tree.svg').default,
    description: (
      <>
        Design, simulate, and deploy autonomous humanoid systems to NVIDIA
        Jetson Orin. Demonstrate end-to-end Physical AI mastery.
      </>
    ),
    link: 'docs/category/module-5-integration--capstone',
  },
];




function Module({title, Svg, description, link}: FeatureItem) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center">
        <Svg className={styles.featureSvg} role="img" />
      </div>
      <div className="text--center padding-horiz--md">
        <Heading as="h3">{title}</Heading>
        <p>{description}</p>
        <a href={link} className="button button--secondary button--sm">
          Explore Module →
        </a>
      </div>
    </div>
  );
}

export default function HomepageFeatures(): ReactNode {
  return (
    <>
      <section className={styles.features}>
        <div className="container">
          <div className="text--center margin-bottom--lg">
            <Heading as="h2">5 Modules · 12 Weeks · Hands-On Learning</Heading>
            <p className="hero__subtitle">
              From ROS 2 basics to deploying AI on physical robots
            </p>
          </div>
          <div className="row">
            {ModuleList.map((props, idx) => (
              <Module key={idx} {...props} />
            ))}
          </div>
        </div>
      </section>

      <section className={styles.features} style={{backgroundColor: '#313336ff', padding: '2rem 0'}}>
        <div className="container">
          <div className="text--center margin-bottom--lg">
            <Heading as="h2">Hardware Requirements</Heading>
          </div>
          <div className="row">
            <div className="col col--6">
              <Heading as="h3">Development Workstation</Heading>
              <ul>
                <li><strong>GPU</strong>: NVIDIA RTX 4070 Ti or better (for Isaac Sim)</li>
                <li><strong>RAM</strong>: 64GB recommended</li>
                <li><strong>Storage</strong>: 1TB NVMe SSD</li>
                <li><strong>OS</strong>: Ubuntu 22.04 LTS</li>
              </ul>
            </div>
            <div className="col col--6">
              <Heading as="h3">Edge AI Kit (Optional)</Heading>
              <ul>
                <li><strong>Board</strong>: NVIDIA Jetson Orin Nano (8GB) or Orin NX (16GB)</li>
                <li><strong>Camera</strong>: Intel RealSense D435i depth camera</li>
                <li><strong>Microphone</strong>: USB microphone for VLA demos</li>
                <li><strong>Robot</strong>: Unitree Go2 (optional physical validation)</li>
              </ul>
            </div>
          </div>
          <div className="text--center margin-top--lg">
            <p><em>Note: All modules can be completed in simulation without physical hardware</em></p>
          </div>
        </div>
      </section>
    </>
  );
}
