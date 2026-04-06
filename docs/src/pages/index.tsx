import type { ReactNode } from 'react';
import { useEffect, useRef, useState } from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';

import styles from './index.module.css';

/* ── Inline SVG Components ─────────────────────────────────── */

function CircuitPattern() {
  return (
    <svg
      className={styles.circuitSvg}
      viewBox="0 0 800 600"
      fill="none"
      xmlns="http://www.w3.org/2000/svg"
    >
      {/* Neural network nodes and connections */}
      <g opacity="0.15" stroke="url(#circuitGrad)" strokeWidth="1">
        {/* Horizontal lines */}
        <line x1="0" y1="100" x2="800" y2="100" className={styles.circuitLine} />
        <line x1="0" y1="200" x2="800" y2="200" className={styles.circuitLine} />
        <line x1="0" y1="300" x2="800" y2="300" className={styles.circuitLine} />
        <line x1="0" y1="400" x2="800" y2="400" className={styles.circuitLine} />
        <line x1="0" y1="500" x2="800" y2="500" className={styles.circuitLine} />
        {/* Vertical lines */}
        <line x1="100" y1="0" x2="100" y2="600" className={styles.circuitLine} />
        <line x1="200" y1="0" x2="200" y2="600" className={styles.circuitLine} />
        <line x1="400" y1="0" x2="400" y2="600" className={styles.circuitLine} />
        <line x1="600" y1="0" x2="600" y2="600" className={styles.circuitLine} />
        <line x1="700" y1="0" x2="700" y2="600" className={styles.circuitLine} />
        {/* Diagonal connections */}
        <line x1="100" y1="100" x2="200" y2="200" className={styles.circuitLine} />
        <line x1="400" y1="100" x2="600" y2="300" className={styles.circuitLine} />
        <line x1="700" y1="200" x2="600" y2="400" className={styles.circuitLine} />
        <line x1="200" y1="300" x2="400" y2="500" className={styles.circuitLine} />
      </g>
      {/* Glowing nodes */}
      <g>
        {[
          [100, 100], [200, 200], [400, 100], [600, 300],
          [700, 200], [600, 400], [200, 300], [400, 500],
          [100, 400], [700, 500], [300, 150], [500, 250],
        ].map(([cx, cy], i) => (
          <circle
            key={i}
            cx={cx}
            cy={cy}
            r="3"
            fill="url(#nodeGrad)"
            className={styles.nodeGlow}
            style={{ animationDelay: `${i * 0.3}s` }}
          />
        ))}
      </g>
      {/* Traveling pulse along paths */}
      <circle r="2" fill="#00d4ff" opacity="0.8">
        <animateMotion
          dur="6s"
          repeatCount="indefinite"
          path="M100,100 L200,200 L400,500 L600,400 L700,200"
        />
      </circle>
      <circle r="2" fill="#7c3aed" opacity="0.8">
        <animateMotion
          dur="8s"
          repeatCount="indefinite"
          path="M700,500 L600,300 L400,100 L200,300 L100,400"
        />
      </circle>
      <circle r="1.5" fill="#06b6d4" opacity="0.6">
        <animateMotion
          dur="5s"
          repeatCount="indefinite"
          path="M300,150 L400,100 L600,300 L500,250 L200,200"
        />
      </circle>
      <defs>
        <linearGradient id="circuitGrad" x1="0%" y1="0%" x2="100%" y2="100%">
          <stop offset="0%" stopColor="#06b6d4" />
          <stop offset="50%" stopColor="#7c3aed" />
          <stop offset="100%" stopColor="#06b6d4" />
        </linearGradient>
        <radialGradient id="nodeGrad">
          <stop offset="0%" stopColor="#00d4ff" />
          <stop offset="100%" stopColor="#7c3aed" stopOpacity="0" />
        </radialGradient>
      </defs>
    </svg>
  );
}

function HexagonGrid() {
  return (
    <svg
      className={styles.hexGrid}
      viewBox="0 0 600 400"
      fill="none"
      xmlns="http://www.w3.org/2000/svg"
    >
      {[
        [50, 50], [150, 50], [250, 50], [350, 50], [450, 50], [550, 50],
        [100, 130], [200, 130], [300, 130], [400, 130], [500, 130],
        [50, 210], [150, 210], [250, 210], [350, 210], [450, 210], [550, 210],
        [100, 290], [200, 290], [300, 290], [400, 290], [500, 290],
        [50, 370], [150, 370], [250, 370], [350, 370], [450, 370], [550, 370],
      ].map(([cx, cy], i) => (
        <polygon
          key={i}
          points={hexPoints(cx, cy, 28)}
          stroke="url(#hexGrad)"
          strokeWidth="0.5"
          fill="none"
          opacity={0.06 + (i % 5) * 0.02}
          className={styles.hexCell}
          style={{ animationDelay: `${i * 0.15}s` }}
        />
      ))}
      <defs>
        <linearGradient id="hexGrad" x1="0%" y1="0%" x2="100%" y2="100%">
          <stop offset="0%" stopColor="#06b6d4" />
          <stop offset="100%" stopColor="#8b5cf6" />
        </linearGradient>
      </defs>
    </svg>
  );
}

function hexPoints(cx: number, cy: number, r: number): string {
  return Array.from({ length: 6 }, (_, i) => {
    const angle = (Math.PI / 3) * i - Math.PI / 6;
    return `${cx + r * Math.cos(angle)},${cy + r * Math.sin(angle)}`;
  }).join(' ');
}

function RobotArmSvg() {
  return (
    <svg
      className={styles.robotArm}
      viewBox="0 0 400 400"
      fill="none"
      xmlns="http://www.w3.org/2000/svg"
    >
      {/* Base */}
      <rect x="160" y="340" width="80" height="40" rx="6" fill="url(#armGrad1)" opacity="0.9" />
      <rect x="180" y="320" width="40" height="30" rx="4" fill="url(#armGrad2)" opacity="0.85" />
      {/* Lower arm */}
      <g className={styles.armSegment1}>
        <rect x="190" y="220" width="20" height="110" rx="10" fill="url(#armGrad2)" opacity="0.8" />
        <circle cx="200" cy="220" r="14" fill="url(#jointGrad)" className={styles.jointPulse} />
        {/* Upper arm */}
        <g className={styles.armSegment2}>
          <rect x="192" y="130" width="16" height="100" rx="8" fill="url(#armGrad3)" opacity="0.75"
            transform="rotate(-15, 200, 220)" />
          <circle cx="185" cy="135" r="12" fill="url(#jointGrad)" className={styles.jointPulse}
            style={{ animationDelay: '0.5s' }} />
          {/* Gripper */}
          <g className={styles.gripper} transform="translate(170, 100)">
            <line x1="15" y1="30" x2="5" y2="50" stroke="#06b6d4" strokeWidth="3" strokeLinecap="round" />
            <line x1="15" y1="30" x2="25" y2="50" stroke="#06b6d4" strokeWidth="3" strokeLinecap="round" />
            <circle cx="15" cy="28" r="5" fill="url(#jointGrad)" />
          </g>
        </g>
      </g>
      {/* Energy rings */}
      <circle cx="200" cy="200" r="150" stroke="url(#ringGrad)" strokeWidth="0.5" opacity="0.2"
        className={styles.energyRing} />
      <circle cx="200" cy="200" r="180" stroke="url(#ringGrad)" strokeWidth="0.3" opacity="0.1"
        className={styles.energyRing} style={{ animationDelay: '1s' }} />
      <defs>
        <linearGradient id="armGrad1" x1="0%" y1="0%" x2="0%" y2="100%">
          <stop offset="0%" stopColor="#1e293b" />
          <stop offset="100%" stopColor="#0f172a" />
        </linearGradient>
        <linearGradient id="armGrad2" x1="0%" y1="0%" x2="0%" y2="100%">
          <stop offset="0%" stopColor="#334155" />
          <stop offset="100%" stopColor="#1e293b" />
        </linearGradient>
        <linearGradient id="armGrad3" x1="0%" y1="0%" x2="0%" y2="100%">
          <stop offset="0%" stopColor="#475569" />
          <stop offset="100%" stopColor="#334155" />
        </linearGradient>
        <radialGradient id="jointGrad">
          <stop offset="0%" stopColor="#22d3ee" />
          <stop offset="100%" stopColor="#0891b2" />
        </radialGradient>
        <linearGradient id="ringGrad" x1="0%" y1="0%" x2="100%" y2="100%">
          <stop offset="0%" stopColor="#06b6d4" />
          <stop offset="50%" stopColor="#8b5cf6" />
          <stop offset="100%" stopColor="#06b6d4" />
        </linearGradient>
      </defs>
    </svg>
  );
}

/* ── Floating Particle System (CSS-only) ──────────────────── */

function Particles() {
  const particles = Array.from({ length: 30 }, (_, i) => ({
    id: i,
    left: `${(i * 37) % 100}%`,
    top: `${(i * 53) % 100}%`,
    size: 1 + (i % 3),
    delay: `${(i * 0.7) % 8}s`,
    duration: `${6 + (i % 6)}s`,
  }));
  return (
    <div className={styles.particleField}>
      {particles.map((p) => (
        <span
          key={p.id}
          className={styles.particle}
          style={{
            left: p.left,
            top: p.top,
            width: p.size,
            height: p.size,
            animationDelay: p.delay,
            animationDuration: p.duration,
          }}
        />
      ))}
    </div>
  );
}

/* ── Scroll-reveal hook ───────────────────────────────────── */

function useReveal() {
  const ref = useRef<HTMLDivElement>(null);
  const [visible, setVisible] = useState(false);
  useEffect(() => {
    const el = ref.current;
    if (!el) return;
    const obs = new IntersectionObserver(
      ([entry]) => { if (entry.isIntersecting) setVisible(true); },
      { threshold: 0.15 },
    );
    obs.observe(el);
    return () => obs.disconnect();
  }, []);
  return { ref, visible };
}

/* ── Stats Data ───────────────────────────────────────────── */

const stats = [
  { value: '4', label: 'Learning Modules' },
  { value: '12+', label: 'Hands-On Lessons' },
  { value: 'AI', label: 'Powered Assistant' },
  { value: '∞', label: 'Open Source' },
];

/* ── Module Cards ─────────────────────────────────────────── */

const modules = [
  {
    icon: (
      <svg viewBox="0 0 48 48" fill="none" className={styles.moduleIcon}>
        <circle cx="24" cy="24" r="22" stroke="url(#mod1)" strokeWidth="1.5" opacity="0.3" />
        <path d="M24 8L24 40M14 14L34 14M14 24L34 24M14 34L34 34" stroke="url(#mod1)" strokeWidth="1.5" strokeLinecap="round" />
        <circle cx="24" cy="14" r="3" fill="#06b6d4" />
        <circle cx="24" cy="24" r="3" fill="#22d3ee" />
        <circle cx="24" cy="34" r="3" fill="#67e8f9" />
        <circle cx="14" cy="24" r="2" fill="#06b6d4" opacity="0.6" />
        <circle cx="34" cy="24" r="2" fill="#06b6d4" opacity="0.6" />
        <defs><linearGradient id="mod1" x1="0%" y1="0%" x2="100%" y2="100%">
          <stop offset="0%" stopColor="#06b6d4" /><stop offset="100%" stopColor="#0891b2" />
        </linearGradient></defs>
      </svg>
    ),
    title: 'ROS 2 Fundamentals',
    desc: 'Master the Robot Operating System 2 — the foundational middleware powering modern autonomous robots.',
    tag: 'Module 1',
    color: '#06b6d4',
  },
  {
    icon: (
      <svg viewBox="0 0 48 48" fill="none" className={styles.moduleIcon}>
        <rect x="6" y="20" width="36" height="22" rx="4" stroke="url(#mod2)" strokeWidth="1.5" />
        <path d="M12 20V12a12 12 0 0 1 24 0v8" stroke="url(#mod2)" strokeWidth="1.5" />
        <circle cx="24" cy="8" r="4" fill="#8b5cf6" opacity="0.5" />
        <rect x="16" y="28" width="4" height="6" rx="1" fill="#8b5cf6" opacity="0.7" />
        <rect x="22" y="26" width="4" height="8" rx="1" fill="#a78bfa" opacity="0.8" />
        <rect x="28" y="30" width="4" height="4" rx="1" fill="#8b5cf6" opacity="0.6" />
        <defs><linearGradient id="mod2" x1="0%" y1="0%" x2="100%" y2="100%">
          <stop offset="0%" stopColor="#8b5cf6" /><stop offset="100%" stopColor="#7c3aed" />
        </linearGradient></defs>
      </svg>
    ),
    title: 'Gazebo Simulation',
    desc: 'Build and test in photorealistic virtual worlds. Simulate physics, sensors, and robot behavior at scale.',
    tag: 'Module 2',
    color: '#8b5cf6',
  },
  {
    icon: (
      <svg viewBox="0 0 48 48" fill="none" className={styles.moduleIcon}>
        <path d="M24 4L44 16V32L24 44L4 32V16L24 4Z" stroke="url(#mod3)" strokeWidth="1.5" />
        <path d="M24 4L24 44M4 16L44 32M44 16L4 32" stroke="url(#mod3)" strokeWidth="0.8" opacity="0.4" />
        <circle cx="24" cy="24" r="6" fill="url(#mod3)" opacity="0.3" />
        <circle cx="24" cy="24" r="3" fill="#10b981" />
        <circle cx="24" cy="4" r="2.5" fill="#10b981" opacity="0.8" />
        <circle cx="44" cy="16" r="2" fill="#10b981" opacity="0.6" />
        <circle cx="4" cy="16" r="2" fill="#10b981" opacity="0.6" />
        <defs><linearGradient id="mod3" x1="0%" y1="0%" x2="100%" y2="100%">
          <stop offset="0%" stopColor="#10b981" /><stop offset="100%" stopColor="#059669" />
        </linearGradient></defs>
      </svg>
    ),
    title: 'NVIDIA Isaac Sim',
    desc: 'Harness GPU-accelerated, AI-driven simulation for training robots in NVIDIA\'s cutting-edge platform.',
    tag: 'Module 3',
    color: '#10b981',
  },
  {
    icon: (
      <svg viewBox="0 0 48 48" fill="none" className={styles.moduleIcon}>
        <circle cx="16" cy="16" r="10" stroke="url(#mod4)" strokeWidth="1.5" />
        <circle cx="32" cy="16" r="10" stroke="url(#mod4)" strokeWidth="1.5" opacity="0.5" />
        <path d="M8 34C8 34 16 28 24 34C32 28 40 34 40 34" stroke="url(#mod4)" strokeWidth="1.5" strokeLinecap="round" />
        <circle cx="13" cy="14" r="1.5" fill="#f59e0b" />
        <circle cx="19" cy="14" r="1.5" fill="#f59e0b" />
        <path d="M13 19a3 3 0 0 0 6 0" stroke="#f59e0b" strokeWidth="1" strokeLinecap="round" />
        <path d="M20 38L28 42L36 38" stroke="url(#mod4)" strokeWidth="1.5" strokeLinecap="round" opacity="0.6" />
        <defs><linearGradient id="mod4" x1="0%" y1="0%" x2="100%" y2="100%">
          <stop offset="0%" stopColor="#f59e0b" /><stop offset="100%" stopColor="#d97706" />
        </linearGradient></defs>
      </svg>
    ),
    title: 'Vision-Language-Action',
    desc: 'Bridge perception and action — build robots that see, understand language, and act autonomously.',
    tag: 'Module 4',
    color: '#f59e0b',
  },
];

/* ── Main Homepage ────────────────────────────────────────── */

function HomepageHero() {
  const { siteConfig } = useDocusaurusContext();
  return (
    <section className={styles.hero}>
      {/* Animated background layers */}
      <div className={styles.bgMesh} />
      <div className={styles.bgNoise} />
      <Particles />
      <CircuitPattern />
      <HexagonGrid />

      {/* Gradient orbs */}
      <div className={clsx(styles.orb, styles.orb1)} />
      <div className={clsx(styles.orb, styles.orb2)} />
      <div className={clsx(styles.orb, styles.orb3)} />

      <div className={styles.heroInner}>
        <div className={styles.heroLeft}>
          <span className={styles.badge}>
            <span className={styles.badgeDot} />
            Open Source &middot; Free Forever
          </span>

          <Heading as="h1" className={styles.heroTitle}>
            <span className={styles.titleLine1}>The Future of</span>
            <span className={styles.titleGradient}>Physical AI</span>
            <span className={styles.titleLine3}>& Humanoid Robotics</span>
          </Heading>

          <p className={styles.heroSubtitle}>
            A comprehensive, hands-on learning platform covering ROS&nbsp;2, Gazebo,
            NVIDIA Isaac Sim, and Vision-Language-Action models — everything you
            need to build the next generation of intelligent robots.
          </p>

          <div className={styles.heroCtas}>
            <Link className={styles.ctaPrimary} to="/docs/intro">
              <span>Start Learning</span>
              <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round">
                <path d="M5 12h14M12 5l7 7-7 7" />
              </svg>
            </Link>
            <Link className={styles.ctaSecondary} to="https://github.com/hashir-qt/Spec-Driven-Book">
              <svg width="18" height="18" viewBox="0 0 24 24" fill="currentColor">
                <path d="M12 0C5.37 0 0 5.37 0 12c0 5.3 3.438 9.8 8.205 11.387.6.113.82-.258.82-.577 0-.285-.01-1.04-.015-2.04-3.338.724-4.042-1.61-4.042-1.61-.546-1.387-1.333-1.756-1.333-1.756-1.089-.745.083-.729.083-.729 1.205.084 1.838 1.236 1.838 1.236 1.07 1.835 2.809 1.305 3.495.998.108-.776.417-1.305.76-1.605-2.665-.3-5.466-1.332-5.466-5.93 0-1.31.465-2.38 1.235-3.22-.135-.303-.54-1.523.105-3.176 0 0 1.005-.322 3.3 1.23.96-.267 1.98-.399 3-.405 1.02.006 2.04.138 3 .405 2.28-1.552 3.285-1.23 3.285-1.23.645 1.653.24 2.873.12 3.176.765.84 1.23 1.91 1.23 3.22 0 4.61-2.805 5.625-5.475 5.92.42.36.81 1.096.81 2.22 0 1.606-.015 2.896-.015 3.286 0 .315.21.69.825.57C20.565 21.795 24 17.295 24 12 24 5.37 18.63 0 12 0z" />
              </svg>
              <span>View on GitHub</span>
            </Link>
          </div>
        </div>

        <div className={styles.heroRight}>
          <div className={styles.visualContainer}>
            <RobotArmSvg />
            <div className={styles.visualGlow} />
          </div>
        </div>
      </div>

      {/* Stats bar */}
      <div className={styles.statsBar}>
        {stats.map((s, i) => (
          <div key={i} className={styles.statItem}>
            <span className={styles.statValue}>{s.value}</span>
            <span className={styles.statLabel}>{s.label}</span>
          </div>
        ))}
      </div>

      {/* Scroll indicator */}
      <div className={styles.scrollIndicator}>
        <div className={styles.scrollDot} />
      </div>
    </section>
  );
}

function ModulesSection() {
  const reveal = useReveal();
  return (
    <section
      ref={reveal.ref}
      className={clsx(styles.modules, reveal.visible && styles.modulesVisible)}
    >
      <div className={styles.modulesInner}>
        <div className={styles.sectionHeader}>
          <span className={styles.sectionTag}>Curriculum</span>
          <Heading as="h2" className={styles.sectionTitle}>
            Four Modules. One Complete Journey.
          </Heading>
          <p className={styles.sectionSubtitle}>
            From foundational robotics middleware to cutting-edge AI — each module
            builds on the last to give you production-ready skills.
          </p>
        </div>

        <div className={styles.moduleGrid}>
          {modules.map((m, i) => (
            <div
              key={i}
              className={styles.moduleCard}
              style={{ '--accent': m.color, animationDelay: `${i * 0.12}s` } as React.CSSProperties}
            >
              <div className={styles.moduleCardHeader}>
                <span className={styles.moduleTag}>{m.tag}</span>
                {m.icon}
              </div>
              <Heading as="h3" className={styles.moduleCardTitle}>{m.title}</Heading>
              <p className={styles.moduleCardDesc}>{m.desc}</p>
              <div className={styles.moduleCardLine} />
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

function CTASection() {
  const reveal = useReveal();
  return (
    <section
      ref={reveal.ref}
      className={clsx(styles.ctaSection, reveal.visible && styles.ctaVisible)}
    >
      <div className={styles.ctaInner}>
        <div className={styles.ctaGlow} />
        <Heading as="h2" className={styles.ctaTitle}>
          Ready to Build the Future?
        </Heading>
        <p className={styles.ctaDesc}>
          Join the open-source community learning Physical AI and Humanoid Robotics.
          Start your journey today — it's completely free.
        </p>
        <Link className={styles.ctaPrimaryLarge} to="/docs/intro">
          <span>Get Started Now</span>
          <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round">
            <path d="M5 12h14M12 5l7 7-7 7" />
          </svg>
        </Link>
      </div>
    </section>
  );
}

export default function Home(): ReactNode {
  const { siteConfig } = useDocusaurusContext();
  return (
    <Layout
      title={`${siteConfig.title}`}
      description="An open-source book on the future of Physical AI and Humanoid Robotics."
    >
      <HomepageHero />
      <ModulesSection />
      <CTASection />
    </Layout>
  );
}
