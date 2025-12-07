import React from 'react';
import Link from '@docusaurus/Link';
import { Bot } from 'lucide-react';
import styles from './styles.module.css';

export default function NavbarLogo(): JSX.Element {
  return (
    <Link to="/" className="navbar__brand">
      <Bot
        className={styles.navbar__logo}
        size={32}
        strokeWidth={1}
        style={{
          marginRight: '0.5rem',
          verticalAlign: 'middle'
        }}
      />
      <strong className="navbar__title">Physical AI & Humanoid Robotics</strong>
    </Link>
  );
}