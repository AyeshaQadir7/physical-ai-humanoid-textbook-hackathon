import { themes as prismThemes } from "prism-react-renderer";
import type { Config } from "@docusaurus/types";
import type * as Preset from "@docusaurus/preset-classic";

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

const config: Config = {
  title: "Physical AI & Humanoid Robotics Textbook",
  tagline: "",
  favicon: "img/bot-favicon.svg",

  // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
  future: {
    v4: true, // Improve compatibility with the upcoming Docusaurus v4
  },

  // Set the production url of your site here
  url: "https://soft-hands.github.io",
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: "/hackathon-ai-textbook/",

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: "AyeshaQadir7", // Usually your GitHub org/user name.
  projectName: "physical-ai-humanoid-textbook-hackathon", // Usually your repo name.

  onBrokenLinks: "warn",

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: "en",
    locales: ["en"],
  },

  presets: [
    [
      "classic",
      {
        docs: {
          sidebarPath: "./sidebars.ts",
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            "https://github.com/AyeshaQadir7/physical-ai-humanoid-textbook-hackathon/edit/main/physical-ai-textbook/",
        },
        // blog: {
        //   showReadingTime: true,
        //   feedOptions: {
        //     type: ["rss", "atom"],
        //     xslt: true,
        //   },
        //   // Please change this to your repo.
        //   // Remove this to remove the "edit this page" links.
        //   editUrl:
        //     "https://github.com/AyeshaQadir7/physical-ai-humanoid-textbook-hackathon/edit/main/physical-ai-textbook/",
        //   // Useful options to enforce blogging best practices
        //   onInlineTags: "warn",
        //   onInlineAuthors: "warn",
        //   onUntruncatedBlogPosts: "warn",
        // },
        theme: {
          customCss: "./src/css/custom.css",
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    // Replace with your project's social card
    image: "img/docusaurus-social-card.jpg",
    colorMode: {
      respectPrefersColorScheme: true,
    },
    navbar: {
      title: "Physical AI & Humanoid Robotics",
      logo: {
        alt: "Physical AI & Humanoid Robotics Textbook Logo",
        src: "img/bot-logo.svg",
      },
      items: [
        {
          type: "docSidebar",
          sidebarId: "tutorialSidebar",
          position: "left",
          label: "Book",
        },
        {
          type: "doc",
          docId: "overview/learning-outcomes",
          position: "left",
          label: "Learning Outcomes",
        },

        {
          href: "https://github.com/AyeshaQadir7",
          label: "Author",
          position: "right",
        },
        // { to: "/blog", label: "Blog", position: "left" },
        {
          href: "https://github.com/AyeshaQadir7/physical-ai-humanoid-textbook-hackathon",
          label: "GitHub",
          position: "right",
        },
      ],
    },
    footer: {
      style: "dark",
      links: [
        {
          title: "Textbook",
          items: [
            {
              label: "Introduction",
              to: "/docs/intro",
            },
            {
              label: "Learning Outcomes",
              to: "/docs/overview/learning-outcomes",
            },
            {
              label: "Why Physical AI Matters",
              to: "/docs/overview/why-physical-ai-matters",
            },
          ],
        },
        {
          title: "Community",
          items: [
            {
              label: "GitHub Repository",
              href: "https://github.com/AyeshaQadir7/physical-ai-humanoid-textbook-hackathon",
            },
            {
              label: "Issues",
              href: "https://github.com/AyeshaQadir7/physical-ai-humanoid-textbook-hackathon/issues",
            },
            {
              label: "Discussions",
              href: "https://github.com/AyeshaQadir7/physical-ai-humanoid-textbook-hackathon/discussions",
            },
          ],
        },
        {
          title: "Resources",
          items: [
            {
              label: "ROS 2 Documentation",
              href: "https://docs.ros.org/",
            },
            {
              label: "Gazebo Simulation",
              href: "https://gazebosim.org/",
            },
            {
              label: "Unity 3D",
              href: "https://unity.com/",
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Textbook. Built with Docusaurus.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
