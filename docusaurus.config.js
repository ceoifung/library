// @ts-check
// Note: type annotations allow type checking and IDEs autocompletion

const lightCodeTheme = require('prism-react-renderer/themes/github');
const darkCodeTheme = require('prism-react-renderer/themes/dracula');
const organizationName = "ceoifung";
const projectName = "library";
const isDevelopment = process.env.NODE_ENV === 'development';
/** @type {import('@docusaurus/types').Config} */
const config = {
  title: '吾生也有涯',
  tagline: '日知其所亡，月无忘其所能，可谓好学也已矣',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  url: isDevelopment ? "http://localhost:3000": `https://${organizationName}.github.io`, // 默认本地开发环境,
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: isDevelopment ? '/' : `/${projectName}/`,

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName, // Usually your GitHub org/user name.
  projectName, // Usually your repo name.

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',

  // Even if you don't use internalization, you can use this field to set useful
  // metadata like html lang. For example, if your site is Chinese, you may want
  // to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
           editUrl: `https://github.com/${organizationName}/${projectName}/tree/master/`
        },
        blog: {
          showReadingTime: true,
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl: `https://github.com/${organizationName}/${projectName}/tree/master/`
        },
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      }),
    ],
  ],
  plugins: [
    // '@aldridged/docusaurus-plugin-lunr',
    require.resolve('docusaurus-lunr-search'),
    // [
    //   require.resolve("@easyops-cn/docusaurus-search-local"),
    //   ({
    //     hashed: true,
    //     indexPages: true
    //   }),
    // ],
    'docusaurus-plugin-image-zoom',
    [
      '@docusaurus/plugin-ideal-image',
      {
        disableInDev: false,
      },
    ],
    [
      '@docusaurus/plugin-pwa',
      {
        debug: true,
        offlineModeActivationStrategies: [
          'appInstalled',
          'standalone',
          'queryString',
        ],
        pwaHead: [
          {
            tagName: 'link',
            rel: 'icon',
            href: 'img/logo.png',
          },
          {
            tagName: 'link',
            rel: 'manifest',
            href: 'manifest.json',
          },
          {
            tagName: 'meta',
            name: 'theme-color',
            content: 'rgb(51 139 255)',
          },
        ],
      },
    ],
  ],
  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      // Replace with your project's social card
      image: 'img/docusaurus-logo.jpg',
      navbar: {
        title: '吾生也有涯',
        logo: {
          alt: 'My Site Logo',
          src: 'img/docusaurus-logo.jpg',
        },
        // 滑动的时候隐藏标题栏
        hideOnScroll: true,
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'tutorialSidebar',
            position: 'left',
            label: '文档',
          },
          { href: 'https://blog.csdn.net/qq_41020634?type=blog', label: '博客', position: 'left' },
          {
            href: 'https://github.com/ceoifung',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Docs',
            items: [
              {
                label: '文档',
                to: '/docs/intro',
              },
            ],
          },
          {
            title: 'Community',
            items: [
              {
                label: 'CSDN',
                href: 'https://blog.csdn.net/qq_41020634?type=blog',
              },
              {
                label: 'npm project',
                href: 'https://npmjs.com/ceoifung',
              },
              // {
              //   label: 'Github',
              //   href: 'https://github.com/ceoifung',
              // },
            ],
          },
          {
            title: 'More',
            items: [
              {
                label: '博客',
                href: 'https://blog.csdn.net/qq_41020634?type=blog',
              },
              {
                label: 'GitHub',
                href: 'https://github.com/facebook/docusaurus',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Ceoifung, Inc. Built with Docusaurus.`,
      },
      prism: {
        theme: lightCodeTheme,
        darkTheme: darkCodeTheme,
        additionalLanguages: ['powershell', 'java', 'kotlin', 'python'],
      },
      tableOfContents: {
        minHeadingLevel: 2,
        maxHeadingLevel: 5,
      },
      zoom: {
        selector: '.markdown :not(em) > img',
        background: {
          light: 'rgb(255, 255, 255)',
          dark: 'rgb(50, 50, 50)',
        },
        config: {},
      },
    }),
};

module.exports = config;
