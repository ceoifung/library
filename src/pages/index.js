/*
 * @Author: Ceoifung
 * @Date: 2023-03-31 10:21:49
 * @LastEditors: Ceoifung
 * @LastEditTime: 2025-03-05 15:50:27
 * @Description: XiaoRGEEK All Rights Reserved. Copyright © 2023
 */
import React from 'react';
// import clsx from 'clsx';
// import Link from '@docusaurus/Link';
// import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
// import Layout from '@theme/Layout';
// import HomepageFeatures from '@site/src/components/HomepageFeatures';

// import styles from './index.module.css';
import { Redirect } from 'react-router-dom';
import { prodcutName } from "../../package.json"

// function HomepageHeader() {
//   const {siteConfig} = useDocusaurusContext();
//   return (
//     <header className={clsx('hero hero--primary', styles.heroBanner)}>
//       <div className="container">
//         <h1 className="hero__title">{siteConfig.title}</h1>
//         <p className="hero__subtitle">{siteConfig.tagline}</p>
        
//         <div className={styles.buttons}>
//           <Link
//             className="button button--secondary button--lg"
//             to="/docs/intro">
//             学而时习之
//           </Link>
//         </div>
//       </div>
//     </header>
//   );
// }

// export default function Home() {
//   const {siteConfig} = useDocusaurusContext();
//   return (
//     <Layout
//       title={`${siteConfig.title}，而知也无涯，也有涯随无涯，殆矣`}
//       description="Description will go into a meta tag in <head />">
//       <HomepageHeader />
//       <main>
//         <HomepageFeatures />
//       </main>
//     </Layout>
//   );
// }
// const redictUrl = process.env.NODE_ENV === 'development' ? "/docs/intro" : `${prodcutName}/docs/intro`

// console.log(redictUrl)
export default function Home() {

  return <Redirect to= {process.env.NODE_ENV == 'development' ?'/docs/intro':`/${prodcutName}/docs/intro`} />;
}
