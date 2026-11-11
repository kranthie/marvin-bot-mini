// Marvin Bot Mini documentation site: Astro Starlight, published to GitHub Pages by
// .github/workflows/pages.yml (Marvin Bot's site, without its interactive guide, which it links
// to). The docs pages are copied from the repository's Markdown (scripts/sync-docs.mjs), the home
// page is src/pages/index.astro, and the 3-D model page is src/content/docs/model.mdx with its
// viewer in src/model/.
import starlight from '@astrojs/starlight';
import { defineConfig } from 'astro/config';

const repo = 'https://github.com/kranthie/marvin-bot-mini';

export default defineConfig({
  site: 'https://kranthie.github.io',
  base: '/marvin-bot-mini',
  trailingSlash: 'always',
  // three.js is one large module, loaded only by the 3-D viewers; keep its licence header
  vite: { build: { chunkSizeWarningLimit: 1500, rolldownOptions: { output: { comments: { legal: true } } } } },
  integrations: [
    starlight({
      title: 'Marvin Bot Mini',
      description:
        'An open-source 0.6 m servo humanoid on Feetech servos and a Jetson Orin Nano, designed in code and trained in simulation.',
      logo: { src: './src/assets/logo.svg', replacesTitle: false },
      favicon: '/favicon.svg',
      social: [{ icon: 'github', label: 'GitHub', href: repo }],
      editLink: { baseUrl: `${repo}/edit/main/` },
      lastUpdated: false,
      // link previews: the social card on every page
      head: [
        { tag: 'meta', attrs: { property: 'og:image', content: 'https://kranthie.github.io/marvin-bot-mini/og-image.png' } },
        { tag: 'meta', attrs: { property: 'og:image:width', content: '1200' } },
        { tag: 'meta', attrs: { property: 'og:image:height', content: '630' } },
        { tag: 'meta', attrs: { property: 'og:image:alt', content: 'Marvin Bot Mini, an open servo humanoid you can build, in simulation' } },
      ],
      customCss: ['./src/styles/theme.css', './src/styles/guide.css'],
      sidebar: [
        {
          label: 'Start here',
          items: [{ label: 'Home', link: '/' }, 'overview', 'getting-started', 'model', 'plan', 'roadmap', 'requirements', 'glossary'],
        },
        { label: 'Design', items: ['decisions'] },
        { label: 'Simulation', items: ['simulation/isaac-lab', 'simulation/motion-tracking', 'simulation/skills'] },
        { label: 'Build', items: ['build/bom', 'build/assembly', 'build/safety'] },
        { label: 'Learn', items: ['tutorials'] },
        {
          label: 'Project',
          items: ['project/contributing', 'project/acknowledgments', 'project/security', 'project/code-of-conduct', 'project/licences'],
        },
      ],
    }),
  ],
});
