# Marvin Bot Mini documentation site

The project website, published to GitHub Pages at <https://kranthie.github.io/marvin-bot-mini/>.
It is Marvin Bot's site without its interactive guide, which it links to instead. It has three
parts:

| Part | Where it comes from |
|---|---|
| Home page: the robot in 3-D | `src/pages/index.astro` and `src/scripts/marvin-viewer.js` |
| The 3-D model page (layers, assembly, scripted skills, joints) | `src/content/docs/model.mdx`, with its viewer, controls and styles in `src/model/` |
| Documentation | the repository's own Markdown (`README.md`, `docs/`, `sim/*/README.md`, `bom/README.md`, `CONTRIBUTING.md`, …), copied in at build time by `scripts/sync-docs.mjs`; plus two pages written for the site (`src/content/docs/getting-started.md`, `tutorials.md`) |

It is built with [Astro Starlight](https://starlight.astro.build) (MIT) and three.js (MIT).
Dependencies are pinned in `package.json` and `package-lock.json`.

## Editing

- **Docs:** edit the Markdown in the repository, never the copies in `src/content/docs/` (they are
  generated and git-ignored; the list is in `src/content/docs/.gitignore`). Relative links between
  synced files become site links; links to other repository files go to GitHub; images are copied.
  To add a document to the site, add it to `PAGES` in `scripts/sync-docs.mjs` and to the sidebar
  in `astro.config.mjs`.
- **The 3-D model:** `scripts/build_model.py` packs the generated URDF, `description/cad_model.yaml`
  and the code-CAD meshes into `public/guide/model/marvin_model.js` (git-ignored), used by the home
  page and the 3-D model page. Both viewers share `src/scripts/marvin-render.js`: smooth shading
  that keeps sharp edges, image-based lighting and soft shadows, and the printed parts' colours.
  The Pages workflow rebuilds it on every deploy, so it always matches the design.

## Building and previewing locally

You need Node.js 22.22 or later and the repository's Python environment (for the 3-D model).

```bash
pip install -e ".[cad]" && python -m cad.build && python site/scripts/build_model.py
cd site
npm ci
npm run dev            # http://localhost:4321/marvin-bot-mini/
npm run build          # the static site in dist/
npm run check-links    # every internal link, image and #anchor in dist/ resolves
```

`.github/workflows/pages.yml` does the same on every push to `main` that touches the site, the docs
or the design, and deploys `dist/` to GitHub Pages.
