import { defineConfig } from 'vite';

// Dev serves at '/', production build is based at '/vrm-constraints/' so it works
// as a GitHub Pages project site (webn00b.github.io/vrm-constraints/).
export default defineConfig(({ command }) => ({
  base: command === 'build' ? '/vrm-constraints/' : '/',
  server: { port: 5180, open: true },
  build: { target: 'es2020' },
}));
