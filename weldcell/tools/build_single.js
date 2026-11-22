#!/usr/bin/env node
/* Bundle index.html + css + js (+ vendored three.js) into ONE offline HTML file,
 * like the original single-file v21 distribution.
 *   node tools/build_single.js   ->  dist/weldcell_v22.html                       */
'use strict';
const fs = require('fs'), path = require('path');
const root = path.join(__dirname, '..');
let html = fs.readFileSync(path.join(root, 'index.html'), 'utf8');
const read = (p) => fs.readFileSync(path.join(root, p), 'utf8');
html = html.replace(/<link rel="stylesheet" href="([^"]+)">/g, (m, href) => `<style>\n${read(href)}\n</style>`);
html = html.replace(/<script src="([^"]+)"><\/script>/g, (m, src) => `<script>/* ${src} */\n${read(src).replace(/<\/script/gi, '<\\/script')}\n</script>`);
html = html.replace('href="docs/manual.html"', 'href="weldcell_v22_manual.html"');
fs.mkdirSync(path.join(root, 'dist'), { recursive: true });
// manual next to the bundle; its "open tool" links point at the bundle
const man = read('docs/manual.html').replace(/\.\.\/index\.html/g, 'weldcell_v22.html');
fs.writeFileSync(path.join(root, 'dist', 'weldcell_v22_manual.html'), man);
const out = path.join(root, 'dist', 'weldcell_v22.html');
fs.writeFileSync(out, html);
console.log(`wrote ${path.relative(root, out)} (${(html.length / 1024).toFixed(0)} KB)`);
