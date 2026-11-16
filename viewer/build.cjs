// Generate portable, offline HTML files from the editable viewer sources.
const fs = require('node:fs');
const path = require('node:path');
const root = path.join(__dirname, '..');
const template = fs.readFileSync(path.join(__dirname, 'index.html'), 'utf8');
const html = template.replace('/* INLINE_STYLE */', fs.readFileSync(path.join(__dirname, 'viewer.css'), 'utf8'))
  .replace('/* INLINE_SCRIPT */', fs.readFileSync(path.join(__dirname, 'viewer.js'), 'utf8'));
for (const name of ['kart_log_view_mecanum_v4.html', 'kart_log_view_mecanum_v3.html']) {
  fs.writeFileSync(path.join(root, name), html, 'utf8');
}
console.log('Built standalone v3 / v4 viewers.');
