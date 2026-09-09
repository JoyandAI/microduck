#!/usr/bin/env node
// build.mjs — 把 lessons/*.md 预渲染成自包含离线 index.html
//
// 用法：node build.mjs
// 产物：index.html（双击即可在浏览器打开，file:// 无 CORS 问题，无需联网）
//
// 为什么预渲染而不是运行时 fetch：
//   浏览器用 file:// 打开页面时禁止 fetch() 同目录文件，所以把所有讲正文
//   在构建期就内联进一个 HTML。Markdown 源文件保留在 lessons/ 便于 grep/编辑。
//
// 样式约束：只依赖系统字体 + 少量 CSS，不引 CDN，保证离线。

import { readFileSync, readdirSync, writeFileSync } from 'node:fs'
import { join } from 'node:path'
import markedPkg from './assets/marked.min.js'

const ROOT = new URL('.', import.meta.url).pathname
const LESSONS_DIR = join(ROOT, 'lessons')
const OUT = join(ROOT, 'index.html')

const marked = markedPkg.marked ?? markedPkg.default ?? markedPkg
const markdown = marked
markdown.setOptions({ gfm: true, breaks: false })

// —— 头部渲染器：给每个标题分配稳定 id，供侧栏/页内跳转 ——
const slugify = (s, lessonKey) =>
  lessonKey +
  '-' +
  s
    .toLowerCase()
    .replace(/[^\p{L}\p{N}]+/gu, '-')
    .replace(/^-+|-+$/g, '')

function renderLesson(md, lessonKey) {
  const renderer = new markdown.Renderer()
  renderer.heading = ({ tokens, depth }) => {
    const text = tokens.map((t) => t.text ?? '').join('')
    const id = slugify(text, lessonKey)
    const size = depth === 1 ? '1' : depth === 2 ? '2' : '3'
    return `<h${depth} id="${id}" class="h-${depth}">${text}</h${depth}>`
  }
  return markdown.parse(md, { renderer })
}

// —— 收集侧栏结构：每讲的 H1 作讲名，H2 作小节 ——
function collectToc(md, lessonKey) {
  const lines = md.split('\n')
  const toc = { title: lessonKey, subs: [] }
  for (const line of lines) {
    const m = line.match(/^(#{1,2})\s+(.*)$/)
    if (!m) continue
    const text = m[2].trim()
    if (m[1] === '#') toc.title = text
    else toc.subs.push({ text, id: slugify(text, lessonKey) })
  }
  return toc
}

const files = readdirSync(LESSONS_DIR)
  .filter((f) => f.endsWith('.md'))
  .sort()

const lessons = []
for (const f of files) {
  const key = f.replace(/\.md$/, '')
  const md = readFileSync(join(LESSONS_DIR, f), 'utf8')
  lessons.push({ key, file: f, md, toc: collectToc(md, key) })
}

// —— 侧栏 ——
const navItems = lessons
  .map(
    (l) => `
    <li class="nav-lesson">
      <a class="nav-l" href="#${l.key}" data-key="${l.key}">${escapeHtml(l.toc.title)}</a>
      <ul class="nav-subs">
        ${l.toc.subs
          .map((s) => `<li><a href="#${s.id}">${escapeHtml(s.text)}</a></li>`)
          .join('\n        ')}
      </ul>
    </li>`
  )
  .join('\n')

// —— 正文 ——
const sections = lessons
  .map(
    (l, i) => `
  <section class="lesson" id="${l.key}" data-key="${l.key}">
    ${renderLesson(l.md, l.key)}
    <nav class="lesson-nav">
      ${i > 0 ? `<a href="#${lessons[i - 1].key}">← ${escapeHtml(lessons[i - 1].toc.title)}</a>` : '<span></span>'}
      ${i < lessons.length - 1 ? `<a href="#${lessons[i + 1].key}">${escapeHtml(lessons[i + 1].toc.title)} →</a>` : '<span></span>'}
    </nav>
  </section>`
  )
  .join('\n')

function escapeHtml(s) {
  return s.replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;')
}

const html = `<!doctype html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>microduck 源码精读</title>
<style>
  :root{
    --bg:#f7f7f5; --panel:#ffffff; --ink:#1f2328; --mut:#57606a; --line:#d8dee4;
    --acc:#b45309; --code:#f6f8fa; --codeink:#24292f; --link:#0a66c2;
  }
  *{box-sizing:border-box}
  html{scroll-behavior:smooth}
  body{margin:0;font:15px/1.7 -apple-system,BlinkMacSystemFont,"Segoe UI","PingFang SC","Hiragino Sans GB","Microsoft YaHei",sans-serif;color:var(--ink);background:var(--bg)}
  a{color:var(--link);text-decoration:none} a:hover{text-decoration:underline}
  .wrap{display:flex;min-height:100vh}
  nav.side{position:sticky;top:0;height:100vh;overflow-y:auto;width:280px;flex:0 0 280px;
    background:var(--panel);border-right:1px solid var(--line);padding:18px 12px}
  nav.side .brand{font-weight:700;font-size:15px;color:var(--acc);margin:0 6px 4px}
  nav.side .tag{font-size:12px;color:var(--mut);margin:0 6px 14px;line-height:1.5}
  nav.side ul{list-style:none;margin:0;padding:0}
  nav.side .nav-lesson{margin:2px 0}
  nav.side a.nav-l{display:block;padding:4px 6px;border-radius:6px;font-weight:600;color:var(--ink)}
  nav.side a.nav-l:hover{background:#efeae2}
  nav.side a.nav-l.on{background:#f3e5cd;color:var(--acc)}
  nav.side .nav-subs{display:none;padding-left:10px;border-left:1px solid var(--line);margin:0 0 6px 12px}
  nav.side .nav-subs li{margin:1px 0}
  nav.side .nav-subs a{display:block;font-size:12.5px;color:var(--mut);padding:1px 4px}
  nav.side .nav-subs a:hover{color:var(--ink)}
  main{flex:1;min-width:0;padding:28px 40px 80px;max-width:980px}
  .lesson{background:var(--panel);border:1px solid var(--line);border-radius:10px;padding:8px 40px 28px;margin-bottom:28px}
  .lesson h1{font-size:24px;margin:22px 0 4px;padding-top:10px;border-bottom:2px solid #eee3d3;padding-bottom:10px}
  .lesson h2{font-size:19px;margin:30px 0 8px;padding-top:14px;border-top:1px solid #ece7df}
  .lesson h3{font-size:16px;margin:22px 0 6px}
  .lesson h1 .hno{color:var(--acc)}
  .lesson p,.lesson li{color:#24292f}
  .lesson code{font:12.5px/1.5 ui-monospace,SFMono-Regular,Menlo,Consolas,"Liberation Mono",monospace;
    background:var(--code);color:var(--codeink);padding:1.5px 5px;border-radius:4px}
  .lesson pre{background:var(--code);border:1px solid var(--line);border-radius:8px;padding:12px 14px;
    overflow-x:auto;line-height:1.55}
  .lesson pre code{background:none;padding:0;font-size:12.5px}
  .lesson table{border-collapse:collapse;width:100%;margin:12px 0;font-size:13.5px;display:block;overflow-x:auto}
  .lesson th,.lesson td{border:1px solid var(--line);padding:5px 9px;text-align:left;vertical-align:top}
  .lesson th{background:#faf6ef;white-space:nowrap}
  .lesson blockquote{margin:10px 0;padding:6px 14px;border-left:4px solid var(--acc);background:#fbf7f0;color:#4a3b24;border-radius:0 6px 6px 0}
  .lesson blockquote p{margin:4px 0}
  .lesson hr{border:none;border-top:1px solid var(--line);margin:22px 0}
  .lesson img{max-width:100%}
  .lesson-nav{display:flex;justify-content:space-between;gap:8px;margin-top:26px;padding-top:12px;border-top:1px solid var(--line);font-size:13.5px}
  @media (max-width:860px){
    .wrap{flex-direction:column}
    nav.side{position:static;height:auto;width:100%;border-right:none;border-bottom:1px solid var(--line);max-height:38vh}
    main{padding:18px 12px 60px}
    .lesson{padding:4px 18px 20px}
  }
</style>
</head>
<body>
<div class="wrap">
  <nav class="side" id="side">
    <p class="brand">microduck 源码精读</p>
    <p class="tag">官方 Rust 运行时 · 无鸭也能读 · 读到能改</p>
    <ul>${navItems}</ul>
  </nav>
  <main>
    ${sections}
  </main>
</div>
<script>
// 极简交互：滚动时高亮当前讲、展开/收起小节、点击讲名直达。
const navLinks = [...document.querySelectorAll('a.nav-l')]
const subsLists = [...document.querySelectorAll('.nav-subs')]
const sections = [...document.querySelectorAll('.lesson')]
function onScroll(){
  let cur = sections[0]
  for(const s of sections){ if(s.getBoundingClientRect().top <= 90) cur = s }
  const key = cur && cur.dataset.key
  navLinks.forEach(a=>{ a.classList.toggle('on', a.dataset.key===key) })
  subsLists.forEach((ul,idx)=>{ ul.style.display = navLinks[idx].dataset.key===key ? 'block':'none' })
}
document.addEventListener('scroll', onScroll, {passive:true})
onScroll()
</script>
</body>
</html>
`

writeFileSync(OUT, html)
const total = lessons.reduce((s, l) => s + l.md.length, 0)
console.log(`✅ 已生成 ${OUT}`)
console.log(`   共 ${lessons.length} 讲，正文 ${(total/1024).toFixed(1)} KB`)
console.log(`   双击 index.html 即可浏览。`)
