"use strict";

const $ = (id) => document.getElementById(id);

let unit = "robotd";
let following = false;
let paused = false;
let ws;
let daemons = [];

function say(text, kind) {
  const el = $("message");
  el.textContent = text;
  el.className = "msg" + (kind ? " " + kind : "");
}

function link(text, ok) {
  const el = $("link");
  el.textContent = text;
  el.className = "pill" + (ok === true ? " ok" : ok === false ? " bad" : "");
}

function query() {
  const params = new URLSearchParams({
    unit,
    stream: $("stream").value,
    lines: $("lines").value,
  });
  return params.toString();
}

function formatTs(us) {
  if (!us) return "";
  const d = new Date(us / 1000);
  if (Number.isNaN(d.getTime())) return "";
  const p = (n) => String(n).padStart(2, "0");
  return p(d.getHours()) + ":" + p(d.getMinutes()) + ":" + p(d.getSeconds());
}

function priClass(p) {
  if (p <= 3) return "err";
  if (p <= 4) return "warn";
  return "info";
}

function wanted(line) {
  const q = $("filter").value.trim().toLowerCase();
  if (!q) return true;
  return (line.unit + " " + line.stream + " " + line.message).toLowerCase().includes(q);
}

function renderLine(line) {
  if (!wanted(line)) return;
  const row = document.createElement("div");
  row.className = "log-line " + priClass(line.priority || 6);
  const meta = document.createElement("span");
  meta.className = "log-meta";
  meta.textContent = formatTs(line.ts) + " " + (line.unit || "?") + " " + (line.stream || "") + " ";
  const msg = document.createElement("span");
  msg.className = "log-msg";
  msg.textContent = line.message || "";
  row.appendChild(meta);
  row.appendChild(msg);
  const box = $("log");
  box.appendChild(row);
  while (box.childElementCount > 4000) box.removeChild(box.firstChild);
  if (!paused) box.scrollTop = box.scrollHeight;
}

function renderMany(lines) {
  const box = $("log");
  const stick = !paused && box.scrollHeight - box.scrollTop - box.clientHeight < 40;
  const frag = document.createDocumentFragment();
  for (const line of lines) {
    if (!wanted(line)) continue;
    const row = document.createElement("div");
    row.className = "log-line " + priClass(line.priority || 6);
    const meta = document.createElement("span");
    meta.className = "log-meta";
    meta.textContent = formatTs(line.ts) + " " + (line.unit || "?") + " " + (line.stream || "") + " ";
    const msg = document.createElement("span");
    msg.className = "log-msg";
    msg.textContent = line.message || "";
    row.appendChild(meta);
    row.appendChild(msg);
    frag.appendChild(row);
  }
  box.appendChild(frag);
  while (box.childElementCount > 4000) box.removeChild(box.firstChild);
  if (stick) box.scrollTop = box.scrollHeight;
}

function paintDaemons() {
  const host = $("daemons");
  host.replaceChildren();
  const all = document.createElement("button");
  all.textContent = "全部";
  all.className = unit === "all" ? "primary" : "";
  all.onclick = () => select("all");
  host.appendChild(all);
  for (const d of daemons) {
    const b = document.createElement("button");
    const state = d.active || "";
    b.textContent = d.id + (state && state !== "unknown" ? " · " + state : "");
    b.className = unit === d.id ? "primary" : "";
    if (state === "failed") b.classList.add("danger");
    b.onclick = () => select(d.id);
    host.appendChild(b);
  }
}

function select(id) {
  unit = id;
  paintDaemons();
  if (following) startFollow();
  else reload();
}

async function loadDaemons() {
  try {
    const r = await fetch("/api/logs/daemons");
    const data = await r.json();
    daemons = data.daemons || [];
    paintDaemons();
  } catch (e) {
    say("读不到进程列表：" + e.message, "bad");
  }
}

async function reload() {
  stopFollow();
  $("log").replaceChildren();
  say("正在读取 " + unit + " …");
  try {
    const r = await fetch("/api/logs?" + query());
    const data = await r.json();
    if (!r.ok) {
      say(data.error || "读取失败", "bad");
      return;
    }
    renderMany(data.lines || []);
    const n = $("log").childElementCount;
    if (data.hint) say(data.hint, n ? "" : "bad");
    else say("已显示 " + n + " 条（" + unit + "）", "ok");
    $("log").scrollTop = $("log").scrollHeight;
  } catch (e) {
    say("读取失败：" + e.message, "bad");
  }
}

function stopFollow() {
  following = false;
  if (ws) {
    ws.close();
    ws = null;
  }
  $("follow").textContent = "跟随";
  link("未跟随");
}

function startFollow() {
  stopFollow();
  following = true;
  paused = false;
  $("pause").textContent = "暂停";
  $("follow").textContent = "停止跟随";
  $("log").replaceChildren();
  const proto = location.protocol === "https:" ? "wss" : "ws";
  ws = new WebSocket(proto + "://" + location.host + "/ws/logs?" + query());
  link("正在跟随…");
  ws.onopen = () => link("跟随中", true);
  ws.onclose = () => {
    if (following) {
      link("跟随断开", false);
      say("跟随断开，已停在最后一条。", "bad");
      following = false;
      $("follow").textContent = "跟随";
    }
  };
  ws.onerror = () => say("跟随出错", "bad");
  ws.onmessage = (ev) => {
    if (paused) return;
    let data;
    try { data = JSON.parse(ev.data); } catch { return; }
    if (data.hint) {
      say(data.hint, "bad");
      return;
    }
    if (data.lines) renderMany(data.lines);
    else renderLine(data);
  };
}

$("reload").onclick = reload;
$("follow").onclick = () => {
  if (following) {
    stopFollow();
    say("已停止跟随");
  } else startFollow();
};
$("pause").onclick = () => {
  paused = !paused;
  $("pause").textContent = paused ? "继续" : "暂停";
};
$("clear").onclick = () => $("log").replaceChildren();
$("stream").onchange = () => select(unit);
$("lines").onchange = () => select(unit);
$("filter").oninput = () => {
  /* filter applies to newly arriving lines; reload to re-filter history */
};

loadDaemons().then(reload);
