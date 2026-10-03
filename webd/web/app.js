"use strict";

const MAX_VX = 0.3;
const MAX_VY = 0.3;
const MAX_WZ = 1.5;
const INTENT_MS = 100;

const $ = (id) => document.getElementById(id);

let ws;
let nextId = 1;
const pending = new Map();
let joints = [];
let twist = { vx: 0, vy: 0, vyaw: 0 };
let sending = false;
let timer = 0;
let apiVersion = 17;
let rouladeHeld = false;

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

function call(method, params, notify) {
  if (!ws || ws.readyState !== WebSocket.OPEN) {
    return Promise.reject(new Error("还没连上"));
  }
  const msg = { jsonrpc: "2.0", method, params: params || {} };
  if (notify) {
    ws.send(JSON.stringify(msg));
    return Promise.resolve();
  }
  const id = nextId++;
  msg.id = id;
  ws.send(JSON.stringify(msg));
  return new Promise((resolve, reject) => {
    pending.set(id, { resolve, reject });
    setTimeout(() => {
      if (pending.has(id)) {
        pending.delete(id);
        reject(new Error(method + " 没有应答"));
      }
    }, 8000);
  });
}

function fmt(n, digits) {
  if (typeof n !== "number" || !Number.isFinite(n)) return "—";
  return n.toFixed(digits);
}

function attitude(g) {
  if (!g || g.length < 3) return null;
  const [x, y, z] = g;
  const n = Math.hypot(x, y, z) || 1;
  const tilt = Math.acos(Math.min(1, Math.max(-1, -z / n))) * 180 / Math.PI;
  const pitch = Math.atan2(x, -z) * 180 / Math.PI;
  const roll = Math.atan2(y, -z) * 180 / Math.PI;
  return { tilt, pitch, roll, x, y, z };
}

function renderState(s) {
  $("policy").textContent = s.policy || "—";
  const safe = s.safety || {};
  $("safety").textContent = safe.fallen ? "倒下" : (safe.limp ? "卸力" : "正常");
  const att = attitude(safe.gravity);
  if (att) {
    $("gravity").textContent = fmt(att.x, 2) + "  " + fmt(att.y, 2) + "  " + fmt(att.z, 2);
    $("tilt").textContent = "倾角 " + fmt(att.tilt, 1) + "° · 俯仰 " + fmt(att.pitch, 1) + "° · 滚转 " + fmt(att.roll, 1) + "°";
    $("body").style.transform = "rotate(" + (-att.pitch) + "deg)";
    $("horizon").style.transform = "rotate(" + att.roll + "deg)";
  }
  const loop = s.loop || {};
  $("loop").textContent = fmt(loop.hz, 1) + " Hz · 错过 " + (loop.missed ?? "—");
  const mv = s.move || {};
  const ap = mv.applied || [0, 0, 0];
  $("applied").textContent = fmt(ap[0], 2) + " / " + fmt(ap[1], 2) + " / " + fmt(ap[2], 2);
  const lim = mv.limited_by || [];
  $("limits").textContent = lim.length ? lim.join(", ") : "无";

  const body = $("joints");
  const measured = s.joints || [];
  const targets = s.targets || [];
  if (!body.childElementCount && joints.length) fillJoints();
  for (let i = 0; i < joints.length; i++) {
    const row = body.children[i];
    if (!row) break;
    const m = measured[i];
    const t = targets[i];
    row.children[2].textContent = rad(m);
    row.children[3].textContent = rad(t);
    const err = (typeof m === "number" && typeof t === "number") ? (m - t) : 0;
    const cells = row.children[4];
    const mark = cells.querySelector("i");
    const x = Math.max(-1, Math.min(1, err / 0.2));
    mark.style.left = (50 + x * 50) + "%";
    cells.title = rad(err);
  }
}

function rad(v) {
  if (typeof v !== "number" || !Number.isFinite(v)) return "—";
  return (v * 180 / Math.PI).toFixed(1) + "°";
}

function fillJoints() {
  const body = $("joints");
  body.replaceChildren();
  for (const j of joints) {
    const tr = document.createElement("tr");
    tr.innerHTML = "<td></td><td></td><td>—</td><td>—</td><td><div class='bar'><i></i></div></td>";
    tr.children[0].textContent = j.label;
    tr.children[1].textContent = String(j.id);
    body.appendChild(tr);
  }
}

function renderHealth(h) {
  if (!h) return;
  if (h.battery) {
    $("battery").textContent = h.battery.percent.toFixed(0) + "% · " + h.battery.volts.toFixed(2) + " V";
  }
  if (h.motors) {
    $("motors").textContent = h.motors.hottest + " " + h.motors.max_c.toFixed(0) + "°C";
  }
}

function onMessage(raw) {
  let msg;
  try { msg = JSON.parse(raw); } catch { return; }
  if (msg.method === "robot.state" && msg.params) {
    renderState(msg.params);
    return;
  }
  if (msg.id == null) {
    if (msg.error && msg.error.message) say(msg.error.message, "bad");
    return;
  }
  const wait = pending.get(msg.id);
  if (!wait) return;
  pending.delete(msg.id);
  if (msg.error) wait.reject(new Error(msg.error.message || "拒绝"));
  else wait.resolve(msg.result || {});
}

function connect() {
  const url = (location.protocol === "https:" ? "wss://" : "ws://") + location.host + "/ws";
  ws = new WebSocket(url);
  ws.onopen = async () => {
    link("已连接", true);
    try {
      const hello = await call("hello", { api_version: apiVersion });
      say("robotd 接口版本 " + (hello.api_version ?? "—"), "ok");
      await call("robot.subscribe", { hz: 10 });
      const health = await call("robot.health", {});
      renderHealth(health);
    } catch (e) {
      say(e.message, "bad");
    }
  };
  ws.onmessage = (ev) => onMessage(ev.data);
  ws.onclose = () => {
    link("已断开", false);
    say("和 robotd 的连接断了，正在重连…", "bad");
    pending.clear();
    setTimeout(connect, 1500);
  };
  ws.onerror = () => link("连接失败", false);
}

function sendTwist() {
  call("robot.move", twist, true).catch(() => {});
}

function setSending(on) {
  sending = on;
  if (on && !timer) timer = setInterval(sendTwist, INTENT_MS);
  if (!on && timer) {
    clearInterval(timer);
    timer = 0;
  }
}

function zeroTwist() {
  twist = { vx: 0, vy: 0, vyaw: 0 };
  placeKnob($("joy-move"), $("knob-move"), 0, 0);
  placeKnob($("joy-turn"), $("knob-turn"), 0, 0);
  showTwist();
  sendTwist();
  setSending(false);
}

function showTwist() {
  $("vx").textContent = twist.vx.toFixed(2);
  $("vy").textContent = twist.vy.toFixed(2);
  $("wz").textContent = twist.vyaw.toFixed(2);
}

function placeKnob(pad, knob, nx, ny) {
  const r = (pad.clientWidth - knob.clientWidth) / 2;
  const cx = pad.clientWidth / 2 - knob.clientWidth / 2;
  const cy = pad.clientHeight / 2 - knob.clientHeight / 2;
  knob.style.left = (cx + nx * r) + "px";
  knob.style.top = (cy + ny * r) + "px";
}

function bindJoy(pad, knob, apply) {
  const move = (ev) => {
    const rect = pad.getBoundingClientRect();
    const cx = rect.left + rect.width / 2;
    const cy = rect.top + rect.height / 2;
    let dx = (ev.clientX - cx) / (rect.width / 2);
    let dy = (ev.clientY - cy) / (rect.height / 2);
    const mag = Math.hypot(dx, dy);
    if (mag > 1) { dx /= mag; dy /= mag; }
    placeKnob(pad, knob, dx, dy);
    apply(dx, dy);
    showTwist();
    setSending(true);
    sendTwist();
  };
  pad.addEventListener("pointerdown", (ev) => {
    pad.setPointerCapture(ev.pointerId);
    move(ev);
  });
  pad.addEventListener("pointermove", (ev) => {
    if (pad.hasPointerCapture(ev.pointerId)) move(ev);
  });
  const end = (ev) => {
    if (pad.hasPointerCapture(ev.pointerId)) pad.releasePointerCapture(ev.pointerId);
    if (pad === $("joy-move")) { twist.vx = 0; twist.vy = 0; }
    if (pad === $("joy-turn")) twist.vyaw = 0;
    placeKnob(pad, knob, 0, 0);
    showTwist();
    sendTwist();
    if (keys.size === 0 && twist.vx === 0 && twist.vy === 0 && twist.vyaw === 0) setSending(false);
  };
  pad.addEventListener("pointerup", end);
  pad.addEventListener("pointercancel", end);
}

const keys = new Set();
function fromKeys() {
  const vx = (keys.has("w") || keys.has("arrowup") ? 1 : 0) - (keys.has("s") || keys.has("arrowdown") ? 1 : 0);
  const vy = (keys.has("a") || keys.has("arrowleft") ? 1 : 0) - (keys.has("d") || keys.has("arrowright") ? 1 : 0);
  const wz = (keys.has("q") ? 1 : 0) - (keys.has("e") ? 1 : 0);
  twist.vx = vx * MAX_VX;
  twist.vy = vy * MAX_VY;
  twist.vyaw = wz * MAX_WZ;
  placeKnob($("joy-move"), $("knob-move"), -vy, -vx);
  placeKnob($("joy-turn"), $("knob-turn"), -wz, 0);
  showTwist();
  if (vx || vy || wz) {
    setSending(true);
    sendTwist();
  } else {
    sendTwist();
    setSending(false);
  }
}

async function discrete(method, params, label) {
  try {
    const result = await call(method, params);
    if (result && result.accepted === false) {
      say(label + "被拒绝：" + (result.reason || "没有说明"), "bad");
    } else {
      say(label + (result && result.reason ? "：" + result.reason : "已发出"), "ok");
    }
  } catch (e) {
    say(label + "失败：" + e.message, "bad");
  }
}

function sendHead() {
  call("robot.head", {
    neck_pitch: Number($("neck").value),
    head_pitch: Number($("head-pitch").value),
    head_yaw: Number($("head-yaw").value),
    head_roll: Number($("head-roll").value),
  }, true).catch(() => {});
}

function bindSlider(id, out, fn) {
  const el = $(id);
  const show = () => { $(out).textContent = Number(el.value).toFixed(2); fn(); };
  el.addEventListener("input", show);
}

async function boot() {
  try {
    const meta = await fetch("/api/meta").then((r) => r.json());
    apiVersion = meta.api_version || apiVersion;
    joints = meta.joints || [];
    fillJoints();
  } catch (e) {
    say("读不到页面信息：" + e.message, "bad");
  }
  bindJoy($("joy-move"), $("knob-move"), (dx, dy) => {
    twist.vy = -dx * MAX_VY;
    twist.vx = -dy * MAX_VX;
  });
  bindJoy($("joy-turn"), $("knob-turn"), (dx) => {
    twist.vyaw = -dx * MAX_WZ;
  });
  $("init").onclick = () => discrete("robot.init", {}, "起立");
  $("enable").onclick = () => discrete("robot.enable", { on: true, toggle: false }, "开始行走");
  $("hold").onclick = () => discrete("robot.enable", { on: false, toggle: false }, "回到站姿");
  $("stop").onclick = () => { zeroTwist(); discrete("robot.stop", {}, "速度归零"); };
  $("relax").onclick = () => {
    if (!confirm("卸力后如果没人扶着，机器人会倒下。确定卸力？")) return;
    zeroTwist();
    discrete("robot.relax", {}, "卸力");
  };
  document.querySelectorAll("[data-skill]").forEach((btn) => {
    const skill = btn.dataset.skill;
    btn.addEventListener("pointerdown", () => {
      if (skill === "roulade") {
        rouladeHeld = true;
        const pump = () => {
          if (!rouladeHeld) return;
          discrete("robot.do", { skill }, "翻滚");
          setTimeout(pump, 400);
        };
        pump();
      }
    });
    btn.addEventListener("pointerup", () => { rouladeHeld = false; });
    btn.addEventListener("pointerleave", () => { rouladeHeld = false; });
    btn.onclick = () => {
      if (skill === "roulade") return;
      discrete("robot.do", { skill }, btn.textContent.trim());
    };
  });
  bindSlider("neck", "neck-v", sendHead);
  bindSlider("head-pitch", "hp-v", sendHead);
  bindSlider("head-yaw", "hy-v", sendHead);
  bindSlider("head-roll", "hr-v", sendHead);
  bindSlider("mouth", "mouth-v", () => {
    call("robot.mouth", { open: Number($("mouth").value) }, true).catch(() => {});
  });
  addEventListener("keydown", (ev) => {
    if (ev.repeat || ev.target.matches("input, textarea")) return;
    if (ev.code === "Space") { ev.preventDefault(); zeroTwist(); return; }
    keys.add(ev.key.toLowerCase());
    fromKeys();
  });
  addEventListener("keyup", (ev) => {
    keys.delete(ev.key.toLowerCase());
    fromKeys();
  });
  document.addEventListener("visibilitychange", () => {
    if (document.hidden) zeroTwist();
  });
  setInterval(() => {
    call("robot.health", {}).then(renderHealth).catch(() => {});
  }, 2000);
  connect();
}

boot();
