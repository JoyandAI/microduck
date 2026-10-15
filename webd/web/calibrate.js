"use strict";

const $ = (id) => document.getElementById(id);
const joints = [];
const dragging = [];
let linked = false;
let active = false;
let pollBusy = false;

function say(text, kind) {
  const el = $("message");
  el.textContent = text;
  el.className = "msg" + (kind ? " " + kind : "");
}

async function api(path, body) {
  const res = await fetch(path, {
    method: body == null ? "GET" : "POST",
    headers: body == null ? {} : { "content-type": "application/json" },
    body: body == null ? undefined : JSON.stringify(body),
  });
  const data = await res.json().catch(() => ({}));
  if (!res.ok || data.ok === false) {
    throw new Error(data.error || ("请求失败 " + res.status));
  }
  return data;
}

function row(index) {
  return document.getElementById("row" + index);
}

async function boot() {
  const meta = await fetch("/api/meta").then((r) => r.json());
  if (meta.servo_port) $("port").value = meta.servo_port;
  const list = $("joints");
  (meta.joints || []).forEach((joint, index) => {
    joints[index] = joint;
    dragging[index] = false;
    list.insertAdjacentHTML("beforeend", `<div class="cal-row" id="row${index}">
      <div class="cal-meta"><b>${joint.label}</b><span>ID ${joint.id}</span><span id="raw${index}">编码器 —</span></div>
      <input id="sl${index}" type="range" min="0" max="4095" value="2048" aria-label="${joint.label} 位置">
      <div class="cal-actions">
        <output id="val${index}">2048</output>
        <button type="button" data-zero="${index}">设 0 位</button>
        <button type="button" data-home="${index}">到默认 ${joint.home}</button>
      </div>
    </div>`);
    const slider = $("sl" + index);
    slider.addEventListener("pointerdown", () => { dragging[index] = true; });
    slider.addEventListener("pointerup", () => { dragging[index] = false; });
    slider.addEventListener("pointercancel", () => { dragging[index] = false; });
    slider.addEventListener("input", () => {
      $("val" + index).textContent = slider.value;
      sendGoto(joint.id, Number(slider.value));
    });
  });

  document.querySelectorAll("[data-zero]").forEach((btn) => {
    btn.onclick = async () => {
      const index = Number(btn.dataset.zero);
      const joint = joints[index];
      const current = $("raw" + index).textContent.replace("编码器 ", "");
      if (!confirm("把「" + joint.label + "」当前位置（" + current + "）写成 0 位？这一步保存在舵机里。")) return;
      try {
        const data = await api("/api/cal/zero", { id: joint.id });
        row(index).classList.add("done");
        showCount(index, data.position);
        say(joint.label + " 的 0 位已写成 " + data.position, "ok");
      } catch (e) { say(e.message, "bad"); }
    };
  });
  document.querySelectorAll("[data-home]").forEach((btn) => {
    btn.onclick = async () => {
      const joint = joints[Number(btn.dataset.home)];
      try {
        await api("/api/cal/default", { id: joint.id });
        say(joint.label + " 正在走到默认位置 " + joint.home, "ok");
      } catch (e) { say(e.message, "bad"); }
    };
  });

  $("connect").onclick = async () => {
    try {
      const data = await api("/api/servo/connect", { port: $("port").value.trim() });
      linked = true;
      $("link").textContent = "已连接 " + data.port;
      $("link").className = "pill ok";
      say("串口已打开。可以开始标定。", "ok");
    } catch (e) {
      linked = false;
      $("link").textContent = "未连接";
      $("link").className = "pill bad";
      say(e.message, "bad");
    }
  };
  $("disconnect").onclick = async () => {
    if (active) await exitCal();
    await api("/api/servo/disconnect", {});
    linked = false;
    $("link").textContent = "未连接";
    $("link").className = "pill";
    say("串口已断开。", "ok");
  };
  $("start").onclick = async () => {
    try {
      await api("/api/cal/enter", {});
      active = true;
      joints.forEach((_, index) => showCount(index, 2048));
      say("标定已开始：KP=32、KD=40，目标 2048。摆到机械零位后设 0 位。", "ok");
    } catch (e) { say(e.message, "bad"); }
  };
  $("home-all").onclick = async () => {
    try {
      await api("/api/cal/default", {});
      say("十五路正在走到站姿默认位置。", "ok");
    } catch (e) { say(e.message, "bad"); }
  };
  $("save-home").onclick = async () => {
    if (!confirm("把当前十五路位置写成默认站姿？这一步写进 robotd.toml 的 policy.home。策略的观测零点也会跟着变，保存后要重新启动 robotd。")) return;
    try {
      const data = await api("/api/cal/home", {});
      (data.home || []).forEach((count, index) => {
        if (!joints[index]) return;
        joints[index].home = count;
        const button = document.querySelector('[data-home="' + index + '"]');
        if (button) button.textContent = "到默认 " + count;
      });
      say("已写入 " + data.path + "。重新启动 robotd 后，站姿和策略零点都会用这组角度。", "ok");
    } catch (e) { say(e.message, "bad"); }
  };
  $("exit").onclick = () => exitCal();
}

function showCount(index, position) {
  $("raw" + index).textContent = "编码器 " + position;
  if (!dragging[index]) {
    $("sl" + index).value = String(position);
    $("val" + index).textContent = String(position);
  }
}

let gotoTail = Promise.resolve();
function sendGoto(id, position) {
  gotoTail = gotoTail
    .catch(() => {})
    .then(() => api("/api/servo/goto", { id, position }))
    .catch((e) => say(e.message, "bad"));
}

async function exitCal() {
  try {
    await api("/api/cal/exit", {});
    active = false;
    say("已退出。力矩已卸，增益恢复为 KP 32、KD 40。", "ok");
  } catch (e) { say(e.message, "bad"); }
}

window.addEventListener("pagehide", () => {
  if (!linked && !active) return;
  navigator.sendBeacon("/api/cal/exit", new Blob([JSON.stringify({})], { type: "application/json" }));
});

setInterval(async () => {
  if (!linked || pollBusy || document.hidden) return;
  pollBusy = true;
  try {
    const data = await api("/api/cal/positions");
    (data.positions || []).forEach((sample, index) => {
      if (sample && sample.position != null) showCount(index, sample.position);
    });
  } catch (e) {
    if (linked) say(e.message, "bad");
  } finally {
    pollBusy = false;
  }
}, 400);

boot().catch((e) => say(e.message, "bad"));
