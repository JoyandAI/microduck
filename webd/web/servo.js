"use strict";

const $ = (id) => document.getElementById(id);

function say(text, kind) {
  const el = $("message");
  el.textContent = text;
  el.className = "msg" + (kind ? " " + kind : "");
}

async function api(path, body) {
  const res = await fetch(path, {
    method: body ? "POST" : "GET",
    headers: body ? { "content-type": "application/json" } : {},
    body: body ? JSON.stringify(body) : undefined,
  });
  const data = await res.json().catch(() => ({}));
  if (!res.ok || data.ok === false) {
    throw new Error(data.error || ("请求失败 " + res.status));
  }
  return data;
}

function selectedId() {
  return Number($("id").value);
}

function showSample(s) {
  $("pos").textContent = s.position + " / 4095";
  $("deg").textContent = s.degrees == null ? "—" : s.degrees.toFixed(1) + "°";
  $("volt").textContent = s.volts.toFixed(2) + " V";
  $("temp").textContent = s.temperature_c + "°C";
  $("load").textContent = s.load_percent.toFixed(0) + "%";
  $("gains-now").textContent = s.kp + " / " + s.kd;
  $("torque").textContent = s.torque ? "开" : "关";
  $("kp").value = s.kp;
  $("kd").value = s.kd;
}

async function boot() {
  const meta = await fetch("/api/meta").then((r) => r.json());
  if (meta.servo_port) $("port").value = meta.servo_port;
  const sel = $("id");
  for (const j of meta.joints || []) {
    const opt = document.createElement("option");
    opt.value = String(j.id);
    opt.textContent = j.id + " · " + j.label;
    sel.appendChild(opt);
  }
  $("connect").onclick = async () => {
    try {
      const data = await api("/api/servo/connect", { port: $("port").value.trim() });
      $("link").textContent = "已连接 " + data.port;
      $("link").className = "pill ok";
      say("串口已打开。先扫描，或直接读取当前 ID。", "ok");
    } catch (e) {
      $("link").textContent = "未连接";
      $("link").className = "pill bad";
      say(e.message, "bad");
    }
  };
  $("disconnect").onclick = async () => {
    await api("/api/servo/disconnect", {});
    $("link").textContent = "未连接";
    $("link").className = "pill";
    say("串口已断开。", "ok");
  };
  $("scan").onclick = async () => {
    try {
      const data = await api("/api/servo/scan");
      const ids = data.online || [];
      $("online").textContent = ids.length ? ("在线：" + ids.join(", ")) : "没有舵机应答";
      say(ids.length + " 颗在线", ids.length ? "ok" : "bad");
    } catch (e) { say(e.message, "bad"); }
  };
  $("read").onclick = async () => {
    try {
      showSample(await api("/api/servo/read/" + selectedId()));
      say("已读取 ID " + selectedId(), "ok");
    } catch (e) { say(e.message, "bad"); }
  };
  $("torque-on").onclick = () => torque(true);
  $("torque-off").onclick = () => torque(false);
  $("go").onclick = async () => {
    const position = Number($("goto").value);
    if (!confirm("ID " + selectedId() + " 将走到 " + position + "，并打开力矩。扶好了吗？")) return;
    try {
      await api("/api/servo/goto", { id: selectedId(), position });
      say("已写入目标 " + position, "ok");
      showSample(await api("/api/servo/read/" + selectedId()));
    } catch (e) { say(e.message, "bad"); }
  };
  $("set-gains").onclick = async () => {
    try {
      await api("/api/servo/gains", {
        id: selectedId(),
        kp: Number($("kp").value),
        kd: Number($("kd").value),
      });
      say("已写入 KP/KD", "ok");
      showSample(await api("/api/servo/read/" + selectedId()));
    } catch (e) { say(e.message, "bad"); }
  };
  $("set-id").onclick = async () => {
    const id = selectedId();
    const newId = Number($("new-id").value);
    if (!confirm("把 ID " + id + " 改成 " + newId + "？总线上只能有这一颗，而且这是 EEPROM。")) return;
    try {
      await api("/api/servo/id", { id, new_id: newId });
      say("ID 已改为 " + newId + "。下拉框请改选新 ID 再读。", "ok");
    } catch (e) { say(e.message, "bad"); }
  };
}

async function torque(on) {
  try {
    await api("/api/servo/torque", { id: selectedId(), on });
    say(on ? "力矩已打开" : "力矩已关掉", "ok");
    showSample(await api("/api/servo/read/" + selectedId()));
  } catch (e) { say(e.message, "bad"); }
}

boot().catch((e) => say(e.message, "bad"));
