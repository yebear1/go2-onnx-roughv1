(() => {
  "use strict";
  const $ = (selector) => document.querySelector(selector);
  const state = { socket: null, connected: false, authenticated: false, armed: false, move: { x: 0, y: 0 }, yaw: 0, retry: 800 };
  const el = {
    link: $("#linkState"), camera: $("#cameraState"), mode: $("#modeLabel"), battery: $("#battery"), batteryBar: $("#batteryBar"),
    latency: $("#latency"), backend: $("#backend"), vx: $("#vx"), wz: $("#wz"), arm: $("#armButton"), safeStop: $("#safeStopButton"),
    safety: $("#safetyLabel"), safeDot: $(".safe-dot"), watchdog: $("#watchdogLabel"), lock: $("#lockScreen"), pinForm: $("#pinForm"),
    pin: $("#pinInput"), pinError: $("#pinError"), toast: $("#toast"), fullscreen: $("#fullscreenButton"), video: $("#videoFeed")
  };

  function toast(message) { el.toast.textContent = message; el.toast.classList.add("show"); clearTimeout(toast.timer); toast.timer = setTimeout(() => el.toast.classList.remove("show"), 1900); }
  function send(payload) { if (state.socket?.readyState === WebSocket.OPEN) state.socket.send(JSON.stringify(payload)); }
  function setLink(kind, label) { el.link.className = `link-state ${kind}`; el.link.querySelector("span").textContent = label; }
  function setCamera(video) {
    const live = video?.mode === "sdk" && video?.ready && video?.frames > 0 && video.frame_age_ms < 1500;
    const mock = video?.mode === "mock";
    el.camera.className = `camera-state ${live || mock ? "live" : video?.error ? "error" : ""}`;
    el.camera.querySelector("span").textContent = live ? "CAM LIVE" : mock ? "CAM SIM" : video?.error ? "CAM ERROR" : "CAM WAIT";
  }

  let videoRetryTimer = null;
  function reloadVideo(delay = 0) {
    clearTimeout(videoRetryTimer);
    videoRetryTimer = setTimeout(() => {
      if (!state.connected) return;
      el.video.src = `/video?cache=${Date.now()}`;
    }, delay);
  }

  function connect() {
    const protocol = location.protocol === "https:" ? "wss" : "ws";
    const socket = new WebSocket(`${protocol}://${location.host}/ws`);
    state.socket = socket; setLink("", "正在连接桥接");
    socket.onopen = () => { state.connected = true; state.retry = 800; setLink("online", "控制链路在线"); };
    socket.onmessage = ({ data }) => handleMessage(JSON.parse(data));
    socket.onerror = () => socket.close();
    socket.onclose = () => { state.connected = false; state.authenticated = false; setArmed(false); setLink("offline", "控制链路离线"); setTimeout(connect, state.retry); state.retry = Math.min(state.retry * 1.5, 5000); };
  }

  function handleMessage(message) {
    if (message.type === "hello") {
      el.backend.textContent = `${message.backend.toUpperCase()}${message.mock ? " · MOCK" : ""}`;
      el.watchdog.textContent = `FAILSAFE ${message.watchdog_ms}ms`; setCamera(message.video);
      if (message.video?.mode === "sdk" && message.video?.ready) reloadVideo();
      document.querySelectorAll("[data-action]").forEach((button) => button.disabled = !message.actions);
      if (message.pin_required) el.lock.hidden = false; else state.authenticated = true;
    } else if (message.type === "auth") {
      if (message.ok) { state.authenticated = true; el.lock.hidden = true; el.pinError.textContent = ""; }
      else el.pinError.textContent = "PIN 不正确";
    } else if (message.type === "armed") {
      setArmed(message.armed); if (message.reason) toast(message.reason);
    } else if (message.type === "telemetry") updateTelemetry(message);
    else if (message.type === "error") toast(message.message);
  }

  function setArmed(armed) {
    state.armed = armed; el.arm.classList.toggle("armed", armed); el.arm.querySelector("span").textContent = armed ? "控制已解锁" : "长按解锁";
    el.arm.querySelector("small").textContent = armed ? "点按锁定" : "1.2s"; el.safety.textContent = armed ? "控制已激活" : "控制已锁定";
    el.safeDot.classList.toggle("armed", armed); if (!armed) resetSticks();
  }

  function updateTelemetry(message) {
    const d = message.data; el.latency.textContent = Math.max(0, Date.now() - message.server_time); el.mode.textContent = d.mode || "—";
    el.vx.textContent = Number(d.velocity?.[0] || 0).toFixed(2); el.wz.textContent = Number(d.velocity?.[2] || 0).toFixed(2);
    if (d.battery == null) { el.battery.textContent = "—"; el.batteryBar.style.width = "0"; }
    else { el.battery.textContent = `${d.battery}%`; el.batteryBar.style.width = `${d.battery}%`; }
    setCamera(message.video);
  }

  function setupStick(element, onChange) {
    const knob = element.querySelector(".stick-knob"); let pointer = null;
    function update(event) {
      const rect = element.getBoundingClientRect(); const radius = rect.width * .29; let x = event.clientX - rect.left - rect.width / 2; let y = event.clientY - rect.top - rect.height / 2;
      const length = Math.hypot(x, y); if (length > radius) { x = x / length * radius; y = y / length * radius; }
      knob.style.transform = `translate(${x}px,${y}px)`; onChange(x / radius, -y / radius);
    }
    element.addEventListener("pointerdown", (event) => { pointer = event.pointerId; element.setPointerCapture(pointer); update(event); });
    element.addEventListener("pointermove", (event) => { if (event.pointerId === pointer) update(event); });
    const release = (event) => { if (event.pointerId !== pointer) return; pointer = null; knob.style.transform = ""; onChange(0, 0); };
    element.addEventListener("pointerup", release); element.addEventListener("pointercancel", release);
  }

  function resetSticks() { state.move = { x: 0, y: 0 }; state.yaw = 0; document.querySelectorAll(".stick-knob").forEach((knob) => knob.style.transform = ""); $("#moveReadout").textContent = "0.00 · 0.00"; $("#yawReadout").textContent = "0.00"; }
  setupStick($("#moveStick"), (x, y) => { state.move = { x, y }; $("#moveReadout").textContent = `${y.toFixed(2)} · ${x.toFixed(2)}`; });
  setupStick($("#yawStick"), (x) => { state.yaw = x; $("#yawReadout").textContent = x.toFixed(2); });

  let armTimer = null;
  el.arm.addEventListener("pointerdown", (event) => {
    event.preventDefault(); if (state.armed) { send({ type: "disarm" }); return; }
    if (!state.connected || !state.authenticated) { toast("控制链路尚未连接"); return; }
    el.arm.classList.add("holding"); armTimer = setTimeout(() => { el.arm.classList.remove("holding"); navigator.vibrate?.(70); send({ type: "arm" }); }, 1200);
  });
  ["pointerup", "pointercancel", "pointerleave"].forEach((name) => el.arm.addEventListener(name, () => { clearTimeout(armTimer); el.arm.classList.remove("holding"); }));
  el.safeStop.addEventListener("click", () => { send({ type: "safe_stop" }); navigator.vibrate?.(60); });
  document.querySelectorAll("[data-action]").forEach((button) => button.addEventListener("click", () => { if (!state.armed) { toast("请先长按解锁"); return; } send({ type: "action", action: button.dataset.action }); }));
  el.pinForm.addEventListener("submit", (event) => { event.preventDefault(); send({ type: "auth", pin: el.pin.value }); });
  el.fullscreen.addEventListener("click", async () => { try { await document.documentElement.requestFullscreen?.(); await screen.orientation?.lock?.("landscape"); } catch (_) { toast("请在浏览器菜单中开启全屏"); } });

  setInterval(() => {
    if (!state.armed) return; const moving = Math.abs(state.move.x) + Math.abs(state.move.y) + Math.abs(state.yaw) > .002;
    send(moving ? { type: "drive", forward: state.move.y, lateral: -state.move.x, yaw: -state.yaw } : { type: "drive", forward: 0, lateral: 0, yaw: 0 });
  }, 50);
  document.addEventListener("visibilitychange", () => { if (document.hidden && state.armed) send({ type: "disarm" }); });
  window.addEventListener("pagehide", () => send({ type: "safe_stop" }));
  el.video.addEventListener("error", () => {
    el.camera.className = "camera-state error";
    el.camera.querySelector("span").textContent = "CAM RETRY";
    reloadVideo(1200);
  });
  el.video.addEventListener("load", () => clearTimeout(videoRetryTimer));
  connect();
})();
