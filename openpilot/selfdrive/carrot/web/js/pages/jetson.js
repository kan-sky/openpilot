"use strict";

let jetsonPanelState = null;
let jetsonPanelTimer = null;
let jetsonPanelLoading = false;
let jetsonPanelDirty = false;

function renderJetsonPanel() {
  return `<section id="jetsonPanel" class="remote-support" aria-label="Jetson 상태 및 설정">
    <div class="remote-support__heading"><h3>Jetson</h3><span id="jetsonBadge" role="status">확인 중</span></div>
    <dl class="remote-support__details">
      <div><dt>주행 추론</dt><dd id="jetsonInference">-</dd></div>
      <div><dt>모델</dt><dd id="jetsonModel">-</dd></div>
      <div><dt>기기</dt><dd id="jetsonDevice">-</dd></div>
      <div><dt>TensorRT</dt><dd id="jetsonRuntime">-</dd></div>
      <div><dt>온도</dt><dd id="jetsonTemp">-</dd></div>
      <div><dt>IP</dt><dd id="jetsonIps">-</dd></div>
      <div><dt>모델 통신</dt><dd id="jetsonTransport">-</dd></div>
      <div><dt>외장 HUD</dt><dd id="jetsonHud">-</dd></div>
      <div><dt>USB 관리</dt><dd id="jetsonUsb">-</dd></div>
      <div><dt>SSH</dt><dd id="jetsonSsh">-</dd></div>
    </dl>
    <p id="jetsonDetail" class="remote-support__message" role="status"></p>
    <p id="jetsonError" class="remote-support__error" role="alert"></p>
    <div class="remote-support__actions"><button type="button" class="smallBtn" id="jetsonCheck" disabled>SSH 연결 확인</button></div>
    <div class="remote-support__heading"><h3>설정</h3></div>
    <form id="jetsonSettings" class="remote-support__form">
      <label for="jetsonDisplay">외장 계기판</label>
      <select id="jetsonDisplay" class="input-text device-select"><option value="0">끄기</option><option value="1">TURZX 9.2인치</option><option value="2">TURZX 12.3인치</option></select>
      <label for="jetsonTheme">계기판 테마</label>
      <select id="jetsonTheme" class="input-text device-select"><option value="0">자동</option><option value="1">다크</option><option value="2">라이트</option></select>
      <label class="remote-support__checkbox"><input type="checkbox" id="jetsonUsbEnabled">USB 관리 연결 · 다음 재부팅 적용</label>
      <div class="remote-support__actions"><button type="submit" class="smallBtn btn--filled" id="jetsonApply" disabled>설정 적용</button></div>
    </form>
    <p id="jetsonOperation" class="remote-support__message" role="status" aria-live="polite"></p>
  </section>`;
}

function jetsonPanelActive() {
  return CURRENT_PAGE === "setting" && CURRENT_SETTING_TAB === "device" && CURRENT_DEVICE_GROUP === "Jetson"
    && !document.hidden && Boolean(document.getElementById("jetsonPanel"));
}

function updateJetsonPanel(value) {
  jetsonPanelState = value;
  if (!jetsonPanelActive()) return;
  const link = value.jetlink || {};
  const health = link.health || {};
  const busy = value.operation?.state === "running";
  remoteText("jetsonBadge", link.badge || (link.fresh ? "연결 대기" : "현재 데이터 없음"));
  remoteText("jetsonInference", link.active ? "Jetson 추론 사용 중" : "Jetson 추론 미사용");
  remoteText("jetsonModel", link.model);
  remoteText("jetsonDevice", link.device);
  remoteText("jetsonRuntime", link.runtime);
  remoteText("jetsonTemp", health.fresh && Number.isFinite(health.temp_c) ? `${health.temp_c.toFixed(1)} °C` : "현재 데이터 없음");
  remoteText("jetsonIps", health.fresh ? (health.addresses || []).join(", ") : "현재 데이터 없음");
  remoteText("jetsonTransport", link.transport ? `${link.transport.toUpperCase()} · 프로토콜 ${link.protocol ?? "-"}` : "-");
  remoteText("jetsonHud", link.hud_connected === true ? "연결됨" : link.hud_connected === false ? "미연결" : "미확인");
  remoteText("jetsonUsb", value.usb?.state === "configured" ? `${value.usb.address} → ${value.usb.peer}` : value.usb?.state || "미설정");
  remoteText("jetsonSsh", value.ssh?.connected ? `마지막 확인: ${value.ssh.transport} · ${value.ssh.hostname}` : value.ssh ? "연결 실패" : value.paired ? "등록됨 · 미확인" : "최초 등록 필요");
  remoteText("jetsonDetail", link.detail || health.reason || " ");
  remoteText("jetsonError", value.operation?.state === "error" ? value.operation.error : " ");
  if (!jetsonPanelDirty && !busy) {
    document.getElementById("jetsonDisplay").value = String(value.settings?.ClusterHud ?? 0);
    document.getElementById("jetsonTheme").value = String(value.settings?.ClusterHudTheme ?? 0);
    document.getElementById("jetsonUsbEnabled").checked = Boolean(value.settings?.usb_management);
  }
  document.getElementById("jetsonCheck").disabled = busy || !value.paired;
  document.getElementById("jetsonApply").disabled = busy || !value.supported || !jetsonPanelDirty;
  for (const id of ["jetsonDisplay", "jetsonTheme", "jetsonUsbEnabled"]) document.getElementById(id).disabled = busy || !value.supported;
  const op = value.operation || {};
  remoteText("jetsonOperation", busy ? "처리 중…" : op.state === "done" && op.action === "jetson-settings"
    ? op.result?.usb_restart_required ? "저장됨 · USB 변경은 주차 후 콤마 재부팅 시 적용됩니다." : "저장됨" : " ");
}

async function refreshJetsonPanel() {
  if (jetsonPanelTimer) clearTimeout(jetsonPanelTimer);
  if (!jetsonPanelActive() || jetsonPanelLoading) return;
  jetsonPanelLoading = true;
  try { updateJetsonPanel(await getJson("/api/jetson")); }
  catch (error) {
    remoteText("jetsonError", error.message);
    remoteText("jetsonBadge", "상태 조회 실패");
    for (const id of ["jetsonTemp", "jetsonIps", "jetsonInference", "jetsonHud"]) remoteText(id, "현재 데이터 없음");
    document.querySelectorAll("#jetsonPanel button, #jetsonPanel select, #jetsonPanel input").forEach((el) => { el.disabled = true; });
  } finally {
    jetsonPanelLoading = false;
    if (jetsonPanelActive()) jetsonPanelTimer = setTimeout(refreshJetsonPanel, 2000);
  }
}

async function jetsonAction(action, body = {}) {
  try {
    const result = await requestJson("/api/remote_support", {
      method: "POST", headers: { "Content-Type": "application/json", "X-HL-CSRF": jetsonPanelState.csrf },
      body: JSON.stringify({ action, confirmed: true, ...body }),
    });
    updateJetsonPanel({ ...jetsonPanelState, operation: result.operation });
    await refreshJetsonPanel();
    return true;
  } catch (error) { remoteText("jetsonError", error.message); return false; }
}

function bindJetsonPanel(container) {
  jetsonPanelDirty = false;
  container.querySelector("#jetsonSettings").onchange = () => {
    jetsonPanelDirty = true;
    if (jetsonPanelState) updateJetsonPanel(jetsonPanelState);
  };
  container.querySelector("#jetsonSettings").onsubmit = async (event) => {
    event.preventDefault();
    const values = { ClusterHud: Number(document.getElementById("jetsonDisplay").value), ClusterHudTheme: Number(document.getElementById("jetsonTheme").value), usb_management: document.getElementById("jetsonUsbEnabled").checked };
    const settings = Object.fromEntries(Object.entries(values).filter(([key, value]) => value !== jetsonPanelState.settings?.[key]));
    if (!Object.keys(settings).length) return;
    if (Object.hasOwn(settings, "usb_management") && !await appConfirm("USB 관리 설정은 다음 콤마 재부팅 때 적용됩니다. 지금 USB 연결이나 Jetlink를 재시작하지 않습니다. 저장할까요?", { title: "USB 관리 설정" })) return;
    if (await jetsonAction("jetson-settings", { settings })) jetsonPanelDirty = false;
  };
  container.querySelector("#jetsonCheck").onclick = () => jetsonAction("jetson-check");
  refreshJetsonPanel();
}

document.addEventListener("visibilitychange", () => { if (jetsonPanelActive()) refreshJetsonPanel(); });
