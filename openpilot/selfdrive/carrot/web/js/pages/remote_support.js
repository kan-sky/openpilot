"use strict";

let remoteSupportState = null;
let remoteSupportTimer = null;
let remoteSupportLoading = false;
let remoteSupportActionError = "";
let remoteSupportPowerPending = false;

function renderRemoteSupportPanel() {
  return `<section id="remoteSupportPanel" class="remote-support" aria-label="원격지원">
    <div class="remote-support__heading"><h3>Tailscale</h3><span id="remoteConnectionState" role="status">확인 중</span></div>
    <label class="device-toggle remote-support__power" for="remotePower">
      <span class="device-toggle__text">Tailscale 연결</span>
      <span id="remotePowerState" aria-hidden="true">-</span>
      <span class="ctrl"><input id="remotePower" type="checkbox" role="switch" aria-label="Tailscale 연결" class="device-toggle__input" disabled><span class="device-toggle__slider"></span></span>
    </label>
    <dl class="remote-support__details">
      <div><dt>계정</dt><dd id="remoteAccount">-</dd></div>
      <div><dt>기기</dt><dd id="remoteHostname">-</dd></div>
      <div><dt>접속 주소</dt><dd id="remoteAddress">-</dd></div>
      <div><dt>설치</dt><dd id="remoteProvider">-</dd></div>
    </dl>
    <div class="remote-support__actions">
      <button type="button" class="smallBtn btn--filled" id="remoteInstall" disabled>설치</button>
      <button type="button" class="smallBtn btn--filled" id="remoteLogin" disabled>로그인</button>
      <button type="button" class="smallBtn" id="remoteSwitch" disabled>계정 변경</button>
    </div>
    <a id="remoteLoginLink" class="remote-support__login smallBtn btn--filled" hidden target="_blank" rel="noopener noreferrer">본인 계정으로 로그인</a>
    <p id="remoteOperation" class="remote-support__message" role="status" aria-live="polite"></p>
    <p id="remoteError" class="remote-support__error" role="alert"></p>
    <div class="remote-support__heading"><h3>기기 공유</h3></div>
    <div class="remote-support__actions"><a class="smallBtn" href="https://login.tailscale.com/admin/machines" target="_blank" rel="noopener noreferrer">Tailscale 기기 관리</a></div>
    <div class="remote-support__heading"><h3>지원용 SSH 키</h3><span id="remoteSshState">-</span></div>
    <label class="remote-support__field" for="remotePublicKey">지원자 공개키 · ssh-ed25519</label>
    <textarea id="remotePublicKey" rows="3" spellcheck="false" autocapitalize="off" autocomplete="off" placeholder="ssh-ed25519 AAAA..."></textarea>
    <div class="remote-support__actions"><button type="button" class="smallBtn" id="remoteAddKey" disabled>키 확인 및 등록</button></div>
    <ul id="remoteKeys" class="remote-support__keys"></ul>
    <div class="remote-support__heading"><h3>Jetson</h3><span id="remoteJetsonState">미확인</span></div>
    <div class="remote-support__actions"><button type="button" class="smallBtn" id="remoteJetsonCheck" disabled>SSH 연결 확인</button></div>
  </section>`;
}

function remotePanelActive() {
  return CURRENT_PAGE === "setting" && CURRENT_SETTING_TAB === "device" && CURRENT_DEVICE_GROUP === "RemoteSupport"
    && !document.hidden && Boolean(document.getElementById("remoteSupportPanel"));
}

function remoteText(id, value) {
  const element = document.getElementById(id);
  if (element) element.textContent = value || "-";
}

function remoteLoginUrl(value) {
  try {
    const url = new URL(value);
    return url.origin === "https://login.tailscale.com" && url.pathname.startsWith("/a/") && !url.username && !url.password ? url.href : "";
  } catch { return ""; }
}

function updateRemoteSupport(value) {
  remoteSupportState = value;
  if (!remotePanelActive()) return;
  const net = value.network || {};
  const busy = value.operation?.state === "running" || remoteSupportPowerPending;
  const disabled = !value.supported || busy;
  const labels = { Running: net.online ? "연결됨" : "인터넷 확인 필요", Starting: "연결 중", NeedsLogin: "로그인 필요", Stopped: "연결 중지", NeedsMachineAuth: "기기 승인 필요", NotInstalled: "미설치", Unavailable: "서비스 확인 필요" };
  remoteText("remoteConnectionState", labels[net.backend] || "확인 필요");
  remoteText("remoteAccount", net.account || (net.backend === "Stopped" ? "저장된 계정 확인 필요" : "로그인 안 됨"));
  remoteText("remoteHostname", net.hostname);
  remoteText("remoteAddress", (net.addresses || []).filter((ip) => !ip.includes(":")).join(", "));
  remoteText("remoteProvider", { hl: "HL", wayon: "기존 Wayon", system: "기존 시스템" }[net.provider] || "미설치");
  remoteText("remoteSshState", value.ssh?.enabled ? "SSH 켜짐" : "SSH 꺼짐");
  const power = document.getElementById("remotePower");
  power.checked = ["Running", "Starting", "NeedsMachineAuth"].includes(net.backend);
  power.disabled = disabled || !net.installed || !["Running", "Starting", "NeedsMachineAuth", "Stopped"].includes(net.backend);
  remoteText("remotePowerState", power.checked ? "켜짐" : net.backend === "Stopped" ? "꺼짐" : "-");
  document.getElementById("remoteInstall").disabled = disabled || Boolean(net.installed);
  document.getElementById("remoteLogin").disabled = disabled || !net.installed || !["NeedsLogin", "NoState"].includes(net.backend);
  document.getElementById("remoteSwitch").disabled = disabled || !net.installed || !net.account;
  document.getElementById("remoteAddKey").disabled = disabled;
  document.getElementById("remoteJetsonCheck").disabled = disabled || !value.jetson_paired;
  const link = document.getElementById("remoteLoginLink");
  const url = remoteLoginUrl(net.auth_url);
  link.hidden = !url || net.backend === "Running";
  if (!link.hidden) link.href = url;
  else link.removeAttribute("href");
  const operation = value.operation || {};
  const actions = { install: "설치", login: "로그인 연결", "switch-account": "계정 변경", "network-on": "Tailscale 켜기", "network-off": "Tailscale 끄기", "add-key": "키 등록", "remove-key": "키 제거", "jetson-check": "Jetson 확인", "jetson-settings": "Jetson 설정" };
  const awaitingLogin = ["login", "switch-account"].includes(operation.action) && net.backend !== "Running";
  remoteText("remoteOperation", operation.state === "running" ? `${actions[operation.action]} 진행 중…`
    : operation.state === "done" ? awaitingLogin ? "로그인 승인 대기" : `${actions[operation.action]} 완료` : " ");
  const health = (net.health || []).filter((warning) => !(net.backend === "Stopped" && warning === "Tailscale is stopped."));
  remoteText("remoteError", remoteSupportActionError || operation.error || net.error || health.join("\n") || (!value.supported ? "AGNOS 콤마에서만 사용할 수 있습니다." : " "));
  const keys = document.getElementById("remoteKeys");
  const signature = JSON.stringify([value.ssh?.keys || [], disabled]);
  if (keys.dataset.signature !== signature) {
    keys.dataset.signature = signature;
    keys.innerHTML = (value.ssh?.keys || []).map((key) => `<li><code>${escapeHtml(key.fingerprint)}</code><button type="button" class="smallBtn btn--danger" data-remove-support-key="${escapeHtml(key.fingerprint)}" ${disabled ? "disabled" : ""}>제거</button></li>`).join("");
  }
  remoteText("remoteJetsonState", value.jetson?.connected ? `마지막 확인: ${value.jetson.transport} · ${value.jetson.hostname}`
    : value.jetson ? "SSH 연결 실패" : value.jetson_paired ? "등록됨 · 연결 미확인" : "최초 SSH 등록 필요");
}

async function refreshRemoteSupport() {
  if (remoteSupportTimer) clearTimeout(remoteSupportTimer);
  if (!remotePanelActive() || remoteSupportLoading) return;
  remoteSupportLoading = true;
  try {
    updateRemoteSupport(await getJson("/api/remote_support"));
  } catch (error) {
    remoteText("remoteError", error.message);
    remoteText("remoteConnectionState", "상태 조회 실패");
    const link = document.getElementById("remoteLoginLink");
    if (link) { link.hidden = true; link.removeAttribute("href"); }
    document.querySelectorAll("#remoteSupportPanel button, #remotePower").forEach((control) => { control.disabled = true; });
  } finally {
    remoteSupportLoading = false;
    if (remotePanelActive()) remoteSupportTimer = setTimeout(refreshRemoteSupport, 4000);
  }
}

async function remoteSupportAction(action, extra = {}) {
  if (!remoteSupportState?.csrf) throw new Error("연결 상태를 먼저 확인해 주세요.");
  return requestJson("/api/remote_support", {
    method: "POST", headers: { "Content-Type": "application/json", "X-HL-CSRF": remoteSupportState.csrf },
    body: JSON.stringify({ action, confirmed: true, ...extra }),
  });
}

async function runRemoteAction(action, extra = {}) {
  remoteSupportActionError = "";
  try {
    const result = await remoteSupportAction(action, extra);
    updateRemoteSupport({ ...remoteSupportState, operation: result.operation });
    await refreshRemoteSupport();
  } catch (error) { remoteSupportActionError = error.message; remoteText("remoteError", error.message); }
}

function bindRemoteSupportPanel(container) {
  remoteSupportActionError = "";
  container.querySelector("#remotePower").onchange = async (event) => {
    const enabled = event.target.checked;
    remoteSupportPowerPending = true;
    updateRemoteSupport(remoteSupportState);
    try {
      if (enabled || await appConfirm("Tailscale 원격 접속이 끊깁니다. 로그인 계정은 유지되며 콤마와 Jetson의 USB 통신은 계속됩니다. 끌까요?", { title: "Tailscale 끄기", confirmLabel: "끄기" })) {
        await runRemoteAction(enabled ? "network-on" : "network-off");
      }
    } finally {
      remoteSupportPowerPending = false;
      updateRemoteSupport(remoteSupportState);
      await refreshRemoteSupport();
    }
  };
  container.querySelector("#remoteInstall").onclick = async () => {
    if (await appConfirm("이 콤마에 Tailscale을 설치하고 부팅 시 자동 실행할까요? 기존 계정과 Jetlink 설정은 변경하지 않습니다.", { title: "원격지원 설치" })) await runRemoteAction("install");
  };
  container.querySelector("#remoteLogin").onclick = () => runRemoteAction("login");
  container.querySelector("#remoteSwitch").onclick = async () => {
    const account = remoteSupportState?.network?.account || "";
    if (await appConfirm(`${account}\n\n현재 계정에서 로그아웃합니다. 기존 Tailscale 원격 접속이 즉시 끊기며, 새 계정에서 기기를 다시 공유해야 합니다. 변경할까요?`, { title: "Tailscale 계정 변경", confirmLabel: "로그아웃 후 변경" })) {
      await runRemoteAction("switch-account", { account });
    }
  };
  container.querySelector("#remoteAddKey").onclick = async () => {
    remoteSupportActionError = "";
    try {
      const publicKey = document.getElementById("remotePublicKey").value.trim();
      const preview = await remoteSupportAction("key-preview", { public_key: publicKey });
      const confirmed = await appConfirm(`${preview.fingerprint}\n\n이 키 소유자에게 콤마 SSH 관리 권한을 허용합니다. 기존 소유자 키는 유지됩니다. 지원자와 지문을 대조했나요?`, { title: "SSH 접근 승인", confirmLabel: "접근 허용" });
      if (confirmed) await runRemoteAction("add-key", preview);
    } catch (error) { remoteSupportActionError = error.message; remoteText("remoteError", error.message); }
  };
  container.querySelector("#remoteKeys").onclick = async (event) => {
    const button = event.target.closest("[data-remove-support-key]");
    if (button && await appConfirm("이 지원 키의 SSH 접근 권한을 제거할까요? 기존 연결을 즉시 종료하지는 않습니다.", { title: "지원 키 제거" })) {
      await runRemoteAction("remove-key", { fingerprint: button.dataset.removeSupportKey });
    }
  };
  container.querySelector("#remoteJetsonCheck").onclick = () => runRemoteAction("jetson-check");
  refreshRemoteSupport();
}

document.addEventListener("visibilitychange", () => { if (remotePanelActive()) refreshRemoteSupport(); });
