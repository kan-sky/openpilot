import assert from "node:assert/strict";
import { readFileSync, existsSync } from "node:fs";
import test from "node:test";
import vm from "node:vm";

const read = (path) => readFileSync(new URL(`../${path}`, import.meta.url), "utf8");

test("device support panels load before their binders without restoring retired bundles", () => {
  const html = read("index.html");
  for (const name of ["remote_support", "jetson"]) {
    assert.ok(html.indexOf(`/js/pages/${name}.js`) > 0);
    assert.ok(html.indexOf(`/js/pages/${name}.js`) < html.indexOf("/js/pages/setting_device_actions.js"));
  }
  assert.ok(html.includes('/css/pages/remote_support.css'));
  assert.ok(html.includes('/js/generated/logs.js'));
  assert.ok(html.includes('/css/generated/settings.css'));
  for (const name of ["css/pages/settings/base.css", "css/pages/settings/device.css", "css/pages/settings/panels.css", "js/pages/logs/dashcam.js"]) {
    assert.equal(existsSync(new URL(`../${name}`, import.meta.url)), false);
  }
});

function panelContext() {
  const elements = new Map();
  const element = (id) => {
    if (!elements.has(id)) elements.set(id, { textContent: "", disabled: false, dataset: {}, removeAttribute() {} });
    return elements.get(id);
  };
  const context = vm.createContext({
    URL, document: { hidden: false, addEventListener() {}, getElementById: element, querySelectorAll: () => [] },
    CURRENT_PAGE: "setting", CURRENT_SETTING_TAB: "device", CURRENT_DEVICE_GROUP: "Jetson",
    clearTimeout() {}, setTimeout() {},
  });
  for (const name of ["remote_support", "jetson"]) vm.runInContext(read(`js/pages/${name}.js`), context);
  return { context, element };
}

test("the login link allowlist rejects injected origins and credentials", () => {
  const { context } = panelContext();
  for (const url of ["javascript:alert(1)", "http://login.tailscale.com/a/x", "https://login.tailscale.com.evil.test/a/x", "https://user@login.tailscale.com/a/x"]) {
    assert.equal(context.remoteLoginUrl(url), "");
  }
  assert.equal(context.remoteLoginUrl("https://login.tailscale.com/a/test-only"), "https://login.tailscale.com/a/test-only");
});

test("Jetson panel clears stale health and disables unpaired management", () => {
  const { context, element } = panelContext();
  context.updateJetsonPanel({ supported: true, paired: true, jetlink: {
    active: true, fresh: true, badge: "jetSON", hud_connected: true,
    health: { fresh: true, temp_c: 42, addresses: ["192.0.2.2"] },
  } });
  assert.match(element("jetsonTemp").textContent, /42/);
  assert.equal(element("jetsonIps").textContent, "192.0.2.2");
  assert.equal(element("jetsonCheck").disabled, false);
  context.updateJetsonPanel({ supported: false, paired: false, jetlink: { health: { fresh: false, temp_c: 42, addresses: ["192.0.2.2"] } } });
  assert.doesNotMatch(element("jetsonTemp").textContent, /42/);
  assert.doesNotMatch(element("jetsonIps").textContent, /192/);
  assert.equal(element("jetsonCheck").disabled, true);
  assert.equal(element("jetsonApply").disabled, true);
});

test("late panel responses do not modify the next view", () => {
  const { context, element } = panelContext();
  context.CURRENT_PAGE = "drive";
  context.updateJetsonPanel({ jetlink: { badge: "jetSON ERROR" } });
  assert.equal(element("jetsonBadge").textContent, "");
});
