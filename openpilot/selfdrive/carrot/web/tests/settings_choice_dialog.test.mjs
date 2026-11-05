import assert from "node:assert/strict";
import fs from "node:fs";
import path from "node:path";
import test from "node:test";
import vm from "node:vm";

import {
  resolveSettingChoiceLayout,
} from "../src/features/settings/choice_dialog.js";
import { createDialogController } from "../src/ui/components/dialog/dialog.js";

const root = path.resolve(import.meta.dirname, "..");
const settings = JSON.parse(fs.readFileSync(path.join(root, "..", "..", "carrot_settings.json"), "utf8"));

function settingItems(value, found = new Map()) {
  if (Array.isArray(value)) {
    value.forEach((item) => settingItems(item, found));
    return found;
  }
  if (!value || typeof value !== "object") return found;
  if (value.name && value.control === "select") found.set(value.name, value);
  Object.values(value).forEach((item) => settingItems(item, found));
  return found;
}

test("semantic settings select choices use the wrapping list in every locale", () => {
  const selects = settingItems(settings);
  const semantic = [...selects.values()].filter((item) => item.options);
  assert.ok(semantic.length > 0);
  for (const item of semantic) {
    for (const locale of ["ko", "en", "zh"]) {
      assert.ok(Array.isArray(item.options[locale]), `${item.name} has ${locale} labels`);
    }
    assert.equal(resolveSettingChoiceLayout(item), "list", item.name);
  }
  assert.equal(resolveSettingChoiceLayout(selects.get("SoundLanguageSetting")), "list");
});

test("raw numeric setting ranges keep the compact value grid", () => {
  const selects = settingItems(settings);
  const numeric = [...selects.values()].filter((item) => !item.options && item.name !== "SoundLanguageSetting");
  assert.ok(numeric.length > 0);
  for (const item of numeric) {
    assert.equal(resolveSettingChoiceLayout(item), "value-grid", item.name);
  }
});

test("Navdy connection choices preserve legacy mode and label both transports", () => {
  const mode = settingItems(settings).get("NavdyConnectionMode");
  assert.ok(mode);
  assert.deepEqual([mode.min, mode.max, mode.default, mode.unit], [0, 2, 0, 1]);
  assert.deepEqual(mode.options, {
    ko: ["기존 설정", "유선 USB", "무선 Bluetooth"],
    en: ["Existing configuration", "Wired USB", "Wireless Bluetooth"],
    zh: ["现有配置", "有线 USB", "无线 Bluetooth"],
  });
  assert.equal(resolveSettingChoiceLayout(mode), "list");
});

function navdyChoiceRenderer(language, current) {
  const source = fs.readFileSync(path.join(root, "js/pages/setting.js"), "utf8");
  const mode = settings.params.find((item) => item.name === "NavdyConnectionMode");
  const makeNode = () => ({
    hidden: true, children: [], attributes: {}, listeners: {},
    style: { removeProperty() {}, setProperty() {} },
    classList: { add() {}, remove() {}, toggle() {} },
    setAttribute(name, value) { this.attributes[name] = value; },
    set innerHTML(value) { this.html = value; this.children = []; },
    appendChild(child) { this.children.push(child); },
    addEventListener(name, callback) { this.listeners[name] = callback; },
  });
  const nodes = Object.fromEntries([
    "appDialog", "appDialogTitle", "appDialogBody", "appDialogChoices", "appDialogConfirm", "appDialogCancel",
  ].map((id) => [id, makeNode()]));
  const dialog = createDialogController({
    target: {},
    document: { getElementById: (id) => nodes[id], createElement: makeNode },
    setTimeout: (callback) => { queueMicrotask(callback); return 1; },
    requestAnimationFrame: () => 1,
    cancelAnimationFrame() {},
    createFocusTrap: () => ({ activate() {}, deactivate() {} }),
  });
  const committed = [];
  const context = vm.createContext({
    LANG: language, p: mode, name: mode.name,
    title: mode[{ ko: "title", en: "etitle", zh: "ctitle" }[language]],
    val: { dataset: { rawValue: current } },
    window: { CarrotUI: { settingsChoiceDialog: { resolveLayout: resolveSettingChoiceLayout } } },
    findSettingItemByName: (name) => ({ item: settings.params.find((item) => item.name === name) }),
    getUIText: (key, fallback) => fallback,
    escapeHtml: (text) => String(text),
    clamp: (value, min, max) => Math.max(min, Math.min(value, max)),
    openAppDialog: dialog.openAppDialog,
    commitSettingValue: async (value) => { committed.push(value); },
  });
  // Run the shipped page's choice construction/formatting and shared renderer.
  for (const [start, end] of [
    ["const SETTING_CONTROL_KINDS", "function formatSettingRangeMeta("],
    ["function getSettingOptionValues(", "function syncSettingControlState("],
    ["function normalizeSettingValue(", "async function refreshSettingHistory("],
    ["async function promptSettingChoice(", "if (toggleInput) {"],
  ]) {
    const from = source.indexOf(start);
    const to = source.indexOf(end, from);
    assert.ok(from >= 0 && to > from);
    vm.runInContext(source.slice(from, to), context);
  }
  context.controlConfig = context.getSettingControlConfig(mode);
  return { context, nodes, committed, dialog, mode };
}

for (const language of ["ko", "en", "zh"]) {
  test(`actual Navdy choice renderer displays and commits localized options: ${language}`, async () => {
    for (const current of [0, 1, 2, "0", "1", "2"]) {
      const { context, nodes, committed, mode } = navdyChoiceRenderer(language, current);
      const completion = context.promptSettingChoice();
      const buttons = nodes.appDialogChoices.children;
      assert.equal(nodes.appDialog.hidden, false);
      assert.equal(nodes.appDialogChoices.className, "app-dialog__choices app-dialog__choices--list");
      assert.deepEqual(buttons.map((button) => button.textContent), mode.options[language]);
      assert.deepEqual(buttons.map((button) => button.attributes["aria-current"] === "true"),
        [0, 1, 2].map((value) => value === Number(current)));
      assert.ok(nodes.appDialogBody.html.includes(mode.options[language][Number(current)]));
      const next = (Number(current) + 1) % 3;
      buttons[next].listeners.click();
      await completion;
      assert.deepEqual(committed, [next]);
      assert.equal(nodes.appDialog.hidden, true);
    }
  });
}

test("actual Navdy choice renderer never writes on cancel or unchanged choice", async () => {
  for (const cancel of [false, true]) {
    const { context, nodes, committed, dialog } = navdyChoiceRenderer("en", "2");
    const completion = context.promptSettingChoice();
    if (cancel) await dialog.cancelAppDialog();
    else nodes.appDialogChoices.children[2].listeners.click();
    await completion;
    assert.deepEqual(committed, []);
  }
});

test("a reviewed setting can explicitly choose its presentation", () => {
  assert.equal(resolveSettingChoiceLayout({ choice_layout: "list" }), "list");
  assert.equal(resolveSettingChoiceLayout({ choice_layout: "value-grid" }), "value-grid");
});
