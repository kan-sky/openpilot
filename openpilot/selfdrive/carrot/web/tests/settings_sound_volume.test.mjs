import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import test from "node:test";
import vm from "node:vm";
import { nextStepValue } from "../src/ui/components/numeric_stepper/numeric_stepper.js";

const source = readFileSync(new URL("../js/pages/setting.js", import.meta.url), "utf8");
const catalog = JSON.parse(readFileSync(new URL("../../../carrot_settings.json", import.meta.url), "utf8"));
const volume = catalog.params.find((item) => item.name === "SoundVolumeAdjust");
const context = vm.createContext({
  LANG: "ko",
  getUIText: (key) => {
    assert.equal(key, "automatic");
    return { ko: "자동", en: "Automatic", zh: "自动" }[context.LANG];
  },
  findSettingItemByName: (name) => ({ item: catalog.params.find((item) => item.name === name) }),
  clamp: (value, min, max) => Math.max(min, Math.min(value, max)),
});
vm.runInContext(source.slice(source.indexOf("const SETTING_DISPLAY_UNIT_TYPES"),
  source.indexOf("function formatSettingRangeMeta(")), context);
vm.runInContext(source.slice(source.indexOf("function normalizeSettingValue("),
  source.indexOf("async function refreshSettingHistory(")), context);

test("only general volume -1 uses the translated Automatic label", () => {
  for (const [language, label] of Object.entries({ ko: "자동", en: "Automatic", zh: "自动" })) {
    context.LANG = language;
    assert.equal(context.formatSettingDisplayValue(volume, -1), label);
    assert.equal(context.formatSettingDisplayValue({ name: volume.name }, "-1"), label);
    for (const value of [0, 5, 30, 100, 200]) {
      assert.equal(context.formatSettingDisplayValue(volume, value), `${value}%`);
    }
    assert.equal(context.formatSettingDisplayValue({ name: "SoundVolumeAdjustEngage" }, 10), "10%");
  }
});

test("manual input and reload retain -1, without granting engage an auto mode", () => {
  context.p = volume;
  context.name = volume.name;
  for (const value of [-1, 0, 5, 100, 200]) {
    assert.equal(context.normalizeSettingValue(String(value)), value);
  }
  assert.equal(context.normalizeSettingValue("-20"), -1);
  context.p = catalog.params.find((item) => item.name === "SoundVolumeAdjustEngage");
  context.name = context.p.name;
  assert.equal(context.normalizeSettingValue("-1"), 5);
});

test("default x1 stepper can reach auto and return to mute", () => {
  const step = (current, sign, multiplier = 1) => nextStepValue({ current, sign, step: multiplier, min: volume.min, max: volume.max });
  assert.equal(step(0, -1), -1);
  assert.equal(step(-1, -1), -1);
  assert.equal(step(-1, 1), 0);
  assert.equal(step(5, -1, 5), 0);
  assert.equal(step(0, -1, 5), -1);
  assert.equal(step(200, 1), 200);
});
