import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import test from "node:test";
import vm from "node:vm";
import { nextStepValue } from "../src/ui/components/numeric_stepper/numeric_stepper.js";

const source = readFileSync(new URL("../js/pages/setting.js", import.meta.url), "utf8");
const catalog = JSON.parse(readFileSync(new URL("../../../carrot_settings.json", import.meta.url), "utf8"));
const ratio = catalog.params.find((item) => item.name === "VehicleCenterToFrontRatio");
const context = vm.createContext({
  SETTING_CONTROL_KINDS: ["toggle", "segmented", "select", "slider"],
  getDeclaredSettingOptionLabel: () => null,
  getSettingDisplayUnit: () => "",
  findSettingItemByName: (name) => ({ item: catalog.params.find((item) => item.name === name) }),
  clamp: (value, min, max) => Math.max(min, Math.min(value, max)),
});
vm.runInContext(source.slice(source.indexOf("function getSettingControlConfig("),
  source.indexOf("const SETTING_DISPLAY_UNIT_TYPES")), context);
vm.runInContext(source.slice(source.indexOf("function formatSettingDisplayValue("),
  source.indexOf("function formatSettingRangeMeta(")), context);
vm.runInContext(source.slice(source.indexOf("function normalizeSettingValue("),
  source.indexOf("async function refreshSettingHistory(")), context);

test("ratio metadata and control preserve decimal units without percent", () => {
  assert.equal(ratio.min, 0.1);
  assert.equal(ratio.max, 0.9);
  assert.equal(ratio.unit, 0.01);
  assert.equal(ratio.display_unit, undefined);
  const config = context.getSettingControlConfig(ratio);
  assert.equal(config.unit, 0.01);
  assert.equal(config.kind, "slider");
  const next = nextStepValue({ current: 0.7, step: config.unit, sign: 1, min: ratio.min, max: ratio.max });
  assert.equal(Number(next.toFixed(6)), 0.71);
});

test("ratio display preserves fractional inputs and at least two decimals", () => {
  assert.equal(context.formatSettingDisplayValue(ratio, 0.7), "0.70");
  assert.equal(context.formatSettingDisplayValue(ratio, 0.735), "0.735");
  assert.equal(context.formatSettingDisplayValue(ratio, 0.1), "0.10");
  assert.equal(context.formatSettingDisplayValue({ name: ratio.name }, 0.7), "0.70");
  assert.equal(context.formatSettingDisplayValue({ name: "VehicleMass" }, 2090), "2090");
});

test("manual vehicle-spec entry rejects out-of-range or fractional mass instead of clamping", () => {
  context.p = ratio;
  context.name = ratio.name;
  assert.equal(context.normalizeSettingValue("0.735"), 0.735);
  assert.equal(context.normalizeSettingValue("70"), null);
  assert.equal(context.normalizeSettingValue("0"), null);
  context.p = catalog.params.find((item) => item.name === "VehicleMass");
  context.name = context.p.name;
  assert.equal(context.normalizeSettingValue("2090"), 2090);
  assert.equal(context.normalizeSettingValue("2090.2"), null);
  assert.equal(context.normalizeSettingValue("0"), null);
});
