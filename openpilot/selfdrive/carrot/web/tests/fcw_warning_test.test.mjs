import assert from "node:assert/strict";
import fs from "node:fs";
import test from "node:test";
import vm from "node:vm";

const page = fs.readFileSync(new URL("../js/pages/setting.js", import.meta.url), "utf8");
const start = page.indexOf("async function commitSettingValue(");
const end = page.indexOf("function bindPopularDetailRows(", start);
assert.ok(start >= 0 && end > start);

function runtime({ profile = null, failWrite = false, failRead = false } = {}) {
  const pending = [], writes = [], displayed = [], errors = [];
  let actual = 0;
  const context = vm.createContext({
    name: "FcwWarningTest", profile, group: "SYS_SOUND", originGroup: "SYS_SOUND",
    el: { isConnected: true }, val: { dataset: {} }, LANG: "en", UI_STRINGS: { en: {} },
    window: { setTimeout: (callback, delay) => pending.push({ callback, delay }) },
    setParam: async (name, value) => {
      if (failWrite) throw new Error("openpilot OFF required");
      writes.push({ name, value });
      actual = value;
    },
    bulkGet: async () => {
      if (failRead) throw new Error("offline");
      return { FcwWarningTest: actual };
    },
    syncSettingControlState: (_, value) => displayed.push(value),
    cacheSettingValue() {}, refreshSettingHistory() {},
    showAppToast: (message) => errors.push(message),
  });
  vm.runInContext(page.slice(start, end), context);
  return { context, pending, writes, displayed, errors, expire: () => { actual = 0; } };
}

test("live 1 follows acknowledgement and automatically reads back 0", async () => {
  const r = runtime();
  await r.context.commitSettingValue(1);
  assert.deepEqual(r.writes, [{ name: "FcwWarningTest", value: 1 }]);
  assert.deepEqual(r.pending.map((timer) => timer.delay), [250, 1000, 2250, 3000]);
  await r.pending[0].callback();
  assert.equal(r.displayed.at(-1), 1);
  r.expire();
  await r.pending[2].callback();
  assert.equal(r.displayed.at(-1), 0);
  assert.equal(r.context.val.dataset.committedValue, "0");
  assert.equal(r.writes.length, 1, "polling must never send another test");
});

test("cancel uses 0 and a lost status connection never leaves ON displayed", async () => {
  const r = runtime({ failRead: true });
  await r.context.commitSettingValue(0);
  await r.pending[0].callback();
  assert.equal(r.writes[0].value, 0);
  assert.equal(r.displayed.at(-1), 0);
});

test("unsafe rejection and profile edits cannot arm a test", async () => {
  for (const options of [{ failWrite: true }, { profile: { id: "test" } }]) {
    const r = runtime(options);
    await r.context.commitSettingValue(1);
    assert.equal(r.writes.length, 0);
    assert.equal(r.displayed.length, 0);
    assert.equal(r.errors.length, 1);
    assert.equal(r.pending.length, 0);
  }
});

test("polling stops touching a detached settings row", async () => {
  const r = runtime();
  await r.context.commitSettingValue(1);
  r.context.el.isConnected = false;
  r.expire();
  await r.pending[0].callback();
  assert.deepEqual(r.displayed, [1]);
});
