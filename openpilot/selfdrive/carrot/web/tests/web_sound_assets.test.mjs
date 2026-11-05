import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import test from "node:test";
import vm from "node:vm";

const source = readFileSync(new URL("../js/realtime/web_sound.js", import.meta.url), "utf8");
const manifest = JSON.parse(readFileSync(new URL("../../../assets/sounds/wayon_audio.json", import.meta.url), "utf8"));

test("every web sound maps to the complete Wayon asset set", () => {
  const context = vm.createContext({});
  vm.runInContext(source.slice(source.indexOf("  const SOUND_MAP"), source.indexOf("  let enabled")) +
    "globalThis.soundMap = SOUND_MAP;", context);
  for (const spec of Object.values(context.soundMap)) {
    assert.ok(Object.hasOwn(manifest.mapping, spec.file), spec.file);
  }
  assert.equal(manifest.mapping[context.soundMap[22].file], "warning_immediate.wav");
  assert.equal(manifest.mapping["engage_tizi.wav"], "engage_tizi.wav");
  assert.equal(manifest.mapping["disengage_tizi.wav"], "disengage_tizi.wav");
});

for (const directory of manifest.directories) {
  test(`web revalidates ${directory} once, reuses decoded buffers and retries errors`, async () => {
    const requests = [];
    let decodeCount = 0;
    let ok = true;
    const payload = new ArrayBuffer(4);
    const buffer = {};
    const context = vm.createContext({
      bufferCache: new Map(), readyBuffers: new Map(), soundDirectory: () => directory,
      ensureContext: () => ({ decodeAudioData: async (data) => {
        assert.equal(data, payload);
        decodeCount++;
        return buffer;
      } }),
      fetch: async (url, options) => {
        requests.push({ url, options });
        return { ok, status: ok ? 200 : 503, arrayBuffer: async () => payload };
      },
    });
    vm.runInContext(source.slice(source.indexOf("  async function loadBuffer("), source.indexOf("  function preloadSounds(")), context);
    assert.equal(await context.loadBuffer("audio_stopstop.wav"), buffer);
    assert.equal(await context.loadBuffer("audio_stopstop.wav"), buffer);
    assert.equal(requests.length, 1);
    assert.equal(decodeCount, 1);
    assert.equal(requests[0].url, `/sound-assets/${directory}/audio_stopstop.wav`);
    assert.equal(requests[0].options.cache, "no-cache");
    assert.equal(context.readyBuffers.get(`${directory}/audio_stopstop.wav`), buffer);
    ok = false;
    await assert.rejects(context.loadBuffer("prompt.wav"), /503/);
    assert.equal(context.bufferCache.has(`${directory}/prompt.wav`), false);
    ok = true;
    assert.equal(await context.loadBuffer("prompt.wav"), buffer);
    assert.equal(requests.length, 3);
    assert.equal(decodeCount, 2);
  });
}
