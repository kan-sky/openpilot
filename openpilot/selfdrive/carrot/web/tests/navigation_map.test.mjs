import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import test from "node:test";
import vm from "node:vm";

const source = await readFile(new URL("../js/pages/navigation_map.js", import.meta.url), "utf8");
const html = await readFile(new URL("../navigation.html", import.meta.url), "utf8");
const index = await readFile(new URL("../index.html", import.meta.url), "utf8");
const settle = () => new Promise((resolve) => setImmediate(resolve));
const deferred = () => {
  let resolve, reject;
  const promise = new Promise((yes, no) => { resolve = yes; reject = no; });
  return { promise, resolve, reject };
};
const destination = { place_name: "Seoul Station", latitude: 37.55, longitude: 126.97 };
const initial = {
  destination: null, key_configured: true, map_ready: false, model_input_ready: false,
  pipeline_status: "model_status_missing", navigation_source: "network", source_scope: "configured",
  destination_editable: true, reason: "model_status_missing",
  map_navigation_source: "mapbox", map_route_id: "123",
};
function response(value, status = 200, headers = {}, blob = { size: 12 }) {
  if (headers["X-Nav-Map-Status"] === "ready") {
    headers = { "X-Nav-Map-Source": "mapbox", "X-Nav-Map-Route": "123", "X-Nav-Map-Valid-For-Ms": "1500", ...headers };
  }
  return {
    ok: status >= 200 && status < 300, status,
    json: async () => value, blob: async () => blob,
    headers: { get: (name) => headers[name] || null },
  };
}

class Element {
  constructor() {
    this.textContent = ""; this.value = ""; this.hidden = false; this.disabled = false;
    this.children = []; this.events = new Map(); this.attributes = {};
  }
  set innerHTML(_) { throw new Error("Unsafe HTML rendering"); }
  addEventListener(name, callback) { this.events.set(name, callback); }
  fire(name, event = {}) { return this.events.get(name)?.({ preventDefault() {}, ...event }); }
  append(...children) { this.children.push(...children); }
  replaceChildren(...children) { this.children = children; }
  setAttribute(name, value) { this.attributes[name] = value; }
  removeAttribute(name) { delete this[name]; delete this.attributes[name]; }
  scrollIntoView() { this.scrolled = true; }
}

async function page({ state = {}, fetcher, language = "en", storageThrows = false } = {}) {
  let saved = { ...initial, ...state };
  const elements = new Map([...html.matchAll(/id="([^"]+)"/g)].map((match) => [match[1], new Element()]));
  const window = new Element(), document = new Element(), requests = [], revoked = [], timers = new Map();
  document.hidden = false;
  document.documentElement = {};
  document.getElementById = (id) => elements.get(id);
  document.createElement = () => new Element();
  let blobId = 0, timerId = 0;
  const context = {
    document, window, navigator: { language }, AbortController, URLSearchParams, performance,
    localStorage: { getItem() { if (storageThrows) throw new Error("blocked storage"); return language; } },
    URL: { createObjectURL: () => `blob:${++blobId}`, revokeObjectURL: (url) => revoked.push(url) },
    setTimeout: (fn, delay) => { timers.set(++timerId, { fn, delay }); return timerId; },
    clearTimeout: (id) => timers.delete(id),
    fetch: async (url, options = {}) => {
      requests.push({ url, options });
      const override = fetcher?.(url, options, requests.length);
      if (override !== undefined) return await override;
      if (url === "/api/navigation/destination") {
        if (options.method === "POST") saved = { ...saved, destination: JSON.parse(options.body) };
        if (options.method === "DELETE") saved = { ...saved, destination: null };
        return response(options.method ? { destination: saved.destination } : saved);
      }
      if (url.startsWith("/api/navigation/search?")) return response({ results: [destination] });
      if (url === "/api/navigation/map.png") return response(null, 204, { "X-Nav-Map-Status": "waiting" });
      throw new Error(`Unexpected request ${url}`);
    },
  };
  vm.runInNewContext(source, context);
  await settle();
  const element = (id) => elements.get(id);
  return {
    element, document, window, requests, revoked, timers,
    saved: () => saved, setSaved: (value) => { saved = { ...saved, ...value }; },
    buttons: () => element("results").children.map((row) => row.children[0]),
    search: async (query = "Seoul") => { element("query").value = query; await element("form").fire("submit"); await settle(); },
    input: (query) => { element("query").value = query; element("query").fire("input"); },
    refresh: async () => { await element("refresh").fire("click"); await settle(); },
  };
}
const writes = (p) => p.requests.filter(({ options }) => ["POST", "DELETE"].includes(options.method));
const polls = (p) => [...p.timers.values()].filter(({ delay }) => delay === 1000);
const ready = {
  pipeline_status: "ready", model_input_ready: true, model_status_fresh: true, model_active: true,
  nav_model: true, nav_input_supported: true, nav_route_available: true, nav_tbt_available: true, nav_map_available: true,
  model_provenance_verified: true, model_navigation_source: "mapbox", model_route_id: "123",
  model_input_valid_for_ms: 1500,
};

test("Navdy configured, missing key and missing optional navigation input remain distinct", async () => {
  const p = await page({ state: { navigation_source: "navdy", key_configured: false, destination_editable: false,
    pipeline_status: "route_unavailable" }, fetcher: (url) => url.endsWith("map.png")
    ? response(null, 204, { "X-Nav-Map-Status": "route" }) : undefined });
  assert.match(p.element("sourceLabel").textContent, /Configured/);
  assert.match(p.element("sourceStatus").textContent, /Navdy.*runtime unverified/);
  assert.equal(p.element("keyStatus").textContent, "Missing");
  assert.equal(p.element("pipelineStatus").textContent, "Complete route unavailable");
  assert.match(p.element("pipelineReason").textContent, /Camera inference is independent/);
  assert.match(p.element("mapStatus").textContent, /Complete route unavailable/);
  assert.equal(p.element("map").hidden, true);
  assert.equal(p.element("clear").disabled, true);
  assert.equal(p.requests.length, 2, "read-only map demand does not require destination ownership");
  assert.equal(p.timers.size, 1, "only one visible-page refresh, no abandoned deadlines");
  assert.equal(polls(p).length, 1);
});

test("search is allowed for Navdy, but selection and clearing cannot send writes", async () => {
  const p = await page({ state: { navigation_source: "navdy", destination_editable: false, destination } });
  await p.search();
  assert.equal(p.buttons().length, 1);
  assert.equal(p.buttons()[0].disabled, true);
  assert.equal(p.element("search").disabled, false);
  await p.buttons()[0].fire("click");
  await p.element("clear").fire("click");
  assert.equal(writes(p).length, 0);
});

test("unknown source, absent permission and reserved external destination fail closed", async () => {
  for (const state of [
    { navigation_source: "unknown", destination_editable: true },
    { destination_editable: undefined },
    { destination: { ...destination, place_name: "External Navi" }, destination_editable: true },
  ]) {
    const p = await page({ state });
    await p.search();
    await p.buttons()[0].fire("click");
    await p.element("clear").fire("click");
    assert.equal(writes(p).length, 0);
    assert.equal(p.buttons()[0].disabled, true);
  }
});

test("select saves only coordinates/name, then authoritative GET; never selects a source", async () => {
  const p = await page({ fetcher: (url) => url.includes("search?")
    ? response({ results: [{ ...destination, access_token: "not-a-real-token", source: "navdy" }] }) : undefined });
  await p.search("서울 역");
  assert.ok(p.requests.some(({ url }) => url.endsWith(`q=${encodeURIComponent("서울 역")}`)));
  await p.buttons()[0].fire("click");
  assert.deepEqual(JSON.parse(writes(p)[0].options.body), destination);
  assert.equal(p.element("current").textContent, destination.place_name);
  assert.equal(p.element("status").textContent, "Destination saved.");
  assert.match(p.element("pipelineStatus").textContent, /Model status unavailable/);
  assert.equal(p.element("clear").disabled, false);
  assert.ok(p.requests.every(({ url }) => url.startsWith("/api/navigation/")));
  assert.ok(p.requests.every(({ options }) => options.cache === "no-store" && options.redirect === "error"));
});

test("clear removes the saved destination without modifying selected source", async () => {
  const p = await page({ state: { destination } });
  await p.element("clear").fire("click");
  assert.equal(writes(p).length, 1);
  assert.equal(writes(p)[0].options.method, "DELETE");
  assert.equal(p.saved().destination, null);
  assert.equal(p.saved().navigation_source, "network");
  assert.equal(p.element("clear").disabled, true);
  assert.match(p.element("current").textContent, /No saved destination/);
});

test("late search responses and failures cannot replace the latest results", async () => {
  for (const fail of [false, true]) {
    const older = deferred();
    const p = await page({ fetcher: (url) => url.includes("q=older") ? older.promise : undefined });
    const oldSearch = p.search("older");
    await p.search("newer");
    if (fail) older.reject(new Error("stale failure"));
    else older.resolve(response({ results: [{ ...destination, place_name: "Old result" }] }));
    await oldSearch;
    assert.equal(p.buttons()[0].textContent, destination.place_name);
    assert.equal(p.element("status").textContent, "Search results");
    assert.equal(p.element("results").attributes["aria-busy"], "false");
  }
});

test("typing invalidates outstanding searches before a new submit", async () => {
  const older = deferred();
  const p = await page({ fetcher: (url) => url.includes("search?") ? older.promise : undefined });
  const pending = p.search();
  p.input("different destination");
  older.resolve(response({ results: [destination] }));
  await pending;
  assert.equal(p.buttons().length, 0);
  assert.equal(p.element("status").textContent, "");
});

test("a detached old result cannot commit while a newer search is outstanding", async () => {
  const newer = deferred();
  const p = await page({ fetcher: (url) => url.includes("q=newer") ? newer.promise : undefined });
  await p.search();
  const stale = p.buttons()[0];
  const pending = p.search("newer");
  await stale.fire("click");
  assert.equal(writes(p).length, 0);
  newer.resolve(response({ results: [destination] }));
  await pending;
});

test("writes are serialized, disabling competing selects, clears and searches", async () => {
  const saving = deferred();
  const p = await page({ fetcher: (_, options) => options.method === "POST" ? saving.promise : undefined });
  await p.search();
  const oldButton = p.buttons()[0];
  const pending = oldButton.fire("click");
  assert.equal(p.element("search").disabled, true);
  assert.equal(p.element("query").disabled, true);
  assert.equal(p.element("clear").disabled, true);
  await oldButton.fire("click");
  await p.element("clear").fire("click");
  await p.search("another");
  assert.equal(writes(p).length, 1);
  p.setSaved({ destination });
  saving.resolve(response({ destination }));
  await pending;
  assert.equal(p.element("search").disabled, false);
  assert.equal(p.element("current").textContent, destination.place_name);
});

test("clear invalidates pending search and refuses late result selection", async () => {
  const searching = deferred();
  const p = await page({ state: { destination }, fetcher: (url) => url.includes("search?") ? searching.promise : undefined });
  const pending = p.search();
  await p.element("clear").fire("click");
  searching.resolve(response({ results: [destination] }));
  await pending;
  assert.equal(p.buttons().length, 0);
  assert.equal(p.saved().destination, null);
});

test("out-of-order status reads cannot restore obsolete permission or destination", async () => {
  const first = deferred();
  const p = await page({ fetcher: (url, _, count) => count === 1 ? first.promise : undefined });
  p.setSaved({ navigation_source: "navdy", destination_editable: false });
  await p.refresh();
  first.resolve(response({ ...initial, destination }));
  await settle();
  assert.match(p.element("sourceStatus").textContent, /Navdy/);
  assert.equal(p.element("clear").disabled, true);
  assert.match(p.element("current").textContent, /No saved/);
});

test("write conflict reloads configured source and locks remaining edits", async () => {
  const p = await page({ fetcher: (_, options) => options.method === "POST" ? response({ error: "private diagnostic" }, 409) : undefined });
  await p.search();
  p.setSaved({ navigation_source: "navdy", destination_editable: false });
  await p.buttons()[0].fire("click");
  assert.match(p.element("status").textContent, /changes are blocked/);
  assert.match(p.element("sourceStatus").textContent, /Navdy/);
  assert.equal(p.element("clear").disabled, true);
});

test("failed write with unknown outcome re-reads server instead of optimistic updates", async () => {
  const p = await page({ fetcher: (_, options) => options.method === "POST" ? Promise.reject(new Error("offline")) : undefined });
  await p.search();
  p.setSaved({ destination });
  await p.buttons()[0].fire("click");
  assert.match(p.element("status").textContent, /outcome unverified/);
  assert.equal(p.element("current").textContent, destination.place_name);
});

test("missing key, invalid query, rate and offline errors are safe actionable text", async () => {
  for (const [status, expected] of [[409, /Public key missing/], [400, /Check the query/], [429, /rate limit/], [502, /Service unavailable/], [403, /Request denied/]]) {
    const p = await page({ fetcher: (url) => url.includes("search?")
      ? response({ error: "https://mapbox.invalid/?access_token=sk.secret <script>unsafe</script>" }, status) : undefined });
    await p.search();
    assert.match(p.element("status").textContent, expected);
    assert.doesNotMatch(p.element("status").textContent, /sk.secret|access_token|script/);
    assert.equal(p.element("search").disabled, false);
  }
});

test("network and malformed JSON failures unlock search without leaking exception messages", async () => {
  for (const broken of [Promise.reject(new Error("token=secret")), { ...response({}), json: async () => { throw new Error("secret"); } }]) {
    // Register the rejection handler before page's initial asynchronous read.
    if (broken instanceof Promise) broken.catch(() => {});
    const p = await page({ fetcher: (url) => url.includes("search?") ? broken : undefined });
    await p.search();
    assert.match(p.element("status").textContent, /Service unavailable/);
    assert.doesNotMatch(p.element("status").textContent, /secret/);
    assert.equal(p.timers.size, 1);
    assert.equal(polls(p).length, 1);
  }
});

test("query validation sends no request for short, long or blank searches", async () => {
  const p = await page();
  for (const query of [" ", "a", "x".repeat(201)]) await p.search(query);
  assert.equal(p.requests.length, 2);
  assert.match(p.element("status").textContent, /Check the query/);
});

test("untrusted names use textContent and invalid coordinates/reserved names are excluded", async () => {
  const name = '<img src=x onerror="window.stolen=true">';
  const p = await page({ fetcher: (url) => url.includes("search?") ? response({ results: [
    { ...destination, place_name: name }, { ...destination, latitude: "37" },
    { ...destination, longitude: 181 }, { ...destination, place_name: "External Navi" },
    { ...destination, place_name: "bad\nname" },
  ] }) : undefined });
  await p.search();
  assert.equal(p.buttons().length, 1);
  assert.equal(p.buttons()[0].textContent, name);
  assert.equal(p.buttons()[0].children.length, 0);
  await p.buttons()[0].fire("click");
  assert.equal(p.element("current").textContent, name);
});

test("empty and malformed search result sets are handled", async () => {
  for (const [results, expected] of [[[], /No results/], [null, /Service unavailable/]]) {
    const p = await page({ fetcher: (url) => url.includes("search?") ? response({ results }) : undefined });
    await p.search();
    assert.equal(p.buttons().length, 0);
    assert.match(p.element("status").textContent, expected);
  }
});

test("status failure locks writes, clears stale map and keeps search usable", async () => {
  let offline = false;
  const p = await page({ state: { destination }, fetcher: (url) => offline && url.endsWith("destination") ? Promise.reject(new Error("secret")) : undefined });
  offline = true;
  await p.refresh();
  assert.equal(p.element("clear").disabled, true);
  assert.match(p.element("current").textContent, /unverified/);
  assert.equal(p.element("search").disabled, false);
  assert.equal(p.element("map").hidden, true);
  assert.match(p.element("pipelineStatus").textContent, /unverified/);
});

test("204 waiting is explicit, with no image decoding and one bounded refresh", async () => {
  const p = await page({ state: { pipeline_status: "unknown" } });
  assert.equal(p.requests.length, 2);
  assert.match(p.element("mapStatus").textContent, /Waiting for map rendering/);
  assert.equal(p.element("map").src, undefined);
  assert.equal(p.timers.size, 1);
  assert.equal(polls(p).length, 1);
});

test("a ready PNG is a preview only and loses its URL when unavailable or unloaded", async () => {
  let available = true;
  const p = await page({ state: { pipeline_status: "unknown", map_ready: true, model_input_ready: true },
    fetcher: (url) => url.endsWith("map.png") ? available
      ? response(null, 200, { "X-Nav-Map-Status": "ready", "Content-Type": "image/png" })
      : response(null, 204, { "X-Nav-Map-Status": "stale" }) : undefined });
  assert.equal(p.element("map").src, "blob:1");
  assert.equal(p.element("map").hidden, false);
  assert.match(p.element("previewNote").textContent, /Model input status checked separately/);
  assert.match(p.element("pipelineStatus").textContent, /unverified/);
  available = false;
  p.setSaved({ pipeline_status: "model_status_stale" });
  await p.refresh();
  assert.deepEqual(p.revoked, ["blob:1"]);
  assert.equal(p.element("map").src, undefined);
  assert.equal(p.element("map").hidden, true);
  assert.equal(p.requests.filter(({ url }) => url.endsWith("map.png")).length, 2);
  assert.match(p.element("mapStatus").textContent, /expired/);
});

test("late preview cannot replace a newer unavailable state", async () => {
  const map = deferred();
  let first = true;
  const p = await page({ fetcher: (url) => url.endsWith("map.png") && first ? map.promise : undefined });
  first = false;
  p.setSaved({ pipeline_status: "model_status_stale" });
  await p.refresh();
  map.resolve(response(null, 200, { "X-Nav-Map-Status": "ready", "Content-Type": "image/png" }));
  await settle();
  assert.equal(p.element("map").hidden, true);
  assert.equal(p.element("map").src, undefined);
});

test("non-PNG previews, empty images and decode errors do not masquerade as a map", async () => {
  for (const [type, size] of [["text/html", 12], ["image/png", 0]]) {
    const p = await page({ state: { pipeline_status: "unknown" }, fetcher: (url) => url.endsWith("map.png")
      ? response(null, 200, { "X-Nav-Map-Status": "ready", "Content-Type": type }, { size }) : undefined });
    assert.equal(p.element("map").hidden, true);
  }
  const p = await page({ state: { pipeline_status: "unknown" }, fetcher: (url) => url.endsWith("map.png")
    ? response(null, 200, { "X-Nav-Map-Status": "ready", "Content-Type": "image/png" }) : undefined });
  p.element("map").fire("error");
  assert.equal(p.element("map").hidden, true);
  assert.match(p.element("mapStatus").textContent, /could not be decoded/);
  assert.deepEqual(p.revoked, ["blob:1"]);
});

test("hidden tabs do not poll; pagehide cancels requests and pageshow refreshes", async () => {
  const p = await page();
  p.document.hidden = true;
  p.document.fire("visibilitychange");
  assert.equal(p.requests.length, 2);
  assert.equal(p.timers.size, 0);
  p.window.fire("pagehide");
  await p.search();
  assert.equal(p.requests.length, 2);
  p.document.hidden = false;
  p.window.fire("pageshow", { persisted: true });
  await settle();
  assert.equal(p.requests.length, 4);
  assert.equal(polls(p).length, 1);
});

test("only a fresh active supported N16 with every input can report ready", async () => {
  const p = await page({ state: ready });
  assert.equal(p.element("pipelineStatus").textContent, "N16 navigation input ready");
  assert.equal(p.element("pipelineStatus").attributes["data-ready"], "true");
  assert.equal(p.element("map").hidden, true, "model input does not require a web PNG");
  for (const key of ["model_input_ready", "model_status_fresh", "model_active", "nav_model", "nav_input_supported",
    "nav_route_available", "nav_tbt_available", "nav_map_available", "model_provenance_verified"]) {
    for (const value of [false, undefined, "true", 1]) {
      p.setSaved({ ...ready, [key]: value });
      await p.refresh();
      assert.match(p.element("pipelineStatus").textContent, /unverified/);
      assert.equal(p.element("pipelineStatus").attributes["data-ready"], "false");
    }
  }
  p.setSaved({ ...ready, navigation_source: "navdy" });
  await p.refresh();
  assert.equal(p.element("pipelineStatus").attributes["data-ready"], "false");
});

test("fresh matched Navdy provenance enables only read-only map and actual model status", async () => {
  const state = { ...ready, navigation_source: "navdy", destination_editable: true, destination,
    map_provenance_verified: true, map_navigation_source: "navdy", map_route_id: "18446744073709551615",
    model_navigation_source: "navdy", model_route_id: "18446744073709551615" };
  const p = await page({ state, fetcher: (url) => url.endsWith("map.png") ? response(null, 200,
    { "X-Nav-Map-Status": "ready", "Content-Type": "image/png", "X-Nav-Map-Source": "navdy",
      "X-Nav-Map-Route": state.map_route_id }) : undefined });
  assert.equal(p.element("pipelineStatus").attributes["data-ready"], "true");
  assert.match(p.element("sourceStatus").textContent, /Navdy.*current route verified/);
  assert.equal(p.element("map").hidden, false);
  await p.search();
  assert.equal(p.buttons()[0].disabled, true);
  await p.buttons()[0].fire("click");
  await p.element("clear").fire("click");
  assert.equal(writes(p).length, 0);
  p.setSaved({ pipeline_status: "model_inactive", model_active: false, model_input_ready: false });
  await p.refresh();
  assert.equal(p.element("pipelineStatus").attributes["data-ready"], "false");
  assert.match(p.element("pipelineStatus").textContent, /model inactive/);
  assert.equal(p.element("map").hidden, false);
});

test("preview source and exact route must match the status snapshot including source changes in flight", async () => {
  for (const headers of [
    { "X-Nav-Map-Source": "mapbox", "X-Nav-Map-Route": "123" },
    { "X-Nav-Map-Source": "navdy", "X-Nav-Map-Route": "124" },
    { "X-Nav-Map-Source": "navdy", "X-Nav-Map-Route": null },
    { "X-Nav-Map-Source": null, "X-Nav-Map-Route": "123" },
  ]) {
    const p = await page({ state: { navigation_source: "navdy", map_navigation_source: "navdy" },
      fetcher: (url) => url.endsWith("map.png") ? response(null, 200,
        { "X-Nav-Map-Status": "ready", "Content-Type": "image/png", ...headers }) : undefined });
    assert.equal(p.element("map").hidden, true);
    assert.match(p.element("mapStatus").textContent, /Route or navigation source changed/);
  }
});

test("model route identity must match current renderer metadata, not merely a ready flag", async () => {
  for (const changes of [{ model_route_id: "124" }, { model_route_id: 123 }, { model_route_id: undefined },
    { model_navigation_source: "network-v2" }, { map_navigation_source: undefined }, { map_route_id: undefined }]) {
    const p = await page({ state: { ...ready, ...changes } });
    assert.equal(p.element("pipelineStatus").attributes["data-ready"], "false");
  }
});

test("displayed model readiness and PNG expire even when further requests stop", async () => {
  const p = await page({ state: ready, fetcher: (url) => url.endsWith("map.png") ? response(null, 200,
    { "X-Nav-Map-Status": "ready", "Content-Type": "image/png" }) : undefined });
  assert.equal(p.element("pipelineStatus").attributes["data-ready"], "true");
  assert.equal(p.element("map").hidden, false);
  const expiries = [...p.timers.values()].filter(({ delay }) => delay > 1000 && delay <= 1500);
  assert.equal(expiries.length, 2);
  expiries.forEach(({ fn }) => fn());
  assert.equal(p.element("pipelineStatus").attributes["data-ready"], "false");
  assert.match(p.element("pipelineStatus").textContent, /expired/);
  assert.equal(p.element("map").hidden, true);
  assert.match(p.element("mapStatus").textContent, /expired/);
});

test("missing or invalid display deadlines cannot keep stale readiness or images visible", async () => {
  for (const lifetime of [undefined, null, 0, -1, "1500", Infinity, NaN, 2501]) {
    const p = await page({ state: { ...ready, model_input_valid_for_ms: lifetime } });
    assert.equal(p.element("pipelineStatus").attributes["data-ready"], "false");
  }
  for (const lifetime of [null, "0", "-1", "2001", "Infinity", "secret"]) {
    const p = await page({ fetcher: (url) => url.endsWith("map.png") ? response(null, 200,
      { "X-Nav-Map-Status": "ready", "Content-Type": "image/png", "X-Nav-Map-Valid-For-Ms": lifetime }) : undefined });
    assert.equal(p.element("map").hidden, true);
  }
});

test("missing, stale, inactive, non-N16 and missing navigation inputs remain distinct", async () => {
  for (const [pipeline_status, expected] of [
    ["model_status_missing", /Model status unavailable/], ["model_status_stale", /Model status expired/],
    ["model_inactive", /model inactive/], ["unsupported_model", /does not support N16/],
    ["route_unavailable", /Complete route unavailable/], ["tbt_unavailable", /turn guidance unavailable/],
    ["map_not_yet_available", /Waiting for model map input/], ["context_expired", /guidance expired/],
    ["provenance_unverified", /Current source and route unverified/],
  ]) {
    const p = await page({ state: { pipeline_status } });
    assert.match(p.element("pipelineStatus").textContent, expected);
    assert.equal(p.element("pipelineStatus").attributes["data-ready"], "false");
  }
});

test("untrusted status and header reasons cannot expose tokens or prototype properties", async () => {
  for (const reason of ["sk.secret", "__proto__", "toString", "constructor"]) {
    const p = await page({ state: { pipeline_status: reason, reason: "access_token=sk.secret" },
      fetcher: (url) => url.endsWith("map.png") ? response(null, 204, { "X-Nav-Map-Status": reason }) : undefined });
    assert.equal(p.element("pipelineStatus").textContent, "Navigation model input unverified");
    assert.equal(p.element("mapStatus").textContent, "Map preview unavailable");
  }
});

test("visible polling refreshes status and demand without overlap, hides stale readiness on suspension", async () => {
  let pending;
  const p = await page({ state: ready, fetcher: (url) => url.endsWith("destination") ? pending?.promise : undefined });
  pending = deferred();
  const poll = [...p.timers].find(([, { delay }]) => delay === 1000);
  p.timers.delete(poll[0]);
  const refresh = poll[1].fn();
  assert.match(p.element("pipelineStatus").textContent, /Checking/);
  assert.equal(p.element("pipelineStatus").attributes["data-ready"], "false");
  assert.equal(polls(p).length, 0, "do not schedule while a request is pending");
  p.document.hidden = true;
  p.document.fire("visibilitychange");
  pending.resolve(response({ ...initial, ...ready }));
  await refresh;
  assert.equal(p.timers.size, 0);
  assert.equal(p.element("pipelineStatus").attributes["data-ready"], "false");
  pending = undefined;
  p.document.hidden = false;
  p.document.fire("visibilitychange");
  await settle();
  assert.equal(polls(p).length, 1);
  assert.equal(p.element("pipelineStatus").attributes["data-ready"], "true");
  p.window.fire("pagehide");
  assert.equal(p.timers.size, 0);
  assert.equal(p.element("pipelineStatus").attributes["data-ready"], "false");
});

test("pagehide invalidates an in-flight write even after a back-forward restoration", async () => {
  const saving = deferred();
  const p = await page({ fetcher: (_, options) => options.method === "POST" ? saving.promise : undefined });
  await p.search();
  const pending = p.buttons()[0].fire("click");
  p.window.fire("pagehide");
  assert.equal(writes(p)[0].options.signal.aborted, true);
  p.window.fire("pageshow", { persisted: true });
  await settle();
  await p.search("new query");
  const count = p.requests.length;
  saving.resolve(response({ destination }));
  await pending;
  assert.equal(p.requests.length, count, "the obsolete write cannot schedule another read");
  assert.equal(p.element("status").textContent, "Search results");
});

test("request deadline aborts a hanging search and keeps the page retryable", async () => {
  const p = await page({ fetcher: (url, options) => url.includes("search?") ? new Promise((_, reject) => {
    options.signal.addEventListener("abort", () => reject(new Error("timeout with private URL")));
  }) : undefined });
  const pending = p.search();
  assert.equal(p.timers.size, 2);
  const timer = [...p.timers.values()].find(({ delay }) => delay === 12000);
  assert.equal(timer.delay, 12000);
  timer.fn();
  await pending;
  assert.match(p.element("status").textContent, /Service unavailable/);
  assert.equal(p.element("search").disabled, false);
  assert.equal(p.element("results").attributes["aria-busy"], "false");
  assert.equal(p.timers.size, 1);
  assert.equal(polls(p).length, 1);
});

test("unavailable storage does not prevent localization or loading", async () => {
  const p = await page({ language: "ko", storageThrows: true });
  assert.match(p.element("pipelineStatus").textContent, /모델 상태 없음/);
  assert.match(p.element("sourceLabel").textContent, /설정된/);
  assert.equal(p.document.documentElement.lang, "ko");
});

test("main-page links dispatch through existing settings groups, without any source writes", async () => {
  for (const [target, group] of [["mapbox", "SYS_NET"], ["source", "SPEED"]]) {
    const calls = [], card = new Element(), window = new Element();
    Object.assign(window, {
      location: { search: `?navigation_settings=${target}` }, __CARROT_WEB_HAS_BOOTSTRAPPED_PAGE: true,
      loadSettings: async () => calls.push("load"), showPage: (name) => calls.push(name),
      switchSettingTab: async (name) => calls.push(name),
      activateSettingGroup: async (name) => calls.push(name), focusSettingItem: (name) => calls.push(name),
      showAppToast: () => assert.fail("unexpected settings failure"),
    });
    if (target === "source") window.findSettingItemByName = (name) => {
      assert.equal(name, "CarrotNaviSource");
      assert.ok(calls.includes("load"), "resolve the rendered group only after settings load");
      return { group };
    };
    vm.runInNewContext(source, {
      document: { readyState: "complete", getElementById: (id) => id === "mapboxTokenCard" ? card : null },
      window, URLSearchParams,
      history: { replaceState: (state, _, url) => {
        assert.equal(state.page, "setting");
        assert.equal(state.screen, "items");
        calls.push([state.group, url]);
      } },
      fetch: () => assert.fail("settings navigation must not call destination/source APIs"),
    });
    await settle();
    assert.deepEqual(calls.slice(0, 4), ["setting", "load", "carrot", group]);
    assert.deepEqual(calls.at(-1), [group, "/"]);
    if (target === "source") assert.ok(calls.includes("CarrotNaviSource"));
    else assert.equal(card.scrolled, true);
  }
});

test("source group resolution waits for the loaded catalog and uses its renamed group in history", async () => {
  const loaded = deferred(), activated = deferred(), calls = [], routes = [], window = new Element();
  let entry = null;
  Object.assign(window, {
    location: { search: "?navigation_settings=source" }, __CARROT_WEB_HAS_BOOTSTRAPPED_PAGE: true,
    showPage: () => {}, loadSettings: () => loaded.promise,
    findSettingItemByName: (name) => { calls.push(name); return entry; },
    switchSettingTab: async () => {},
    activateSettingGroup: (group) => { calls.push(group); return activated.promise; },
    focusSettingItem: (name) => calls.push(`focus:${name}`),
    showAppToast: () => assert.fail("unexpected settings failure"),
  });
  vm.runInNewContext(source, {
    document: { readyState: "complete", getElementById: () => null }, window, URLSearchParams,
    history: { replaceState: (state, _, url) => routes.push({ ...state, url }) },
  });
  await settle();
  assert.deepEqual(calls, []);
  entry = { group: "flattened-navigation-group" };
  loaded.resolve();
  await settle();
  assert.deepEqual(calls, ["CarrotNaviSource", entry.group]);
  assert.deepEqual(routes, [], "history must not point to an unrendered group");
  activated.resolve();
  await settle();
  assert.deepEqual(calls, ["CarrotNaviSource", entry.group, "focus:CarrotNaviSource"]);
  assert.deepEqual(routes, [{ page: "setting", screen: "items", group: entry.group, url: "/" }]);
  assert.doesNotMatch(source, /SPEED_CAMERA/);
});

test("missing source setting reports an explicit error without opening an empty group or rewriting history", async () => {
  for (const entry of [null, {}, { group: "" }, { group: " " }]) {
    const errors = [], window = new Element();
    Object.assign(window, {
      location: { search: "?navigation_settings=source" }, __CARROT_WEB_HAS_BOOTSTRAPPED_PAGE: true,
      showPage: () => {}, loadSettings: async () => {}, findSettingItemByName: () => entry,
      switchSettingTab: () => assert.fail("must not navigate to an absent setting"),
      activateSettingGroup: () => assert.fail("must not activate an empty group"),
      focusSettingItem: () => assert.fail("must not focus an absent setting"),
      showAppToast: (message, options) => errors.push({ message, tone: options.tone }),
    });
    vm.runInNewContext(source, {
      document: { readyState: "complete", getElementById: () => null }, window, URLSearchParams,
      history: { replaceState: () => assert.fail("must preserve existing history on failure") },
    });
    await settle();
    assert.equal(errors.length, 1);
    assert.match(errors[0].message, /CarrotNaviSource.*unavailable/);
    assert.equal(errors[0].tone, "error");
    window.fire("carrot:pagechange");
    await settle();
    assert.equal(errors.length, 1);
  }
});

test("settings entry waits for DOM, app bootstrap and callable globals, then runs once", async () => {
  const calls = [], window = new Element();
  const document = { readyState: "loading", getElementById: () => null };
  Object.assign(window, {
    location: { search: "?navigation_settings=source" }, __CARROT_WEB_HAS_BOOTSTRAPPED_PAGE: false,
    showPage: () => calls.push("show"), loadSettings: async () => calls.push("load"),
    switchSettingTab: async () => {}, activateSettingGroup: async () => calls.push("activate"),
    findSettingItemByName: () => ({ group: "SPEED" }),
  });
  vm.runInNewContext(source, { document, window, URLSearchParams, history: { replaceState() {} } });
  await settle();
  assert.deepEqual(calls, []);
  document.readyState = "interactive";
  window.fire("DOMContentLoaded");
  await settle();
  assert.deepEqual(calls, []);
  window.fire("carrot:pagechange");
  window.__CARROT_WEB_HAS_BOOTSTRAPPED_PAGE = true;
  await settle();
  assert.deepEqual(calls, [], "missing focusSettingItem must not invoke partial settings UI");
  window.focusSettingItem = () => calls.push("focus");
  window.fire("carrot:pagechange");
  await settle();
  assert.deepEqual(calls, ["show", "load", "activate", "focus"]);
  window.fire("carrot:pagechange");
  await settle();
  assert.equal(calls.length, 4);
});

test("unrelated or prototype-shaped settings targets do nothing on the main page", async () => {
  for (const target of ["", "toString", "__proto__", "unknown"]) {
    const window = new Element();
    window.location = { search: `?navigation_settings=${target}` };
    vm.runInNewContext(source, { document: { getElementById: () => null }, window, URLSearchParams });
    await settle();
    assert.equal(window.events.size, 0);
  }
});

test("HTML uses accessible responsive controls, existing assets and settings entry points", () => {
  assert.match(index, /id="toolsNavigation"[^>]*href="\/navigation"/);
  assert.match(index, /src="\/js\/pages\/navigation_map.js"/);
  assert.match(html, /href="\/\?navigation_settings=mapbox"/);
  assert.match(html, /href="\/\?navigation_settings=source"/);
  assert.match(html, /aria-live="polite"/);
  assert.match(html, /grid-template-columns: minmax\(0, 1fr\)/);
  assert.match(html, /aspect-ratio: 1/);
  assert.doesNotMatch(source, /innerHTML|setInterval|console\.|api\/mapbox\/tokens|access_token=/);
  assert.doesNotMatch(html + source, /Model input unavailable|모델 입력 없음|Web page|Delivered/);
});
