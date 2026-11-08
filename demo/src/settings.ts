// Per-page settings kept in sessionStorage, so a reload or a trip to another page and back keeps
// each page's controls where they were. Every read and write tolerates storage being unavailable.

const PREFIX = 'solidmesh-demo-';

export function saveSetting(demo: string, key: string, value: string | number | boolean): void {
  try {
    const storeKey = `${PREFIX}${demo}`;
    const existing = JSON.parse(sessionStorage.getItem(storeKey) || '{}');
    existing[key] = value;
    sessionStorage.setItem(storeKey, JSON.stringify(existing));
  } catch { /* ignore storage errors */ }
}

export function loadSettings(demo: string): Record<string, any> {
  try {
    return JSON.parse(sessionStorage.getItem(`${PREFIX}${demo}`) || '{}');
  } catch { return {}; }
}

export function loadSetting<T>(demo: string, key: string, fallback: T): T {
  const settings = loadSettings(demo);
  return key in settings ? settings[key] : fallback;
}
