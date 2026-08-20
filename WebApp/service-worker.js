// Bump this on every deploy that changes any precached file - it is the only thing
// that makes the browser fetch a new service-worker.js and start an update cycle.
const CACHE_VERSION = 'v1';

const APP_SHELL_CACHE = `txonemove-shell-${CACHE_VERSION}`;
const TILE_CACHE = `txonemove-tiles-${CACHE_VERSION}`;
const MAX_TILE_CACHE_ENTRIES = 400;

// Everything the app needs to run with zero network access.
const PRECACHE_URLS = [
    './',
    './index.html',
    './manifest.json',
    './icons/icon-192.png',
    './icons/icon-512.png',
    './icons/icon-maskable-192.png',
    './icons/icon-maskable-512.png',
    './icons/apple-touch-icon.png',
    './icons/favicon-32.png',
    'https://unpkg.com/leaflet@1.9.4/dist/leaflet.css',
    'https://unpkg.com/leaflet@1.9.4/dist/leaflet.js',
    'https://unpkg.com/leaflet-control-compass/dist/leaflet-compass.min.css',
    'https://unpkg.com/leaflet-control-compass/dist/leaflet-compass.min.js',
];

self.addEventListener('install', (event) => {
    event.waitUntil(
        caches.open(APP_SHELL_CACHE)
            .then((cache) => cache.addAll(PRECACHE_URLS))
    );
    // Do NOT self.skipWaiting() here - the page decides when to activate an
    // update (see the SKIP_WAITING message handler below), so an update never
    // yanks the app shell out from under an open BLE session.
});

self.addEventListener('activate', (event) => {
    event.waitUntil(
        caches.keys()
            .then((names) => Promise.all(
                names
                    .filter((name) => name !== APP_SHELL_CACHE && name !== TILE_CACHE)
                    .map((name) => caches.delete(name))
            ))
            .then(() => self.clients.claim())
    );
});

self.addEventListener('message', (event) => {
    if (event.data && event.data.type === 'SKIP_WAITING') {
        self.skipWaiting();
    }
});

function isTileRequest(url) {
    return /tile\.openstreetmap\.org/.test(url.hostname);
}

function isAppShellRequest(url) {
    if (url.origin === self.location.origin) return true;
    return url.href.startsWith('https://unpkg.com/leaflet');
}

// Cache-first, but silently refreshes the cache entry in the background so the
// NEXT load is up to date whenever there happens to be a connection.
async function staleWhileRevalidate(request, cacheName) {
    const cache = await caches.open(cacheName);
    const cached = await cache.match(request);

    const networkFetch = fetch(request)
        .then((response) => {
            if (response && response.ok) {
                cache.put(request, response.clone());
            }
            return response;
        })
        .catch(() => undefined);

    return cached || (await networkFetch) || Response.error();
}

async function cacheFirstWithLimit(request, cacheName, maxEntries) {
    const cache = await caches.open(cacheName);
    const cached = await cache.match(request);
    if (cached) return cached;

    try {
        const response = await fetch(request);
        if (response && response.ok) {
            await cache.put(request, response.clone());
            const keys = await cache.keys();
            if (keys.length > maxEntries) {
                await cache.delete(keys[0]);
            }
        }
        return response;
    } catch (err) {
        return Response.error();
    }
}

self.addEventListener('fetch', (event) => {
    const { request } = event;
    if (request.method !== 'GET') return;

    const url = new URL(request.url);

    if (isTileRequest(url)) {
        event.respondWith(cacheFirstWithLimit(request, TILE_CACHE, MAX_TILE_CACHE_ENTRIES));
        return;
    }

    if (isAppShellRequest(url)) {
        event.respondWith(staleWhileRevalidate(request, APP_SHELL_CACHE));
    }
    // Everything else (e.g. BLE, other origins) is left to the network as-is.
});
