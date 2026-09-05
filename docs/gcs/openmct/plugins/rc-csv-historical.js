/**
 * Historical provider — loads one CSV (fixture or live) once.
 * Columns: timestamp,seq,rssi,snr (ISO-8601; prefer explicit Z)
 * After openmct.start(), sets Fixed conductor to CSV min/max (+ small pad).
 */
(function (global) {
  const NAMESPACE = 'rocket-chip.hello';

  function parseCsv(text) {
    const lines = text.trim().split(/\r?\n/);
    if (lines.length < 2) return [];
    const header = lines[0].split(',').map(function (h) { return h.trim(); });
    const rows = [];
    for (let i = 1; i < lines.length; i++) {
      if (!lines[i].trim()) continue;
      const cols = lines[i].split(',');
      const row = {};
      header.forEach(function (h, idx) { row[h] = (cols[idx] || '').trim(); });
      const ts = Date.parse(row.timestamp);
      if (Number.isNaN(ts)) continue;
      rows.push({
        timestamp: ts,
        seq: Number(row.seq),
        rssi: Number(row.rssi),
        snr: Number(row.snr)
      });
    }
    return rows;
  }

  function applyFixedBounds(openmct, rows) {
    if (!rows || !rows.length) return;
    var start = rows[0].timestamp;
    var end = rows[rows.length - 1].timestamp;
    if (!(end >= start)) return;
    var pad = Math.max(250, Math.floor((end - start) * 0.02) || 250);
    start -= pad;
    end += pad;
    try {
      if (typeof openmct.time.stopClock === 'function') {
        openmct.time.stopClock();
      }
      openmct.time.setTimeSystem('utc');
      openmct.time.bounds({ start: start, end: end });
      console.info('[rc-csv] Fixed bounds', new Date(start).toISOString(), '->', new Date(end).toISOString());
    } catch (e) {
      console.warn('[rc-csv] could not set Fixed bounds', e);
    }
  }

  function RcCsvHistoricalPlugin(options) {
    options = options || {};
    const csvUrl = options.csvUrl || '../fixtures/live.csv';

    return function install(openmct) {
      let cache = null;
      let loadPromise = null;
      let boundsApplied = false;

      function loadRows() {
        if (cache) return Promise.resolve(cache);
        if (loadPromise) return loadPromise;
        loadPromise = fetch(csvUrl, { cache: 'no-store' })
          .then(function (r) {
            if (!r.ok) throw new Error('CSV fetch failed ' + r.status + ' ' + csvUrl);
            return r.text();
          })
          .then(function (text) {
            cache = parseCsv(text);
            console.info('[rc-csv] loaded', cache.length, 'rows from', csvUrl);
            return cache;
          });
        return loadPromise;
      }

      function tryApplyBounds() {
        if (boundsApplied) return;
        loadRows().then(function (rows) {
          if (boundsApplied) return;
          boundsApplied = true;
          applyFixedBounds(openmct, rows);
        }).catch(function (e) { console.error(e); });
      }

      // Prefer after start (avoids boot race). Fallback timeout if no event.
      if (typeof openmct.on === 'function') {
        openmct.on('start', tryApplyBounds);
      }
      setTimeout(tryApplyBounds, 800);

      openmct.telemetry.addProvider({
        supportsRequest: function (domainObject) {
          return domainObject.type === 'rocket-chip.telemetry' &&
            domainObject.identifier.namespace === NAMESPACE;
        },
        request: function (domainObject, requestOptions) {
          const key = domainObject.identifier.key;
          return loadRows().then(function (rows) {
            const start = requestOptions.start;
            const end = requestOptions.end;
            return rows
              .filter(function (row) {
                return row.timestamp >= start && row.timestamp <= end;
              })
              .map(function (row) {
                return { timestamp: row.timestamp, value: row[key] };
              });
          });
        }
      });
    };
  }

  global.RcCsvHistoricalPlugin = RcCsvHistoricalPlugin;
})(typeof window !== 'undefined' ? window : globalThis);