/**
 * Historical provider — loads one CSV (fixture or live) once.
 * Columns: timestamp,seq,rssi,snr (ISO-8601 timestamp)
 * Pattern: nasa/openmct-tutorial HistoricalTelemetryPlugin, file-backed.
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

  function RcCsvHistoricalPlugin(options) {
    options = options || {};
    const csvUrl = options.csvUrl || '../fixtures/omct_hello_fixture.csv';

    return function install(openmct) {
      let cache = null;
      let loadPromise = null;

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

      loadRows().then(function (rows) {
        if (!rows.length) return;
        const start = rows[0].timestamp;
        const end = rows[rows.length - 1].timestamp;
        const pad = Math.max(1000, Math.floor((end - start) * 0.1) || 1000);
        try {
          openmct.time.setClock('local');
          openmct.time.setClockOffsets({ start: -(end - start + pad), end: pad });
          openmct.time.bounds({ start: start - pad, end: end + pad });
        } catch (e) {
          console.warn('[rc-csv] could not set bounds', e);
        }
      }).catch(function (e) { console.error(e); });

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
