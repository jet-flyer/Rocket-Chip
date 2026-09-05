/**
 * Rocket Chip hello-world dictionary — CSV columns as telemetry objects.
 * Pattern: nasa/openmct-tutorial DictionaryPlugin (simplified, no HTTP).
 */
(function (global) {
  const NAMESPACE = 'rocket-chip.hello';
  const MEASUREMENTS = [
    { key: 'seq', name: 'Sequence', units: 'count', format: 'integer', hints: { range: 1 } },
    { key: 'rssi', name: 'RSSI', units: 'dBm', format: 'integer', hints: { range: 1 } },
    { key: 'snr', name: 'SNR', units: 'dB', format: 'float', hints: { range: 1 } }
  ];

  function RcCsvDictionaryPlugin() {
    return function install(openmct) {
      const rootId = { namespace: NAMESPACE, key: 'link' };

      openmct.objects.addRoot(rootId);

      openmct.objects.addProvider(NAMESPACE, {
        get: function (identifier) {
          if (identifier.key === 'link') {
            return Promise.resolve({
              identifier: rootId,
              name: 'Rocket Chip Link (CSV)',
              type: 'folder',
              location: 'ROOT',
              composition: MEASUREMENTS.map(function (m) {
                return { namespace: NAMESPACE, key: m.key };
              })
            });
          }
          const m = MEASUREMENTS.find(function (x) { return x.key === identifier.key; });
          if (!m) {
            return Promise.reject(new Error('Unknown object ' + identifier.key));
          }
          return Promise.resolve({
            identifier: { namespace: NAMESPACE, key: m.key },
            name: m.name,
            type: 'rocket-chip.telemetry',
            telemetry: {
              values: [
                { key: 'utc', source: 'timestamp', name: 'Timestamp', format: 'utc', hints: { domain: 1 } },
                { key: 'value', name: m.name, units: m.units, format: m.format, hints: m.hints }
              ]
            },
            location: NAMESPACE + ':link'
          });
        }
      });

      openmct.composition.addProvider({
        appliesTo: function (domainObject) {
          return domainObject.identifier.namespace === NAMESPACE &&
            domainObject.type === 'folder';
        },
        load: function (domainObject) {
          return Promise.resolve(domainObject.composition || []);
        }
      });

      openmct.types.addType('rocket-chip.telemetry', {
        name: 'Rocket Chip CSV Telemetry',
        description: 'Hello-world telemetry from station m CSV / fixture',
        cssClass: 'icon-telemetry'
      });
    };
  }

  global.RcCsvDictionaryPlugin = RcCsvDictionaryPlugin;
})(typeof window !== 'undefined' ? window : globalThis);
