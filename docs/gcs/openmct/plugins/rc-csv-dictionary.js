/**
 * Rocket Chip hello-world dictionary — CSV columns as telemetry objects,
 * plus default stacked / overlay plot layouts.
 */
(function (global) {
  const NAMESPACE = 'rocket-chip.hello';
  const MEASUREMENTS = [
    { key: 'seq', name: 'Sequence', units: 'count', format: 'integer', hints: { range: 1 } },
    { key: 'rssi', name: 'RSSI', units: 'dBm', format: 'integer', hints: { range: 1 } },
    { key: 'snr', name: 'SNR', units: 'dB', format: 'float', hints: { range: 1 } },
    { key: 'baro', name: 'Baro AGL', units: 'm', format: 'float', hints: { range: 1 } }
  ];

  function idFor(key) {
    return { namespace: NAMESPACE, key: key };
  }

  const STACKED = {
    identifier: idFor('link-stacked'),
    name: 'Link Live (stacked)',
    type: 'telemetry.plot.stacked',
    location: NAMESPACE + ':link',
    composition: [idFor('rssi'), idFor('snr'), idFor('baro')],
    configuration: { series: [], yAxis: {}, xAxis: {} }
  };

  // RSSI + SNR only — shared radio domain; baro stays on stacked (different units)
  const RADIO_OVERLAY = {
    identifier: idFor('link-overlay-radio'),
    name: 'Radio (RSSI+SNR overlay)',
    type: 'telemetry.plot.overlay',
    location: NAMESPACE + ':link',
    composition: [idFor('rssi'), idFor('snr')],
    configuration: {
      series: [
        { identifier: idFor('rssi') },
        { identifier: idFor('snr') }
      ]
    }
  };

  const LAYOUTS = {
    'link-stacked': STACKED,
    'link-overlay-radio': RADIO_OVERLAY
  };

  function RcCsvDictionaryPlugin(options) {
    options = options || {};
    const navigateOnStart = options.navigateOnStart !== false;

    return function install(openmct) {
      const rootId = { namespace: NAMESPACE, key: 'link' };

      openmct.objects.addRoot(rootId);

      openmct.objects.addProvider(NAMESPACE, {
        get: function (identifier) {
          if (identifier.key === 'link') {
            var composition = MEASUREMENTS.map(function (m) {
              return idFor(m.key);
            }).concat([idFor('link-stacked'), idFor('link-overlay-radio')]);
            return Promise.resolve({
              identifier: rootId,
              name: 'Rocket Chip Link (CSV)',
              type: 'folder',
              location: 'ROOT',
              composition: composition
            });
          }
          if (LAYOUTS[identifier.key]) {
            return Promise.resolve(LAYOUTS[identifier.key]);
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
            (domainObject.type === 'folder' ||
             domainObject.type === 'telemetry.plot.stacked' ||
             domainObject.type === 'telemetry.plot.overlay');
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

      if (navigateOnStart) {
        openmct.on('start', function () {
          try {
            var hash = window.location.hash || '';
            // Respect an explicit browse target (single point or other layout)
            if (hash.indexOf('/browse/') >= 0 &&
                hash.indexOf('link-stacked') < 0 &&
                hash.indexOf('link-overlay-radio') < 0) {
              return;
            }
            var path = '/browse/' + NAMESPACE + ':link-stacked';
            if (openmct.router && typeof openmct.router.setPath === 'function') {
              openmct.router.setPath(path);
            } else {
              window.location.hash = '#' + path +
                '?tc.mode=local&tc.startDelta=30000&tc.endDelta=5000&tc.timeSystem=utc';
            }
          } catch (e) {
            console.warn('[rc-dict] default navigate failed', e);
          }
        });
      }
    };
  }

  global.RcCsvDictionaryPlugin = RcCsvDictionaryPlugin;
})(typeof window !== 'undefined' ? window : globalThis);
