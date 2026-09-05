/**
 * Rocket Chip hello-world dictionary — telemetry objects + default layouts.
 * Supports live scrape columns (rssi/snr/baro) and Big Daddy glance columns.
 */
(function (global) {
  const NAMESPACE = 'rocket-chip.hello';

  // Tree measurements (keys must match parsed row fields after normalize)
  const MEASUREMENTS = [
    { key: 'seq', name: 'Sequence', units: 'count', format: 'integer', hints: { range: 1 } },
    { key: 'flight_state', name: 'Flight State', units: '', format: 'integer', hints: { range: 1 } },
    { key: 'chute_detected', name: 'Chute Detected', units: '', format: 'integer', hints: { range: 1 } },
    { key: 'alt_m', name: 'Altitude MSL', units: 'm', format: 'float', hints: { range: 1 } },
    { key: 'max_alt_m', name: 'Max Altitude', units: 'm', format: 'float', hints: { range: 1 } },
    { key: 'baro_alt_m', name: 'Baro AGL', units: 'm', format: 'float', hints: { range: 1 } },
    { key: 'vvel_mps', name: 'Vertical Vel', units: 'm/s', format: 'float', hints: { range: 1 } },
    { key: 'speed_mps', name: 'Speed', units: 'm/s', format: 'float', hints: { range: 1 } },
    { key: 'gps_sats', name: 'GPS Sats', units: 'count', format: 'integer', hints: { range: 1 } },
    { key: 'lat', name: 'Latitude', units: 'deg', format: 'float', hints: { range: 1 } },
    { key: 'lon', name: 'Longitude', units: 'deg', format: 'float', hints: { range: 1 } },
    { key: 'batt_v', name: 'Battery', units: 'V', format: 'float', hints: { range: 1 } },
    { key: 'temp_c', name: 'Temp', units: 'C', format: 'float', hints: { range: 1 } },
    { key: 'health', name: 'Health', units: '', format: 'integer', hints: { range: 1 } },
    { key: 'rssi', name: 'RSSI', units: 'dBm', format: 'float', hints: { range: 1 } },
    { key: 'snr', name: 'SNR', units: 'dB', format: 'float', hints: { range: 1 } },
    { key: 'lq_pct', name: 'Link Quality', units: '%', format: 'float', hints: { range: 1 } },
    { key: 'rx_hz', name: 'RX Rate', units: 'Hz', format: 'float', hints: { range: 1 } }
  ];

  function idFor(key) {
    return { namespace: NAMESPACE, key: key };
  }

  const STACKED = {
    identifier: idFor('link-stacked'),
    name: 'Link Live (stacked)',
    type: 'telemetry.plot.stacked',
    location: NAMESPACE + ':link',
    composition: [idFor('alt_m'), idFor('vvel_mps'), idFor('rssi'), idFor('batt_v')],
    configuration: { series: [], yAxis: {}, xAxis: {} }
  };

  const RADIO_OVERLAY = {
    identifier: idFor('link-overlay-radio'),
    name: 'Radio (RSSI+SNR overlay)',
    type: 'telemetry.plot.overlay',
    location: NAMESPACE + ':link',
    composition: [idFor('rssi'), idFor('snr')],
    configuration: {
      series: [{ identifier: idFor('rssi') }, { identifier: idFor('snr') }]
    }
  };

  const TRAJ_OVERLAY = {
    identifier: idFor('link-overlay-traj'),
    name: 'Trajectory (alt + baro AGL)',
    type: 'telemetry.plot.overlay',
    location: NAMESPACE + ':link',
    composition: [idFor('alt_m'), idFor('baro_alt_m'), idFor('max_alt_m')],
    configuration: {
      series: [
        { identifier: idFor('alt_m') },
        { identifier: idFor('baro_alt_m') },
        { identifier: idFor('max_alt_m') }
      ]
    }
  };

  const LAYOUTS = {
    'link-stacked': STACKED,
    'link-overlay-radio': RADIO_OVERLAY,
    'link-overlay-traj': TRAJ_OVERLAY
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
            }).concat([
              idFor('link-stacked'),
              idFor('link-overlay-traj'),
              idFor('link-overlay-radio')
            ]);
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
        description: 'Station live CSV / Big Daddy glance replay',
        cssClass: 'icon-telemetry'
      });

      if (navigateOnStart) {
        openmct.on('start', function () {
          try {
            var hash = window.location.hash || '';
            if (hash.indexOf('/browse/') >= 0 &&
                hash.indexOf('link-stacked') < 0 &&
                hash.indexOf('link-overlay') < 0) {
              return;
            }
            var path = '/browse/' + NAMESPACE + ':link-stacked';
            if (openmct.router && typeof openmct.router.setPath === 'function') {
              openmct.router.setPath(path);
            } else {
              window.location.hash = '#' + path +
                '?tc.mode=fixed&tc.timeSystem=utc';
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
