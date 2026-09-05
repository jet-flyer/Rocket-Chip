/**
 * Rocket Chip CSV / facsimile dictionary + Master Dashboard v1 layouts.
 */
(function (global) {
  const NAMESPACE = 'rocket-chip.hello';

  const MEASUREMENTS = [
    { key: 'seq', name: 'Sequence', units: 'count', format: 'integer', hints: { range: 1 } },
    { key: 'met_ms', name: 'MET', units: 'ms', format: 'integer', hints: { range: 1 } },
    { key: 'flight_state', name: 'Flight State', units: '', format: 'integer', hints: { range: 1 } },
    { key: 'phase_event', name: 'Phase Event', units: '', format: 'integer', hints: { range: 1 } },
    { key: 'chute_detected', name: 'Chute Detected', units: '', format: 'integer', hints: { range: 1 } },
    { key: 'alt_m', name: 'Altitude MSL', units: 'm', format: 'float', hints: { range: 1 } },
    { key: 'max_alt_m', name: 'Max Altitude', units: 'm', format: 'float', hints: { range: 1 } },
    { key: 'baro_alt_m', name: 'Baro AGL', units: 'm', format: 'float', hints: { range: 1 } },
    { key: 'vvel_mps', name: 'Vertical Vel', units: 'm/s', format: 'float', hints: { range: 1 } },
    { key: 'speed_mps', name: 'Speed', units: 'm/s', format: 'float', hints: { range: 1 } },
    { key: 'accel_g', name: 'Accel', units: 'g', format: 'float', hints: { range: 1 } },
    { key: 'q_w', name: 'Quat W', units: '', format: 'float', hints: { range: 1 } },
    { key: 'q_x', name: 'Quat X', units: '', format: 'float', hints: { range: 1 } },
    { key: 'q_y', name: 'Quat Y', units: '', format: 'float', hints: { range: 1 } },
    { key: 'q_z', name: 'Quat Z', units: '', format: 'float', hints: { range: 1 } },
    { key: 'gps_sats', name: 'GPS Sats', units: 'count', format: 'integer', hints: { range: 1 } },
    { key: 'gps_fix', name: 'GPS Fix', units: '', format: 'integer', hints: { range: 1 } },
    { key: 'lat', name: 'Latitude', units: 'deg', format: 'float', hints: { range: 1 } },
    { key: 'lon', name: 'Longitude', units: 'deg', format: 'float', hints: { range: 1 } },
    { key: 'batt_v', name: 'Battery', units: 'V', format: 'float', hints: { range: 1 } },
    { key: 'temp_c', name: 'Temp', units: 'C', format: 'float', hints: { range: 1 } },
    { key: 'imu_temp_c', name: 'IMU Temp', units: 'C', format: 'float', hints: { range: 1 } },
    { key: 'baro_temp_c', name: 'Baro Temp', units: 'C', format: 'float', hints: { range: 1 } },
    { key: 'die_temp_c', name: 'Die Temp', units: 'C', format: 'float', hints: { range: 1 } },
    { key: 'health', name: 'Health', units: '', format: 'integer', hints: { range: 1 } },
    { key: 'rssi', name: 'RSSI', units: 'dBm', format: 'float', hints: { range: 1 } },
    { key: 'snr', name: 'SNR', units: 'dB', format: 'float', hints: { range: 1 } },
    { key: 'lq_pct', name: 'Link Quality', units: '%', format: 'float', hints: { range: 1 } },
    { key: 'rx_hz', name: 'RX Rate', units: 'Hz', format: 'float', hints: { range: 1 } }
  ];

  function idFor(key) { return { namespace: NAMESPACE, key: key }; }

  const MASTER_STACKED = {
    identifier: idFor('master-stacked'),
    name: 'Master Glance (stacked)',
    type: 'telemetry.plot.stacked',
    location: NAMESPACE + ':master',
    composition: [
      idFor('flight_state'), idFor('alt_m'), idFor('vvel_mps'),
      idFor('accel_g'), idFor('rssi'), idFor('batt_v')
    ],
    configuration: { series: [], yAxis: {}, xAxis: {} }
  };

  const TRAJ = {
    identifier: idFor('link-overlay-traj'),
    name: 'Trajectory (alt + baro AGL)',
    type: 'telemetry.plot.overlay',
    location: NAMESPACE + ':master',
    composition: [idFor('alt_m'), idFor('baro_alt_m'), idFor('max_alt_m')],
    configuration: {
      series: [
        { identifier: idFor('alt_m') },
        { identifier: idFor('baro_alt_m') },
        { identifier: idFor('max_alt_m') }
      ]
    }
  };

  const DYN = {
    identifier: idFor('link-overlay-dyn'),
    name: 'Dynamics (vvel + speed + accel)',
    type: 'telemetry.plot.overlay',
    location: NAMESPACE + ':master',
    composition: [idFor('vvel_mps'), idFor('speed_mps'), idFor('accel_g')],
    configuration: {
      series: [
        { identifier: idFor('vvel_mps') },
        { identifier: idFor('speed_mps') },
        { identifier: idFor('accel_g') }
      ]
    }
  };

  const RADIO = {
    identifier: idFor('link-overlay-radio'),
    name: 'Radio (RSSI+SNR)',
    type: 'telemetry.plot.overlay',
    location: NAMESPACE + ':master',
    composition: [idFor('rssi'), idFor('snr')],
    configuration: {
      series: [{ identifier: idFor('rssi') }, { identifier: idFor('snr') }]
    }
  };

  const VEHICLE = {
    identifier: idFor('link-overlay-vehicle'),
    name: 'Vehicle (batt + temps)',
    type: 'telemetry.plot.overlay',
    location: NAMESPACE + ':master',
    composition: [idFor('batt_v'), idFor('imu_temp_c'), idFor('baro_temp_c'), idFor('die_temp_c')],
    configuration: {
      series: [
        { identifier: idFor('batt_v') },
        { identifier: idFor('imu_temp_c') },
        { identifier: idFor('baro_temp_c') },
        { identifier: idFor('die_temp_c') }
      ]
    }
  };

  const LAYOUTS = {
    'master-stacked': MASTER_STACKED,
    'link-overlay-traj': TRAJ,
    'link-overlay-dyn': DYN,
    'link-overlay-radio': RADIO,
    'link-overlay-vehicle': VEHICLE,
    // keep old key as alias to master stacked
    'link-stacked': Object.assign({}, MASTER_STACKED, {
      identifier: idFor('link-stacked'),
      name: 'Link Live (stacked)',
      location: NAMESPACE + ':link'
    })
  };

  function RcCsvDictionaryPlugin(options) {
    options = options || {};
    const navigateOnStart = options.navigateOnStart !== false;

    return function install(openmct) {
      const rootLink = { namespace: NAMESPACE, key: 'link' };
      const rootMaster = { namespace: NAMESPACE, key: 'master' };

      openmct.objects.addRoot(rootMaster);
      openmct.objects.addRoot(rootLink);

      openmct.objects.addProvider(NAMESPACE, {
        get: function (identifier) {
          if (identifier.key === 'master') {
            return Promise.resolve({
              identifier: rootMaster,
              name: 'Master Dashboard v1',
              type: 'folder',
              location: 'ROOT',
              composition: [
                idFor('master-stacked'),
                idFor('link-overlay-traj'),
                idFor('link-overlay-dyn'),
                idFor('link-overlay-radio'),
                idFor('link-overlay-vehicle')
              ].concat(MEASUREMENTS.map(function (m) { return idFor(m.key); }))
            });
          }
          if (identifier.key === 'link') {
            return Promise.resolve({
              identifier: rootLink,
              name: 'Rocket Chip Link (CSV)',
              type: 'folder',
              location: 'ROOT',
              composition: MEASUREMENTS.map(function (m) { return idFor(m.key); })
                .concat([idFor('link-stacked'), idFor('link-overlay-traj'), idFor('link-overlay-radio')])
            });
          }
          if (LAYOUTS[identifier.key]) {
            return Promise.resolve(LAYOUTS[identifier.key]);
          }
          const m = MEASUREMENTS.find(function (x) { return x.key === identifier.key; });
          if (!m) return Promise.reject(new Error('Unknown object ' + identifier.key));
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
            location: NAMESPACE + ':master'
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
        description: 'Live scrape / Big Daddy fidelity facsimile',
        cssClass: 'icon-telemetry'
      });

      if (navigateOnStart) {
        openmct.on('start', function () {
          try {
            var hash = window.location.hash || '';
            if (hash.indexOf('/browse/') >= 0 && hash.indexOf('master-stacked') < 0 &&
                hash.indexOf('link-stacked') < 0 && hash.indexOf('link-overlay') < 0) {
              return;
            }
            var path = '/browse/' + NAMESPACE + ':master-stacked';
            if (openmct.router && typeof openmct.router.setPath === 'function') {
              openmct.router.setPath(path);
            } else {
              window.location.hash = '#' + path;
            }
          } catch (e) {
            console.warn('[rc-dict] navigate failed', e);
          }
        });
      }
    };
  }

  global.RcCsvDictionaryPlugin = RcCsvDictionaryPlugin;
})(typeof window !== 'undefined' ? window : globalThis);
