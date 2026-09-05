/**
 * Rocket Chip Master Dashboard — Flexible Layout home (MCS-style).
 * Discrete → LAD; limits → gauges; trends → plots (dual Y for RSSI/SNR);
 * master-caution Summary Widgets along the bottom.
 */
(function (global) {
  const NAMESPACE = 'rocket-chip.hello';
  const KS = function (key) { return NAMESPACE + ':' + key; };

  const FLIGHT_ENUMS = [
    { value: 1, string: 'ARMED' },
    { value: 2, string: 'BOOST' },
    { value: 3, string: 'COAST' },
    { value: 4, string: 'DESCENT' },
    { value: 6, string: 'LANDED' }
  ];
  const PHASE_ENUMS = [
    { value: 0, string: '-' },
    { value: 1, string: 'EDGE' }
  ];
  const BOOL_ENUMS = [
    { value: 0, string: 'NO' },
    { value: 1, string: 'YES' }
  ];

  const MEASUREMENTS = [
    { key: 'seq', name: 'Sequence', units: 'count', format: 'integer', hints: { range: 1 } },
    { key: 'met_ms', name: 'MET', units: 'ms', format: 'integer', hints: { range: 1 } },
    { key: 'flight_state', name: 'Flight State', units: '', format: 'enum', hints: { range: 1 }, enumerations: FLIGHT_ENUMS },
    { key: 'phase_event', name: 'Phase Event', units: '', format: 'enum', hints: { range: 1 }, enumerations: PHASE_ENUMS },
    { key: 'chute_detected', name: 'Chute Detected', units: '', format: 'enum', hints: { range: 1 }, enumerations: BOOL_ENUMS },
    { key: 'alt_m', name: 'Altitude MSL', units: 'm', format: 'float', hints: { range: 1 } },
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

  function gauge(key, name, min, max, low, high) {
    return {
      identifier: idFor('gauge-' + key),
      name: name,
      type: 'gauge',
      location: NAMESPACE + ':master',
      composition: [idFor(key)],
      configuration: {
        gaugeController: {
          gaugeType: 'meter-vertical',
          isDisplayMinMax: true,
          isDisplayCurVal: true,
          isDisplayUnits: true,
          isUseTelemetryLimits: false,
          limitLow: low,
          limitHigh: high,
          max: max,
          min: min,
          precision: 1
        }
      }
    };
  }

  function frame(id, key, size) {
    return {
      id: id,
      domainObjectIdentifier: idFor(key),
      size: size,
      noFrame: false
    };
  }

  function swRule(id, label, bg, conditions) {
    return {
      name: label,
      label: label,
      message: '',
      id: id,
      icon: ' ',
      style: {
        color: '#ffffff',
        'background-color': bg,
        'border-color': 'rgba(0,0,0,0)'
      },
      description: label,
      conditions: conditions,
      jsCondition: '',
      trigger: 'any',
      expanded: 'true'
    };
  }

  function caution(key, title, telemKey, alarmOp, alarmVal, alarmLabel, alarmBg) {
    const telem = KS(telemKey);
    return {
      identifier: idFor(key),
      name: title,
      type: 'summary-widget',
      location: NAMESPACE + ':master',
      composition: [idFor(telemKey)],
      openNewTab: 'thisTab',
      telemetry: {},
      configuration: {
        ruleOrder: ['default', 'alarm'],
        ruleConfigById: {
          default: swRule('default', title + ' OK', '#38761d', []),
          alarm: swRule('alarm', alarmLabel, alarmBg, [{
            object: telem,
            key: 'value',
            operation: alarmOp,
            values: [String(alarmVal)]
          }])
        },
        testDataConfig: [{ object: '', key: '', value: '' }]
      }
    };
  }

  const PHASE_TABLE = {
    identifier: idFor('master-phase'),
    name: 'Phase / Status (LAD)',
    type: 'LadTable',
    location: NAMESPACE + ':master',
    composition: [
      idFor('flight_state'), idFor('phase_event'), idFor('chute_detected'),
      idFor('met_ms'), idFor('gps_fix'), idFor('gps_sats'), idFor('health')
    ],
    configuration: {}
  };

  const TRAJ = {
    identifier: idFor('link-overlay-traj'),
    name: 'Trajectory (alt + baro)',
    type: 'telemetry.plot.overlay',
    location: NAMESPACE + ':master',
    composition: [idFor('alt_m'), idFor('baro_alt_m')],
    configuration: {
      series: [
        { identifier: idFor('alt_m'), yAxisId: 1 },
        { identifier: idFor('baro_alt_m'), yAxisId: 1 }
      ],
      yAxis: { id: 1 }
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
        { identifier: idFor('vvel_mps'), yAxisId: 1 },
        { identifier: idFor('speed_mps'), yAxisId: 1 },
        { identifier: idFor('accel_g'), yAxisId: 1 }
      ],
      yAxis: { id: 1 }
    }
  };

  /** RSSI left axis (dBm), SNR right axis (dB) — like alt vs VSI. */
  const RADIO = {
    identifier: idFor('link-overlay-radio'),
    name: 'Radio (RSSI | SNR)',
    type: 'telemetry.plot.overlay',
    location: NAMESPACE + ':master',
    composition: [idFor('rssi'), idFor('snr')],
    configuration: {
      series: [
        { identifier: idFor('rssi'), yAxisId: 1 },
        { identifier: idFor('snr'), yAxisId: 2 }
      ],
      yAxis: { id: 1, label: 'RSSI (dBm)' },
      additionalYAxes: [
        { id: 2, label: 'SNR (dB)' }
      ]
    }
  };

  const G_BATT = gauge('batt_v', 'Battery gauge', 3.0, 4.2, 3.3, 4.1);
  const G_RSSI = gauge('rssi', 'RSSI gauge', -120, -20, -100, -40);
  const G_LQ = gauge('lq_pct', 'LQ gauge', 0, 100, 40, 90);

  const SW_RSSI = caution('sw-rssi', 'RSSI', 'rssi', 'lessThan', -100, 'RSSI LO', '#990000');
  const SW_LQ = caution('sw-lq', 'LQ', 'lq_pct', 'lessThan', 40, 'LQ LO', '#990000');
  const SW_BATT = caution('sw-batt', 'BATT', 'batt_v', 'lessThan', 3.5, 'BATT LO', '#990000');
  const SW_CHUTE = caution('sw-chute', 'CHUTE', 'chute_detected', 'equalTo', 1, 'CHUTE', '#b45f06');
  const SW_GPS = caution('sw-gps', 'GPS', 'gps_fix', 'lessThan', 3, 'NO FIX', '#990000');

  const MASTER_HOME = {
    identifier: idFor('master'),
    name: 'Master Dashboard',
    type: 'flexible-layout',
    location: 'ROOT',
    composition: [
      idFor('master-phase'),
      idFor('link-overlay-traj'),
      idFor('link-overlay-dyn'),
      idFor('link-overlay-radio'),
      idFor('gauge-rssi'),
      idFor('gauge-lq_pct'),
      idFor('gauge-batt_v'),
      idFor('sw-rssi'),
      idFor('sw-lq'),
      idFor('sw-batt'),
      idFor('sw-chute'),
      idFor('sw-gps')
    ],
    configuration: {
      rowsLayout: true,
      containers: [
        {
          id: 'rc-c-phase',
          size: 18,
          frames: [frame('rc-f-phase', 'master-phase', 100)]
        },
        {
          id: 'rc-c-plots',
          size: 36,
          frames: [
            frame('rc-f-traj', 'link-overlay-traj', 55),
            frame('rc-f-dyn', 'link-overlay-dyn', 45)
          ]
        },
        {
          id: 'rc-c-link',
          size: 28,
          frames: [
            frame('rc-f-radio', 'link-overlay-radio', 40),
            frame('rc-f-rssi', 'gauge-rssi', 20),
            frame('rc-f-lq', 'gauge-lq_pct', 20),
            frame('rc-f-batt', 'gauge-batt_v', 20)
          ]
        },
        {
          id: 'rc-c-caution',
          size: 18,
          frames: [
            frame('rc-f-sw-rssi', 'sw-rssi', 20),
            frame('rc-f-sw-lq', 'sw-lq', 20),
            frame('rc-f-sw-batt', 'sw-batt', 20),
            frame('rc-f-sw-chute', 'sw-chute', 20),
            frame('rc-f-sw-gps', 'sw-gps', 20)
          ]
        }
      ]
    }
  };

  const LAYOUTS = {
    'master': MASTER_HOME,
    'master-phase': PHASE_TABLE,
    'link-overlay-traj': TRAJ,
    'link-overlay-dyn': DYN,
    'link-overlay-radio': RADIO,
    'gauge-batt_v': G_BATT,
    'gauge-rssi': G_RSSI,
    'gauge-lq_pct': G_LQ,
    'sw-rssi': SW_RSSI,
    'sw-lq': SW_LQ,
    'sw-batt': SW_BATT,
    'sw-chute': SW_CHUTE,
    'sw-gps': SW_GPS
  };

  function RcCsvDictionaryPlugin(options) {
    options = options || {};
    const navigateOnStart = options.navigateOnStart !== false;

    return function install(openmct) {
      const rootMaster = { namespace: NAMESPACE, key: 'master' };
      const rootLink = { namespace: NAMESPACE, key: 'link' };

      openmct.objects.addRoot(rootMaster);
      openmct.objects.addRoot(rootLink);

      openmct.objects.addProvider(NAMESPACE, {
        get: function (identifier) {
          if (LAYOUTS[identifier.key]) {
            return Promise.resolve(LAYOUTS[identifier.key]);
          }
          if (identifier.key === 'link') {
            return Promise.resolve({
              identifier: rootLink,
              name: 'All measurements',
              type: 'folder',
              location: 'ROOT',
              composition: MEASUREMENTS.map(function (m) { return idFor(m.key); })
            });
          }
          const m = MEASUREMENTS.find(function (x) { return x.key === identifier.key; });
          if (!m) return Promise.reject(new Error('Unknown ' + identifier.key));
          const valueMeta = {
            key: 'value',
            name: m.name,
            units: m.units,
            format: m.format,
            hints: m.hints
          };
          if (m.enumerations) valueMeta.enumerations = m.enumerations;
          return Promise.resolve({
            identifier: { namespace: NAMESPACE, key: m.key },
            name: m.name,
            type: 'rocket-chip.telemetry',
            telemetry: {
              values: [
                { key: 'utc', source: 'timestamp', name: 'Timestamp', format: 'utc', hints: { domain: 1 } },
                valueMeta
              ]
            },
            location: NAMESPACE + ':master'
          });
        }
      });

      openmct.composition.addProvider({
        appliesTo: function (o) {
          return o.identifier.namespace === NAMESPACE &&
            (o.type === 'folder' || o.type === 'table' || o.type === 'LadTable' ||
             o.type === 'gauge' || o.type === 'summary-widget' ||
             o.type === 'flexible-layout' || o.type === 'display-layout' ||
             o.type === 'telemetry.plot.overlay' || o.type === 'telemetry.plot.stacked');
        },
        load: function (o) { return Promise.resolve(o.composition || []); }
      });

      openmct.types.addType('rocket-chip.telemetry', {
        name: 'Rocket Chip Telemetry',
        description: 'Facsimile / live CSV point',
        cssClass: 'icon-telemetry'
      });

      if (navigateOnStart) {
        openmct.on('start', function () {
          try {
            var hash = window.location.hash || '';
            if (hash.indexOf('/browse/') >= 0 && hash.indexOf('master') < 0) return;
            var path = '/browse/' + NAMESPACE + ':master';
            if (openmct.router && openmct.router.setPath) openmct.router.setPath(path);
            else window.location.hash = '#' + path;
          } catch (e) { console.warn(e); }
        });
      }
    };
  }

  global.RcCsvDictionaryPlugin = RcCsvDictionaryPlugin;
})(typeof window !== 'undefined' ? window : globalThis);
