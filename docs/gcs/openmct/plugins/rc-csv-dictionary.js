/**
 * Rocket Chip Master Dashboard — QGC/Mission Planner-style instrument ring.
 * Primary pane = trajectory; ring = phase LAD + vehicle gauges + Condition Set
 * alphanumerics for link/power; drill-down = dynamics. Radio overlay kept in
 * dict for a later dedicated RF page (not on home — plots barely move vs alt/VSI).
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

  function frame(id, key, size, noFrame) {
    return {
      id: id,
      domainObjectIdentifier: idFor(key),
      size: size,
      noFrame: !!noFrame
    };
  }

  /** Guide-style Condition Set: alarm → warn → default (first match wins). */
  function conditionSet(key, title, telemKey, alarmOp, alarmVal, warnOp, warnVal) {
    const telemKs = KS(telemKey);
    const alarmId = key + '-alarm';
    const warnId = key + '-warn';
    const okId = key + '-ok';
    const staleId = key + '-stale';
    return {
      identifier: idFor(key),
      name: title,
      type: 'conditionSet',
      location: NAMESPACE + ':master',
      composition: [idFor(telemKey)],
      configuration: {
        conditionCollection: [
          {
            // First, so a silent link never shows its last good (green) value.
            id: staleId,
            configuration: {
              name: 'NO DATA',
              output: 'NO DATA',
              trigger: 'all',
              criteria: [{
                id: staleId + '-c0',
                telemetry: telemKs,
                operation: 'isStale',
                input: [String(STALE_S)],
                metadata: 'dataReceived'
              }]
            },
            summary: title + ' no data (no update for ' + STALE_S + ' s)'
          },
          {
            id: alarmId,
            configuration: {
              name: 'LO',
              output: 'LO',
              trigger: 'all',
              criteria: [{
                id: alarmId + '-c0',
                telemetry: telemKs,
                operation: alarmOp,
                input: [String(alarmVal)],
                metadata: 'value'
              }]
            },
            summary: title + ' alarm'
          },
          {
            id: warnId,
            configuration: {
              name: 'WARN',
              output: 'WARN',
              trigger: 'all',
              criteria: [{
                id: warnId + '-c0',
                telemetry: telemKs,
                operation: warnOp,
                input: [String(warnVal)],
                metadata: 'value'
              }]
            },
            summary: title + ' warn'
          },
          {
            isDefault: true,
            id: okId,
            configuration: {
              name: 'OK',
              output: 'OK',
              trigger: 'all',
              criteria: []
            },
            summary: 'Default'
          }
        ]
      },
      telemetry: {}
    };
  }

  // NO DATA: no update for this long X-es the box out (grey, white X, dimmed text), like the
  // avionics "X through a field not receiving valid data". Also the state before any data
  // arrives. Station link is ~10 Hz; 3 s leaves room for a slower USB dash scrape.
  const STALE_S = 3;
  const X_LINE = 'transparent calc(50% - 1.5px), #ffffff calc(50% - 1.5px), #ffffff calc(50% + 1.5px), transparent calc(50% + 1.5px)';
  const S_XOUT = {
    backgroundColor: '#4d4d4d', border: '1px solid #9e9e9e', color: 'rgba(255,255,255,0.35)',
    backgroundImage: 'linear-gradient(to top right, ' + X_LINE + '), linear-gradient(to bottom right, ' + X_LINE + ')'
  };
  const S_OK = { backgroundColor: '#38761d', border: 'rgba(0,0,0,0)', color: '#ffffff', backgroundImage: 'none' };
  const S_WARN = { backgroundColor: '#b45f06', border: 'rgba(0,0,0,0)', color: '#ffffff', backgroundImage: 'none' };
  const S_ALARM = { backgroundColor: '#990000', border: 'rgba(0,0,0,0)', color: '#ffffff', backgroundImage: 'none' };

  /** objectStyles block: entries = [[conditionId, output, style], ...]; last entry is the default. */
  function styleBlock(csKey, entries) {
    const def = entries[entries.length - 1];
    return {
      conditionSetIdentifier: idFor(csKey),
      selectedConditionId: '',
      defaultConditionId: def[0],
      staticStyle: { style: Object.assign({}, S_XOUT) },
      styles: entries.map(function (e) {
        return { conditionId: e[0], style: Object.assign({ output: e[1] }, e[2]) };
      })
    };
  }

  function alphaStyle(csKey, alarmId, warnId, okId) {
    return styleBlock(csKey, [
      [csKey + '-stale', 'NO DATA', S_XOUT],
      [alarmId, 'LO', S_ALARM],
      [warnId, 'WARN', S_WARN],
      [okId, 'OK', S_OK]
    ]);
  }

  /**
   * Status Condition Set for a labelled caution box: NO DATA first, then rules in order
   * ({ id, output, op, val }), then the default output.
   */
  function statusSet(key, title, telemKey, rules, defOutput) {
    const telemKs = KS(telemKey);
    const coll = [{
      id: key + '-stale',
      configuration: {
        name: 'NO DATA', output: title, trigger: 'all',
        criteria: [{ id: key + '-stale-c0', telemetry: telemKs, operation: 'isStale',
          input: [String(STALE_S)], metadata: 'dataReceived' }]
      },
      summary: title + ' no data'
    }];
    rules.forEach(function (r) {
      coll.push({
        id: key + '-' + r.id,
        configuration: {
          name: r.output, output: r.output, trigger: 'all',
          criteria: [{ id: key + '-' + r.id + '-c0', telemetry: telemKs, operation: r.op,
            input: [String(r.val)], metadata: 'value' }]
        },
        summary: r.output
      });
    });
    coll.push({
      isDefault: true, id: key + '-ok',
      configuration: { name: defOutput, output: defOutput, trigger: 'all', criteria: [] },
      summary: 'Default'
    });
    return {
      identifier: idFor(key), name: title + ' conditions', type: 'conditionSet',
      location: NAMESPACE + ':master', composition: [idFor(telemKey)],
      configuration: { conditionCollection: coll }, telemetry: {}
    };
  }

  /** Condition Widget whose label is the Condition Set output, styled per condition. */
  function statusWidget(key, title, csKey, entries) {
    return {
      identifier: idFor(key), name: title, type: 'conditionWidget',
      location: NAMESPACE + ':master', label: title,
      configuration: { useConditionSetOutputAsLabel: true, objectStyles: styleBlock(csKey, entries) }
    };
  }

  // 2x2 grid sized to the ~40-cell-wide Link / Power frame (grid cell = 10 px).
  function alphaItem(id, telemKey, x, y) {
    return {
      id: id,
      type: 'telemetry-view',
      identifier: idFor(telemKey),
      x: x,
      y: y,
      width: 18,
      height: 7,
      displayMode: 'all',
      value: 'value',
      stroke: '',
      fill: '',
      color: '',
      fontSize: '16px',
      font: 'default',
      showUnits: true
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

  /** Parked for dedicated RF page — not on Master home (Nathan 2026-09-23). */
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

  const G_BARO = gauge('baro_alt_m', 'Baro AGL gauge', -50, 4000, 0, 3500);
  const G_VVEL = gauge('vvel_mps', 'VVel gauge', -100, 100, -80, 80);
  const G_SPEED = gauge('speed_mps', 'Speed gauge', 0, 400, 0, 350);

  const CS_CHUTE = statusSet('cs-chute', 'CHUTE', 'chute_detected',
    [{ id: 'out', output: 'CHUTE DEPLOYED', op: 'equalTo', val: 1 }], 'CHUTE STOWED');
  const CS_GPS = statusSet('cs-gps', 'GPS', 'gps_fix',
    [{ id: 'nofix', output: 'GPS NO FIX', op: 'lessThan', val: 2 },
     { id: '2d', output: 'GPS 2D FIX', op: 'lessThan', val: 3 }], 'GPS 3D FIX');
  const CW_CHUTE = statusWidget('cw-chute', 'CHUTE', 'cs-chute', [
    ['cs-chute-stale', 'CHUTE', S_XOUT],
    ['cs-chute-out', 'CHUTE DEPLOYED', S_WARN],
    ['cs-chute-ok', 'CHUTE STOWED', S_OK]
  ]);
  const CW_GPS = statusWidget('cw-gps', 'GPS', 'cs-gps', [
    ['cs-gps-stale', 'GPS', S_XOUT],
    ['cs-gps-nofix', 'GPS NO FIX', S_ALARM],
    ['cs-gps-2d', 'GPS 2D FIX', S_WARN],
    ['cs-gps-ok', 'GPS 3D FIX', S_OK]
  ]);

  // SNR limits assume LoRa SF7 (default radio_config.h; demod floor ~-7.5 dB):
  // red ~2.5 dB above floor, amber 5 dB above red. RSSI: SF7/BW125 sensitivity ~-123 dBm,
  // red ~8 dB above, amber 10 dB above red. Retune both if SF/BW changes; FSK needs its own.
  const CS_RSSI = conditionSet('cs-rssi', 'RSSI conditions', 'rssi', 'lessThan', -115, 'lessThan', -105);
  const CS_SNR = conditionSet('cs-snr', 'SNR conditions', 'snr', 'lessThan', -5, 'lessThan', 0);
  const CS_LQ = conditionSet('cs-lq', 'LQ conditions', 'lq_pct', 'lessThan', 40, 'lessThan', 60);
  const CS_BATT = conditionSet('cs-batt', 'Battery conditions', 'batt_v', 'lessThan', 3.5, 'lessThan', 3.7);

  const LINK_ALPHA = {
    identifier: idFor('master-link-alpha'),
    name: 'Link / Power (alphas)',
    type: 'layout',
    location: NAMESPACE + ':master',
    composition: [
      idFor('rssi'), idFor('snr'), idFor('lq_pct'), idFor('batt_v'),
      idFor('cs-rssi'), idFor('cs-snr'), idFor('cs-lq'), idFor('cs-batt')
    ],
    configuration: {
      layoutGrid: [10, 10],
      items: [
        alphaItem('li-alpha-rssi', 'rssi', 1, 1),
        alphaItem('li-alpha-snr', 'snr', 20, 1),
        alphaItem('li-alpha-lq', 'lq_pct', 1, 9),
        alphaItem('li-alpha-batt', 'batt_v', 20, 9)
      ],
      objectStyles: {
        'li-alpha-rssi': alphaStyle('cs-rssi', 'cs-rssi-alarm', 'cs-rssi-warn', 'cs-rssi-ok'),
        'li-alpha-snr': alphaStyle('cs-snr', 'cs-snr-alarm', 'cs-snr-warn', 'cs-snr-ok'),
        'li-alpha-lq': alphaStyle('cs-lq', 'cs-lq-alarm', 'cs-lq-warn', 'cs-lq-ok'),
        'li-alpha-batt': alphaStyle('cs-batt', 'cs-batt-alarm', 'cs-batt-warn', 'cs-batt-ok')
      }
    }
  };

  function layoutItem(id, key, x, width) {
    return {
      id: id,
      type: 'subobject-view',
      identifier: idFor(key),
      x: x,
      y: 1,
      width: width,
      height: 5,
      hasFrame: false
    };
  }

  const MASTER_CAUTION = {
    identifier: idFor('master-caution'),
    name: 'Master Caution',
    type: 'layout',
    location: NAMESPACE + ':master',
    composition: [
      idFor('cw-chute'), idFor('cw-gps'), idFor('cs-chute'), idFor('cs-gps')
    ],
    configuration: {
      layoutGrid: [10, 10],
      items: [
        layoutItem('li-cw-chute', 'cw-chute', 1, 20),
        layoutItem('li-cw-gps', 'cw-gps', 22, 20)
      ]
    }
  };

  const MASTER_HOME = {
    identifier: idFor('master'),
    name: 'Master Dashboard',
    type: 'flexible-layout',
    location: 'ROOT',
    composition: [
      idFor('master-phase'),
      idFor('link-overlay-traj'),
      idFor('link-overlay-dyn'),
      idFor('gauge-baro_alt_m'),
      idFor('gauge-vvel_mps'),
      idFor('gauge-speed_mps'),
      idFor('master-link-alpha'),
      idFor('cs-rssi'),
      idFor('cs-snr'),
      idFor('cs-lq'),
      idFor('cs-batt'),
      idFor('master-caution'),
      idFor('cw-chute'),
      idFor('cw-gps'),
      idFor('cs-chute'),
      idFor('cs-gps')
    ],
    configuration: {
      rowsLayout: true,
      containers: [
        {
          id: 'rc-c-phase',
          size: 14,
          frames: [frame('rc-f-phase', 'master-phase', 100)]
        },
        {
          id: 'rc-c-ring',
          size: 48,
          frames: [
            frame('rc-f-baro', 'gauge-baro_alt_m', 10),
            frame('rc-f-vvel', 'gauge-vvel_mps', 10),
            frame('rc-f-speed', 'gauge-speed_mps', 10),
            frame('rc-f-traj', 'link-overlay-traj', 40),
            frame('rc-f-link', 'master-link-alpha', 30)
          ]
        },
        {
          id: 'rc-c-drill',
          size: 22,
          frames: [
            frame('rc-f-dyn', 'link-overlay-dyn', 100)
          ]
        },
        {
          id: 'rc-c-caution',
          size: 16,
          frames: [
            frame('rc-f-caution', 'master-caution', 100, true)
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
    'gauge-baro_alt_m': G_BARO,
    'gauge-vvel_mps': G_VVEL,
    'gauge-speed_mps': G_SPEED,
    'master-link-alpha': LINK_ALPHA,
    'cs-rssi': CS_RSSI,
    'cs-snr': CS_SNR,
    'cs-lq': CS_LQ,
    'cs-batt': CS_BATT,
    'master-caution': MASTER_CAUTION,
    'cs-chute': CS_CHUTE,
    'cs-gps': CS_GPS,
    'cw-chute': CW_CHUTE,
    'cw-gps': CW_GPS
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
            source: 'value',
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
             o.type === 'conditionSet' ||
             o.type === 'flexible-layout' || o.type === 'display-layout' || o.type === 'layout' ||
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
