// Read-only, lightweight provisional prediction for a growing acquisition pair.
// It shares the calibrated calculations with the desktop predictor, but never
// loads waveform blobs or changes the device acquisition process.
'use strict';

const fs = require('fs');
const path = require('path');
const { DatabaseSync } = require('node:sqlite');
const {
  loadModel, readEvents, median, nearestReference, evaluateMultiFeatureDistance
} = require('./predict_distance_10.20.js');

const REFERENCE_RATE = 2_000_000;
const REFERENCE_SAMPLES = 200_000;
const MIN_EVENTS = 30;

function readWaveformSummary(tradbPath) {
  const db = new DatabaseSync(tradbPath, { readOnly: true });
  try {
    const rows = db.prepare(
      'SELECT SampleRate, Samples, COUNT(*) AS n FROM tr_data GROUP BY SampleRate, Samples'
    ).all();
    return {
      count: rows.reduce((sum, row) => sum + Number(row.n), 0),
      shapes: rows.map(row => ({
        sampleRateHz: Number(row.SampleRate),
        sampleCount: Number(row.Samples),
        count: Number(row.n)
      }))
    };
  } finally {
    db.close();
  }
}

function predict(pridbPath) {
  const modelPack = loadModel();
  const absPridb = path.resolve(pridbPath);
  const absTradb = absPridb.replace(/\.pridb$/i, '.tradb');
  const base = {
    phase: 'provisional',
    modelDataset: modelPack.model.dataset,
    sourceFile: path.basename(absPridb),
    updatedAt: new Date().toISOString(),
    eventCount: 0,
    waveformCount: 0,
    waveformShape: [],
    result: { hasPartialDischarge: null, distanceCm: null, distanceText: '等待数据', nearestReferenceDistanceCm: null },
    quality: { waveformCompatible: null, waveformComplete: false, warnings: [] }
  };

  if (!/\.pridb$/i.test(absPridb) || !fs.existsSync(absPridb)) {
    return { ...base, status: 'waiting_for_pridb', result: { ...base.result, distanceText: '等待采集数据库' } };
  }
  if (!fs.existsSync(absTradb)) {
    return { ...base, status: 'waiting_for_tradb', result: { ...base.result, distanceText: '等待配套波形数据库' } };
  }

  try {
    const events = readEvents(absPridb);
    const wave = readWaveformSummary(absTradb);
    base.eventCount = events.length;
    base.waveformCount = wave.count;
    base.waveformShape = wave.shapes;
    base.quality.waveformComplete = wave.count >= events.length && events.length > 0;

    if (!events.length || !wave.count) {
      return { ...base, status: 'collecting', result: { ...base.result, distanceText: '正在等待有效事件和波形' } };
    }
    const compatible = wave.shapes.length === 1
      && wave.shapes[0].sampleRateHz === REFERENCE_RATE
      && wave.shapes[0].sampleCount === REFERENCE_SAMPLES;
    base.quality.waveformCompatible = compatible;
    if (!compatible) {
      const shapeText = wave.shapes.map(s => s.sampleRateHz + ' Hz / ' + s.sampleCount + ' 点').join('；');
      base.quality.warnings.push('当前波形：' + shapeText + '；10.20 标准要求 2 MHz / 200,000 点。');
      return { ...base, status: 'incompatible_acquisition', result: { ...base.result, distanceText: '采样条件不匹配，不能预测距离' } };
    }
    if (events.length < MIN_EVENTS) {
      return { ...base, status: 'insufficient_events', result: { ...base.result, distanceText: '有效事件不足 30 条，继续采集' } };
    }

    const rms = median(events.map(e => e.rms));
    const amp = median(events.map(e => e.amp));
    const ss = median(events.map(e => e.ss));
    const estimated = evaluateMultiFeatureDistance(rms, amp, ss, modelPack.model);
    const nearest = nearestReference(rms, modelPack.model.rms_reference_ranges);
    const calibrated = ['near_source_point', 'near_10cm_or_near_field', 'within_calibration_range'].includes(estimated.status);
    const hasPartialDischarge = estimated.status === 'below_noise_or_beyond_50cm' ? false
      : calibrated ? true : null;
    const distanceCm = calibrated ? estimated.reportedDistanceCm : null;
    let distanceText = estimated.rangeText;
    if (!calibrated && estimated.reportedDistanceCm != null) {
      distanceText = '超出 10.20 标定范围，不显示数值距离';
    }
    base.quality.warnings.push('实时估算未读取完整波形频谱；采集结束后仍需复核。');
    if (!base.quality.waveformComplete) {
      base.quality.warnings.push('特征与波形数量尚未齐全，结果可能继续变化。');
    }
    return {
      ...base,
      status: estimated.status,
      features: { rmsMedian: rms, amplitudeMedian: amp, signalStrengthMedian: ss },
      result: {
        hasPartialDischarge, distanceCm, distanceText,
        nearestReferenceDistanceCm: distanceCm == null ? null : nearest?.distanceCm ?? null
      }
    };
  } catch (error) {
    return {
      ...base, status: 'temporarily_unavailable',
      result: { ...base.result, distanceText: '数据库正在写入，稍后重试' },
      quality: { ...base.quality, warnings: [String(error.message)] }
    };
  }
}

if (require.main === module) {
  const file = process.argv[2];
  if (!file) {
    console.error('用法: node live_prediction_10.20.js <当前采集.pridb>');
    process.exitCode = 2;
  } else {
    try {
      console.log(JSON.stringify(predict(file)));
    } catch (error) {
      console.error(error.message);
      process.exitCode = 1;
    }
  }
}

module.exports = { predict };
