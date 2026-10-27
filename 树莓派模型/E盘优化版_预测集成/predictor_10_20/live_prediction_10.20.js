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
const SHORT_WINDOW_SAMPLES = 20_000;
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
    quality: { waveformCompatible: null, waveformComplete: false, predictionMode: null, warnings: [] }
  };

  if (!/\.pridb$/i.test(absPridb) || !fs.existsSync(absPridb)) {
    return { ...base, status: 'waiting_for_pridb', result: { ...base.result, distanceText: '等待采集数据库' } };
  }
  if (!fs.existsSync(absTradb)) {
    return { ...base, status: 'waiting_for_tradb', result: { ...base.result, distanceText: '等待配套波形数据库' } };
  }

  let readStage = '事件数据库 (.pridb)';
  try {
    const events = readEvents(absPridb);
    readStage = '波形数据库 (.tradb)';
    const wave = readWaveformSummary(absTradb);
    readStage = '预测计算';
    base.eventCount = events.length;
    base.waveformCount = wave.count;
    base.waveformShape = wave.shapes;
    base.quality.waveformComplete = wave.count >= events.length && events.length > 0;

    if (!events.length || !wave.count) {
      return { ...base, status: 'collecting', result: { ...base.result, distanceText: '正在等待有效事件和波形' } };
    }
    const oneRate = wave.shapes.length === 1 && wave.shapes[0].sampleRateHz === REFERENCE_RATE;
    const fullWindow = oneRate && wave.shapes[0].sampleCount === REFERENCE_SAMPLES;
    const shortWindow = oneRate && wave.shapes[0].sampleCount === SHORT_WINDOW_SAMPLES;
    const compatible = fullWindow || shortWindow;
    base.quality.waveformCompatible = compatible;
    base.quality.predictionMode = fullWindow ? 'full_multifeature' : shortWindow ? 'experimental_20k_rms' : null;
    if (!compatible) {
      const shapeText = wave.shapes.map(s => s.sampleRateHz + ' Hz / ' + s.sampleCount + ' 点').join('；');
      base.quality.warnings.push('当前波形：' + shapeText + '；可用模式为 2 MHz / 200,000 点，或实验性的 2 MHz / 20,000 点 RMS 模式。');
      return { ...base, status: 'incompatible_acquisition', result: { ...base.result, distanceText: '采样条件不匹配，不能预测距离' } };
    }
    if (events.length < MIN_EVENTS) {
      return { ...base, status: 'insufficient_events', result: { ...base.result, distanceText: '有效事件不足 30 条，继续采集' } };
    }

    const rms = median(events.map(e => e.rms));
    const amp = median(events.map(e => e.amp));
    const ss = median(events.map(e => e.ss));
    // 20k 窗口的 Amp / SS 与 200k 标定窗长不同，只使用 RMS。
    // 在 10.20 亚克力的 31 组记录中，截取前 20k 点后 RMS 中位数
    // 相对原始 200k RMS 中位数的最大偏差为 1.47%（内部核验，不是现场验证）。
    const estimated = shortWindow
      ? evaluateMultiFeatureDistance(rms, NaN, NaN, modelPack.model)
      : evaluateMultiFeatureDistance(rms, amp, ss, modelPack.model);
    const nearest = nearestReference(rms, modelPack.model.rms_reference_ranges);
    const calibrated = ['near_source_point', 'near_10cm_or_near_field', 'near_10cm_boundary', 'within_calibration_range', 'near_50cm_boundary'].includes(estimated.status);
    const displayable = (calibrated || estimated.status === 'beyond_50cm_outside_calibration')
      && estimated.reportedDistanceCm != null && estimated.reportedDistanceCm < 60;
    const hasPartialDischarge = shortWindow ? null
      : estimated.status === 'below_noise_or_beyond_50cm' ? false
      : calibrated ? true : null;
    const distanceCm = displayable ? estimated.reportedDistanceCm : null;
    let distanceText = estimated.rangeText;
    if (shortWindow && distanceCm != null) {
      distanceText = '实验性 RMS 距离估算：' + distanceText;
    }
    if (shortWindow) {
      base.quality.warnings.push('实验性 20,000 点 RMS 单特征估算：仅在 10.20 亚克力标定文件内部核验；旧文件的探头、增益、滤波和耦合条件未证实一致，请用已知距离实测复核。');
    }
    if (distanceCm > 50 && distanceCm < 60) {
      base.quality.warnings.push('50–60 cm 是由 10–50 cm 拟合线外推的未标定估算，须用已知距离样本复核。');
    }
    if (estimated.status === 'near_10cm_boundary') {
      base.quality.warnings.push('0–10 cm 只有 0 和 10 cm 标定点；10 cm 边界附近数值未经 3/5 cm 样本验证。');
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
    const message = String(error.message || error);
    const busy = /SQLITE_BUSY|SQLITE_LOCKED|database is locked|database is busy/i.test(message);
    const missingTable = /no such table/i.test(message);
    const malformed = /database disk image is malformed|SQLITE_CORRUPT|not a database/i.test(message);
    return {
      ...base, status: busy ? 'temporarily_unavailable' : 'prediction_read_error',
      result: { ...base.result, distanceText: busy
        ? '数据库暂时被占用，稍后自动重试'
        : malformed ? readStage + '损坏或文件不完整，无法预测距离'
        : missingTable ? '文件缺少预测所需数据表，可能尚未初始化或文件不完整'
          : '数据库读取失败，无法预测距离；具体原因见警告' },
      quality: { ...base.quality, warnings: [readStage + '：' + message] }
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
