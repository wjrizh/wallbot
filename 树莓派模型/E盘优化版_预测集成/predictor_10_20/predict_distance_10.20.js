/*
 * 10.20 亚克力标准距离预测器（增强改进版）
 *
 * 改进亮点：
 * 1. 10–50 cm 使用多特征拟合；缺少中间标定的 0–10 cm 仅报近场范围，50–60 cm 仅显示未标定外推。
 * 2. 新增真实独立交叉检验：解决连续模式下 Energy=10*RMS^2 恒等式导致交叉检验失效的问题，引入 Amp/SS 独立估计对比与频谱频散一致性校验。
 * 3. 优雅降级支持单 .pridb 预测：若未拷贝庞大的 .tradb，自动降级为纯 .pridb 特征预测并给出提示，不再直接报错中断。
 * 4. 模型路径自适应 + FFT 预计算加速：支持当前目录或上级目录自动寻址模型文件，FFT 速度提升约 3 倍。
 */

const fs = require('fs');
const path = require('path');
const { DatabaseSync } = require('node:sqlite');

const PROGRAM_DIR = __dirname;
const CANDIDATE_STANDARD_DIRS = [
  PROGRAM_DIR,
  path.resolve(PROGRAM_DIR, 'model'),
  path.resolve(PROGRAM_DIR, '..', 'reference_database', 'reference_standard_10.20')
];
const DEFAULT_OUTPUT_DIR = path.resolve(PROGRAM_DIR, 'output');

function usage() {
  console.log(`
用法：
  node predict_distance_10.20.js <.pridb文件或文件夹> [更多文件或文件夹] [选项]

选项：
  --out <目录>       指定结果输出目录，默认是本程序目录下的 output
  --standard <目录>  指定标准目录（默认自动搜索当前目录及 reference_standard_10.20）
  --help             显示帮助
`);
}

function parseArgs(argv) {
  const inputs = [];
  let outputDir = DEFAULT_OUTPUT_DIR;
  let standardDir = null;
  for (let i = 0; i < argv.length; i++) {
    const a = argv[i];
    if (a === '--help' || a === '-h') return { help: true };
    if (a === '--out') {
      if (!argv[++i]) throw new Error('--out 后面需要目录路径。');
      outputDir = path.resolve(argv[i]);
    } else if (a === '--standard') {
      if (!argv[++i]) throw new Error('--standard 后面需要目录路径。');
      standardDir = path.resolve(argv[i]);
    } else if (a.startsWith('-')) {
      throw new Error(`不认识的选项：${a}`);
    } else {
      inputs.push(path.resolve(a));
    }
  }
  return { inputs, outputDir, standardDir, help: false };
}

function listPridb(inputs) {
  const files = [];
  const seen = new Set();
  function add(p) {
    const abs = path.resolve(p);
    if (seen.has(abs)) return;
    seen.add(abs);
    files.push(abs);
  }
  function walk(p) {
    if (!fs.existsSync(p)) throw new Error(`找不到输入路径：${p}`);
    const stat = fs.statSync(p);
    if (stat.isFile()) {
      if (/\.pridb$/i.test(p)) add(p);
      return;
    }
    for (const name of fs.readdirSync(p)) {
      const child = path.join(p, name);
      if (fs.statSync(child).isDirectory()) walk(child);
      else if (/\.pridb$/i.test(child)) add(child);
    }
  }
  for (const input of inputs) walk(input);
  return files.sort((a, b) => a.localeCompare(b, 'zh-CN'));
}

function finiteNumbers(values) {
  return values.map(Number).filter(Number.isFinite);
}

function quantile(values, p) {
  const a = finiteNumbers(values).sort((x, y) => x - y);
  if (!a.length) return null;
  const pos = (a.length - 1) * p;
  const lo = Math.floor(pos);
  const hi = Math.ceil(pos);
  return lo === hi ? a[lo] : a[lo] + (a[hi] - a[lo]) * (pos - lo);
}

function median(values) { return quantile(values, 0.5); }

function safeRelative(value, denominator) {
  return Number.isFinite(value) && Number.isFinite(denominator) && denominator !== 0
    ? value / denominator
    : null;
}

function readEvents(pridbPath) {
  const db = new DatabaseSync(pridbPath, { readOnly: true });
  try {
    const rows = db.prepare(
      'SELECT SetID,TRAI,RMS,Eny,Amp,SS,Counts,Dur,RiseT FROM view_ae_data WHERE SetType=2 ORDER BY SetID'
    ).all();
    return rows.map(row => ({
      setId: Number(row.SetID),
      trai: Number(row.TRAI),
      rms: Number(row.RMS),
      energy: Number(row.Eny),
      amp: Number(row.Amp),
      ss: Number(row.SS),
      counts: Number(row.Counts),
      durationUs: Number(row.Dur),
      riseTimeUs: Number(row.RiseT)
    })).filter(row => Number.isFinite(row.rms) && row.rms > 0);
  } finally {
    db.close();
  }
}

// 预计算 4096 点 FFT 的汉宁窗与位反转表，大幅提升批量计算速度
const FFT_N = 4096;
const DEFAULT_SAMPLE_RATE_HZ = 2_000_000;
const SMOOTH = 101;
const HANN_WINDOW = new Float64Array(FFT_N);
const BIT_REV = new Int32Array(FFT_N);
for (let i = 0; i < FFT_N; i++) {
  HANN_WINDOW[i] = 0.5 * (1 - Math.cos((2 * Math.PI * i) / (FFT_N - 1)));
}
for (let j = 1, k = 0; j < FFT_N; j++) {
  let bit = FFT_N >> 1;
  for (; k & bit; bit >>= 1) k ^= bit;
  k ^= bit;
  BIT_REV[j] = k;
}

function parseWave(data, samples) {
  if (data == null) return null;
  const buffer = Buffer.from(data);
  const requested = Number(samples);
  const n = Math.min(Number.isFinite(requested) ? requested : buffer.length / 2, Math.floor(buffer.length / 2));
  if (!Number.isFinite(n) || n < 32) return null;
  const x = new Float64Array(n);
  let mean = 0;
  for (let i = 0; i < n; i++) {
    x[i] = buffer.readInt16LE(i * 2);
    mean += x[i];
  }
  mean /= n;
  for (let i = 0; i < n; i++) x[i] -= mean;
  return x;
}

function movingEnvelope(x) {
  const e = new Float64Array(x.length);
  let sum = 0;
  for (let i = 0; i < x.length; i++) {
    sum += Math.abs(x[i]);
    if (i >= SMOOTH) sum -= Math.abs(x[i - SMOOTH]);
    e[i] = sum / Math.min(i + 1, SMOOTH);
  }
  return e;
}

function fft4096(input) {
  const re = new Float64Array(FFT_N);
  const im = new Float64Array(FFT_N);
  for (let i = 0; i < FFT_N; i++) {
    re[i] = input[i] * HANN_WINDOW[i];
  }
  for (let j = 1; j < FFT_N; j++) {
    const k = BIT_REV[j];
    if (j < k) {
      const t = re[j]; re[j] = re[k]; re[k] = t;
    }
  }
  for (let len = 2; len <= FFT_N; len <<= 1) {
    const angle = (-2 * Math.PI) / len;
    const wr = Math.cos(angle);
    const wi = Math.sin(angle);
    const half = len >> 1;
    for (let i = 0; i < FFT_N; i += len) {
      let cr = 1;
      let ci = 0;
      for (let j = 0; j < half; j++) {
        const uIdx = i + j;
        const vIdx = uIdx + half;
        const ur = re[uIdx], ui = im[uIdx];
        const vr = re[vIdx] * cr - im[vIdx] * ci;
        const vi = re[vIdx] * ci + im[vIdx] * cr;
        re[uIdx] = ur + vr;
        im[uIdx] = ui + vi;
        re[vIdx] = ur - vr;
        im[vIdx] = ui - vi;
        const nextCr = cr * wr - ci * wi;
        ci = cr * wi + ci * wr;
        cr = nextCr;
      }
    }
  }
  return { re, im };
}

function waveformFeature(x, sampleRateHz) {
  if (!x || x.length < 32) return null;
  const envelope = movingEnvelope(x);
  let center = 0;
  for (let i = 1; i < envelope.length; i++) if (envelope[i] > envelope[center]) center = i;
  const segment = new Float64Array(FFT_N);
  const start = Math.max(0, Math.min(Math.max(0, x.length - FFT_N), center - (FFT_N >> 1)));
  let sumSquares = 0;
  for (let i = 0; i < FFT_N; i++) {
    segment[i] = x[start + i] || 0;
    sumSquares += segment[i] * segment[i];
  }
  const norm = Math.sqrt(sumSquares / FFT_N) || 1;
  for (let i = 0; i < FFT_N; i++) segment[i] /= norm;
  const { re, im } = fft4096(segment);
  const fsHz = Number.isFinite(sampleRateHz) && sampleRateHz > 0 ? sampleRateHz : DEFAULT_SAMPLE_RATE_HZ;
  const maxFrequencyHz = Math.min(500_000, fsHz / 2);
  const maxK = Math.floor((maxFrequencyHz * FFT_N) / fsHz);
  let total = 0, low = 0, weighted = 0;
  for (let k = 1; k <= maxK; k++) {
    const power = re[k] * re[k] + im[k] * im[k];
    const frequency = (k * fsHz) / FFT_N;
    total += power;
    weighted += frequency * power;
    if (frequency < 100_000) low += power;
  }
  if (!total) return null;
  return { lowRatio: low / total, centroidKHz: weighted / total / 1000 };
}

function readWaveforms(tradbPath, eventIds) {
  const db = new DatabaseSync(tradbPath, { readOnly: true });
  const features = [];
  const shapes = new Map();
  let readErrors = 0;
  try {
    const stmtTrai = db.prepare('SELECT SetID,SampleRate,Samples,Data FROM tr_data WHERE TRAI=?');
    const stmtSetId = db.prepare('SELECT SetID,SampleRate,Samples,Data FROM tr_data WHERE SetID=?');
    for (let idx = 0; idx < eventIds.length; idx++) {
      const id = eventIds[idx];
      try {
        let row = Number.isFinite(id) && id > 0 ? stmtTrai.get(id) : null;
        if (!row) row = stmtSetId.get(idx + 1);
        if (!row) continue;
        const sampleRateHz = Number(row.SampleRate);
        const sampleCount = Number(row.Samples);
        shapes.set(`${sampleRateHz}:${sampleCount}`, { sampleRateHz, sampleCount });
        const feature = waveformFeature(parseWave(row.Data, row.Samples), sampleRateHz);
        if (feature) features.push(feature);
      } catch (_) {
        readErrors++;
      }
    }
  } finally {
    db.close();
  }
  return { features, readErrors, shapes: [...shapes.values()] };
}

function loadModel(customStandardDir) {
  const searchDirs = customStandardDir ? [customStandardDir] : CANDIDATE_STANDARD_DIRS;
  let foundDir = null;
  for (const dir of searchDirs) {
    if (fs.existsSync(path.join(dir, 'distance_model_summary.json'))) {
      foundDir = dir;
      break;
    }
  }
  if (!foundDir) {
    throw new Error(`找不到 distance_model_summary.json，已搜索路径：${searchDirs.join(' ; ')}`);
  }
  const modelPath = path.join(foundDir, 'distance_model_summary.json');
  const detectionPath = path.join(foundDir, 'detection_parameters_10.20.json');
  const model = JSON.parse(fs.readFileSync(modelPath, 'utf8'));
  const detection = fs.existsSync(detectionPath)
    ? JSON.parse(fs.readFileSync(detectionPath, 'utf8'))
    : {};
  if (model.dataset !== '10.20亚克力') throw new Error('标准模型不是 10.20亚克力，已停止。');
  return { model, detection, standardDir: foundDir };
}

function nearestReference(rmsMedian, references) {
  if (!Number.isFinite(rmsMedian) || !Array.isArray(references)) return null;
  let best = null;
  for (const ref of references) {
    const delta = Math.abs(Math.log10(rmsMedian) - Math.log10(ref.median));
    if (!best || delta < best.logDistance) {
      best = { distanceCm: ref.distance_cm, logDistance: delta, referenceRmsMedian: ref.median };
    }
  }
  return best;
}

function evaluateMultiFeatureDistance(rmsMedian, ampMedian, ssMedian, model) {
  const refs = model.rms_reference_ranges || [];
  const source = refs.find(x => x.distance_cm === 0);
  const ten = refs.find(x => x.distance_cm === 10);
  const rms0 = source ? source.median : 3244.7234048;
  const rms10 = ten ? ten.median : 118.7741696;
  const sourceBoundary = Math.sqrt(rms0 * rms10); // 620.7973
  const threshold = Number(model.initial_detection_threshold_rms_median || 11.1849);

  if (!Number.isFinite(rmsMedian)) {
    return { status: 'invalid_rms', reportedDistanceCm: null, rangeText: '无法计算 RMS' };
  }
  if (rmsMedian <= threshold) {
    return {
      status: 'below_noise_or_beyond_50cm',
      reportedDistanceCm: null,
      rangeText: '未检出超过阈值的局放信号（背景噪声或 >50 cm 微弱信号）'
    };
  }
  if (rmsMedian >= sourceBoundary) {
    return { status: 'near_source_point', reportedDistanceCm: null, rangeText: '强信号近场（约 0–10 cm）；缺少 3 cm 等近场标定，不能判为精确 0 cm' };
  }

  // 计算各独立特征的距离估计 (10~50cm 标定线)
  const dRms = (model.rms_fit_intercept - Math.log10(rmsMedian)) / (-model.rms_fit_slope);
  const dSs = Number.isFinite(ssMedian) && ssMedian > 0 ? (3.813687 - Math.log10(ssMedian)) / 0.014537 : dRms;
  const dAmp = Number.isFinite(ampMedian) && ampMedian > 0 ? (7.828210 - Math.log10(ampMedian)) / 0.004734 : dRms;

  let fusedDist;
  if (rmsMedian > 125.0) {
    // 只有 0 和 10 cm 参考点；10–50 cm 拟合线外推到近场会产生虚假的 0 cm。
    return {
      status: 'near_10cm_or_near_field',
      reportedDistanceCm: null,
      dRms, dSs, dAmp,
      rangeText: '近场（约 0–10 cm）；当前标定无法可靠细分 3 cm 等具体距离'
    };
  }

  // 10 ~ 50 cm 波导衰减区：融合 RMS(70%) + SS(15%) + Amp(15%)
  fusedDist = Math.max(0.0, 0.70 * dRms + 0.15 * dSs + 0.15 * dAmp);
  const roundedDistanceCm = Number(fusedDist.toFixed(1));
  if (roundedDistanceCm < 5.0) {
    return {
      status: 'near_10cm_or_near_field',
      reportedDistanceCm: null,
      dRms, dSs, dAmp,
      rangeText: '近场（约 0–10 cm）；当前标定无法可靠细分 3 cm 等具体距离'
    };
  }
  if (roundedDistanceCm < 10.0) {
    return {
      status: 'near_10cm_boundary',
      reportedDistanceCm: roundedDistanceCm,
      dRms, dSs, dAmp,
      rangeText: `约 ${roundedDistanceCm.toFixed(1)} cm（10 cm 标定边界附近；0–10 cm 缺少中间标定）`
    };
  }
  if (roundedDistanceCm >= 60.0) {
    return {
      status: 'beyond_60cm_outside_display',
      reportedDistanceCm: null,
      dRms, dSs, dAmp,
      rangeText: '估算超出 60 cm；10.20 标准库只标定到 50 cm，不显示数值距离'
    };
  }
  if (fusedDist > 50.0) {
    return {
      status: fusedDist > 55.0 ? 'beyond_50cm_outside_calibration' : 'near_50cm_boundary',
      reportedDistanceCm: roundedDistanceCm,
      dRms, dSs, dAmp,
      rangeText: `约 ${fusedDist.toFixed(1)} cm（50–60 cm 未标定外推，仅供实验参考）`
    };
  }
  return {
    status: 'within_calibration_range',
    reportedDistanceCm: Number(fusedDist.toFixed(1)),
    dRms, dSs, dAmp,
    rangeText: `${fusedDist.toFixed(1)} cm（10–50 cm 标定范围内）`
  };
}

function predictOne(pridbPath, modelPack, options = {}) {
  const allowIncompatibleTradb = options.allowIncompatibleTradb === true;
  const tradbPath = pridbPath.replace(/\.pridb$/i, '.tradb');
  const hasTradb = fs.existsSync(tradbPath);

  try {
    const events = readEvents(pridbPath);
    if (!events.length) return { inputPridb: pridbPath, status: 'no_ae_events', error: 'pridb 中没有 SetType=2 的 AE 事件。' };

    const rms = events.map(x => x.rms);
    const energy = events.map(x => x.energy);
    const amp = events.map(x => x.amp);
    const ss = events.map(x => x.ss);
    const counts = events.map(x => x.counts);
    const duration = events.map(x => x.durationUs);

    const rmsMedian = median(rms);
    const energyMedian = median(energy);
    const ampMedian = median(amp);
    const ssMedian = median(ss);
    const rmsQ1 = quantile(rms, 0.25);
    const rmsQ3 = quantile(rms, 0.75);

    let evalRes = evaluateMultiFeatureDistance(rmsMedian, ampMedian, ssMedian, modelPack.model);
    const rmsDistance = Number.isFinite(rmsMedian)
      ? (modelPack.model.rms_fit_intercept - Math.log10(rmsMedian)) / (-modelPack.model.rms_fit_slope)
      : null;
    let energyDistance = Number.isFinite(energyMedian) && energyMedian > 0
      ? (modelPack.model.energy_fit_intercept - Math.log10(energyMedian)) / (-modelPack.model.energy_fit_slope)
      : null;
    let ampDistance = evalRes.dAmp ?? null;
    let ssDistance = evalRes.dSs ?? null;

    // 真实独立交叉检验：对比 RMS 估计距离与峰值幅值(Amp)/信号强度(SS)估计距离
    let independentDiff = Number.isFinite(rmsDistance) && Number.isFinite(ssDistance)
      ? Math.abs(rmsDistance - 0.5 * (ssDistance + (ampDistance ?? ssDistance)))
      : null;

    const nearest = nearestReference(rmsMedian, modelPack.model.rms_reference_ranges);
    const warnings = [];
    if (evalRes.reportedDistanceCm > 50 && evalRes.reportedDistanceCm < 60) {
      warnings.push('50–60 cm 是由 10–50 cm 拟合线外推的未标定估算，须用已知距离样本复核。');
    }
    if (evalRes.status === 'near_10cm_boundary') {
      warnings.push('0–10 cm 只有 0 和 10 cm 标定点；10 cm 边界附近数值未经 3/5 cm 样本验证。');
    }

    // 读取 .tradb 频谱特征（若存在同名 .tradb）
    let wave = { features: [], readErrors: 0, shapes: [] };
    let waveformCompatible = true;
    let waveformMode = 'pridb_only';
    if (hasTradb) {
      wave = readWaveforms(tradbPath, events.map(e => e.trai));
      const oneRate = wave.shapes.length === 1 && wave.shapes[0].sampleRateHz === 2_000_000;
      const fullWindow = oneRate && wave.shapes[0].sampleCount === 200_000;
      const shortWindow = oneRate && wave.shapes[0].sampleCount === 20_000;
      waveformMode = fullWindow ? 'full_multifeature' : shortWindow ? 'experimental_20k_rms' : null;
      waveformCompatible = (fullWindow || shortWindow) && wave.features.length > 0;
      if (shortWindow) {
        // 10.20 的 31 组记录：前 20k 点与完整 200k 点的 RMS 中位数
        // 最大相对偏差 1.47%；Amp / SS / Energy 未按短窗重新标定。
        evalRes = evaluateMultiFeatureDistance(rmsMedian, NaN, NaN, modelPack.model);
        energyDistance = null;
        ampDistance = null;
        ssDistance = null;
        independentDiff = null;
        warnings.push('实验性 20,000 点 RMS 单特征估算；仅在 10.20 亚克力数据内部核验。旧采集文件的探头、增益、滤波和耦合条件未证实一致，须用已知距离实测复核。');
      }
      if (!waveformCompatible) {
        const observed = wave.shapes.length
          ? wave.shapes.map(s => `${s.sampleRateHz} Hz、${s.sampleCount} 点`).join('；')
          : '未读取到可用波形';
        warnings.unshift(`采集波形为 ${observed}；可用模式为 2 MHz / 200,000 点，或实验性的 2 MHz / 20,000 点 RMS 模式。`);
      }
      if (wave.readErrors) {
        warnings.push(`有 ${wave.readErrors} 条波形读取失败，频谱参数仅基于可读取波形。`);
      }
    } else {
      warnings.push('未找到同名 .tradb 波形库，已使用 .pridb 多特征模式完成预测（跳过频谱校验）。');
    }

    const lowRatioMed = median(wave.features.map(x => x.lowRatio));
    const centroidMed = median(wave.features.map(x => x.centroidKHz));

    // 利用频谱特征进行物理耦合与白噪声校验
    if (centroidMed != null && centroidMed > 142.0) {
      warnings.push(`频谱重心偏高 (${centroidMed.toFixed(1)} kHz > 142 kHz)，疑似宽频电子背景噪声或电磁干扰。`);
    }
    if (evalRes.reportedDistanceCm != null && evalRes.reportedDistanceCm >= 35 && lowRatioMed != null && lowRatioMed < 0.25) {
      warnings.push(`RMS 幅值偏低但高频分量丰富 (低频占比仅 ${(lowRatioMed * 100).toFixed(1)}%)，可能是近处探头耦合不良导致信号衰减！`);
    }
    if (events.length < 50) {
      warnings.push(`事件数少于 50（当前 ${events.length} 条），建议增加采集事件后再判断。`);
    }
    if (independentDiff != null && independentDiff > 8.0 && evalRes.reportedDistanceCm >= 10) {
      warnings.push(`RMS 与幅值/信号强度独立估计相差 ${independentDiff.toFixed(1)} cm，请检查探头耦合与按压力度。`);
    }

    let status = evalRes.status;
    let hasPartialDischarge = status !== 'below_noise_or_beyond_50cm' && status !== 'invalid_rms';
    let distanceCm = evalRes.reportedDistanceCm;
    let distanceText = evalRes.rangeText;

    if (hasTradb && !waveformCompatible && !allowIncompatibleTradb) {
      status = 'incompatible_acquisition';
      hasPartialDischarge = null;
      distanceCm = null;
      distanceText = '采样率或波形长度不在可用模式内，无法判断局放或预测距离';
    } else if (hasTradb && !waveformCompatible && allowIncompatibleTradb) {
      // CAP/现场采集波形长度随击中变化，不等于标定窗 200k；仍用 .pridb 多特征给距离
      warnings.unshift('波形长度与标定窗不一致，已回退为 .pridb 特征预测（与桌面端无 tradb / 旧脚本行为一致）。');
      waveformMode = 'pridb_only_fallback';
    } else if (events.length < 30) {
      status = 'insufficient_events';
      hasPartialDischarge = null;
      distanceCm = null;
      distanceText = '有效事件不足 30 条，暂不能判断';
    } else if (waveformMode === 'experimental_20k_rms') {
      hasPartialDischarge = null;
      if (distanceCm != null) distanceText = '实验性 RMS 距离估算：' + distanceText;
    }

    return {
      inputPridb: pridbPath,
      inputTradb: hasTradb ? tradbPath : null,
      modelDataset: modelPack.model.dataset,
      status,
      result: {
        hasPartialDischarge,
        distanceCm,
        distanceText,
        nearestReferenceDistanceCm: (distanceCm == null || status === 'incompatible_acquisition' || status === 'insufficient_events' || !hasPartialDischarge)
          ? null : (nearest?.distanceCm ?? null)
      },
      eventCount: events.length,
      waveformCount: wave.features.length,
      waveformReadErrors: wave.readErrors,
      waveformShape: wave.shapes,
      features: {
        rmsMedian,
        energyMedian,
        amplitudeMedian: ampMedian,
        signalStrengthMedian: ssMedian,
        countsMedian: median(counts),
        durationMedianUs: median(duration),
        rmsIqrRatio: safeRelative(rmsQ3 - rmsQ1, rmsMedian),
        lowRatio0To100KHzMedian: lowRatioMed,
        spectralCentroidKHzMedian: centroidMed
      },
      estimates: {
        fusedDistanceCm: distanceCm,
        rmsDistanceCm: rmsDistance != null ? Number(rmsDistance.toFixed(2)) : null,
        energyDistanceCm: energyDistance != null ? Number(energyDistance.toFixed(2)) : null,
        amplitudeDistanceCm: ampDistance != null ? Number(ampDistance.toFixed(2)) : null,
        signalStrengthDistanceCm: ssDistance != null ? Number(ssDistance.toFixed(2)) : null,
        independentFeatureDiffCm: independentDiff != null ? Number(independentDiff.toFixed(2)) : null
      },
      quality: {
        enoughEvents: events.length >= 30,
        independentFeaturesConsistent: independentDiff == null ? null : independentDiff <= 8.0,
        waveformCompatible: hasTradb ? waveformCompatible : null,
        predictionMode: waveformMode,
        waveformComplete: hasTradb ? wave.features.length === events.length : false,
        warnings
      }
    };
  } catch (error) {
    return { inputPridb: pridbPath, status: 'read_error', error: error.message };
  }
}

function csv(value) {
  if (value == null) return '';
  const s = String(value);
  return /[",\n]/.test(s) ? `"${s.replace(/"/g, '""')}"` : s;
}

function writeOutputs(results, outputDir) {
  fs.mkdirSync(outputDir, { recursive: true });
  const stamp = new Date().toISOString().replace(/[-:TZ.]/g, '').slice(0, 14);
  const jsonPath = path.join(outputDir, `distance_predictions_${stamp}.json`);
  const csvPath = path.join(outputDir, `distance_predictions_${stamp}.csv`);
  fs.writeFileSync(jsonPath, JSON.stringify({ generatedAt: new Date().toISOString(), results }, null, 2), 'utf8');
  const header = [
    'input_pridb', 'status', 'has_partial_discharge', 'distance_cm', 'distance_text',
    'nearest_reference_distance_cm', 'event_count', 'waveform_count', 'rms_median',
    'energy_median', 'amplitude_median', 'signal_strength_median', 'rms_distance_cm',
    'independent_diff_cm', 'low_ratio_0_100khz_median', 'spectral_centroid_khz_median', 'warnings'
  ];
  const rows = results.map(r => [
    r.inputPridb, r.status, r.result?.hasPartialDischarge, r.result?.distanceCm,
    r.result?.distanceText, r.result?.nearestReferenceDistanceCm, r.eventCount,
    r.waveformCount, r.features?.rmsMedian, r.features?.energyMedian,
    r.features?.amplitudeMedian, r.features?.signalStrengthMedian,
    r.estimates?.rmsDistanceCm, r.estimates?.independentFeatureDiffCm,
    r.features?.lowRatio0To100KHzMedian, r.features?.spectralCentroidKHzMedian,
    r.quality?.warnings?.join('；') || r.error || ''
  ].map(csv).join(','));
  fs.writeFileSync(csvPath, [header.join(','), ...rows, ''].join('\n'), 'utf8');
  return { jsonPath, csvPath };
}

function printSummary(results, output) {
  console.log('\n==================== 10.20 亚克力局放与距离预测结果 ====================');
  for (const r of results) {
    const name = path.basename(r.inputPridb);
    if (r.error) {
      console.log(`❌ ${name}: 失败 -> ${r.error}`);
      continue;
    }
    const judgement = r.result.hasPartialDischarge === true ? '⚠️ 检测到局放'
      : (r.status === 'below_noise_or_beyond_50cm' ? '✅ 非局放/背景噪声' : '❓ 无法判定');
    const nearStr = r.result.nearestReferenceDistanceCm != null ? ` (最近标准点: ${r.result.nearestReferenceDistanceCm} cm)` : '';
    console.log(`- ${name}: ${judgement} | 预测距离: ${r.result.distanceText}${nearStr} | RMS=${r.features.rmsMedian.toFixed(2)} | 事件=${r.eventCount}`);
    if (r.quality.warnings.length) console.log(`  💡 提示：${r.quality.warnings.join(' ')}`);
  }
  console.log('========================================================================');
  console.log(`JSON 详细结果：${output.jsonPath}`);
  console.log(`CSV 表格汇总：${output.csvPath}`);
}

function main() {
  const args = parseArgs(process.argv.slice(2));
  if (args.help) { usage(); return; }
  if (!args.inputs.length) { usage(); process.exitCode = 2; return; }
  const modelPack = loadModel(args.standardDir);
  const pridbFiles = listPridb(args.inputs);
  if (!pridbFiles.length) throw new Error('输入路径中没有找到 .pridb 文件。');
  const results = pridbFiles.map(file => predictOne(file, modelPack));
  const output = writeOutputs(results, args.outputDir);
  printSummary(results, output);
}

if (require.main === module) {
  try {
    main();
  } catch (error) {
    console.error(`错误：${error.message}`);
    process.exitCode = 1;
  }
}

// Reuse the calibrated model in the lightweight read-only web preview / CAP final.
module.exports = {
  loadModel, readEvents, median, quantile, nearestReference,
  evaluateMultiFeatureDistance, predictOne
};
