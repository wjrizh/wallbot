// Use only the 10.20 calibration model. Input is a new .pridb file with a
// same-stem .tradb file beside it. The source files are read but never changed.
const fs = require('fs');
const path = require('path');
const { DatabaseSync } = require('node:sqlite');

const here = __dirname;
const pridbPath = process.argv[2];
if (!pridbPath) { console.error('用法: node predict_distance_10.20.js <新数据.pridb>'); process.exit(2); }
const absPridb = path.resolve(pridbPath);
const absTradb = absPridb.replace(/\.pridb$/i,'.tradb');
if (!fs.existsSync(absPridb)) throw new Error(`找不到 pridb: ${absPridb}`);
if (!fs.existsSync(absTradb)) throw new Error(`找不到配套 tradb: ${absTradb}`);

const model = JSON.parse(fs.readFileSync(path.join(here,'distance_model_summary.json'),'utf8'));
if (model.dataset !== '10.20亚克力') throw new Error('模型不是 10.20亚克力 标定模型，已停止。');
const aeDb = new DatabaseSync(absPridb);
const events = aeDb.prepare(`SELECT RMS, Eny, Counts, Dur FROM view_ae_data WHERE SetType=2 ORDER BY SetID`).all();
aeDb.close();
if (!events.length) throw new Error('pridb 中没有 SetType=2 的 AE 事件。');
const trDb = new DatabaseSync(absTradb);
// Read individual SetID rows so a truncated/corrupted tail does not prevent
// using the valid waveforms that precede it. The original file is untouched.
const waves=[]; let waveformReadErrors=0;
const waveById=trDb.prepare(`SELECT SetID,SampleRate,Samples,Data FROM tr_data WHERE SetID=?`);
for(let id=1;id<=events.length;id++){try{const w=waveById.get(id);if(w)waves.push(w);}catch(e){waveformReadErrors++;}}
trDb.close();

function quantile(a,p){const x=a.filter(Number.isFinite).slice().sort((u,v)=>u-v);if(!x.length)return null;const q=(x.length-1)*p,l=Math.floor(q),h=Math.ceil(q);return l===h?x[l]:x[l]+(x[h]-x[l])*(q-l);}
function median(a){return quantile(a,.5);}
function parseWave(data,samples){const b=Buffer.from(data),n=Math.min(samples,b.length/2|0),x=new Float64Array(n);let m=0;for(let i=0;i<n;i++){x[i]=b.readInt16LE(i*2);m+=x[i];}m/=n;for(let i=0;i<n;i++)x[i]-=m;return x;}
const FFT_N=4096,FS=2_000_000,SMOOTH=101;
function envelope(x){const e=new Float64Array(x.length);let s=0;for(let i=0;i<x.length;i++){s+=Math.abs(x[i]);if(i>=SMOOTH)s-=Math.abs(x[i-SMOOTH]);e[i]=s/Math.min(i+1,SMOOTH);}return e;}
function fft(x){const n=x.length,re=new Float64Array(n),im=new Float64Array(n);for(let i=0;i<n;i++){const w=.5*(1-Math.cos(2*Math.PI*i/(n-1)));re[i]=x[i]*w;}for(let j=1,k=0;j<n;j++){let bit=n>>1;for(;k&bit;bit>>=1)k^=bit;k^=bit;if(j<k){let t=re[j];re[j]=re[k];re[k]=t;}}for(let len=2;len<=n;len<<=1){const a=-2*Math.PI/len,wr=Math.cos(a),wi=Math.sin(a);for(let i=0;i<n;i+=len){let cr=1,ci=0;for(let j=0;j<len/2;j++){const ur=re[i+j],ui=im[i+j],vr=re[i+j+len/2]*cr-im[i+j+len/2]*ci,vi=re[i+j+len/2]*ci+im[i+j+len/2]*cr;re[i+j]=ur+vr;im[i+j]=ui+vi;re[i+j+len/2]=ur-vr;im[i+j+len/2]=ui-vi;const nr=cr*wr-ci*wi;ci=cr*wi+ci*wr;cr=nr;}}}return {re,im};}
function waveFeature(x){const e=envelope(x);let center=0;for(let i=1;i<e.length;i++)if(e[i]>e[center])center=i;const seg=new Float64Array(FFT_N),start=Math.max(0,Math.min(x.length-FFT_N,center-(FFT_N>>1)));let ss=0;for(let i=0;i<FFT_N;i++){seg[i]=x[start+i]||0;ss+=seg[i]*seg[i];}const norm=Math.sqrt(ss/FFT_N)||1;for(let i=0;i<FFT_N;i++)seg[i]/=norm;const {re,im}=fft(seg);let total=0,low=0,weighted=0;const maxK=Math.floor(500000*FFT_N/FS);for(let k=1;k<=maxK;k++){const p=re[k]*re[k]+im[k]*im[k],f=k*FS/FFT_N;total+=p;weighted+=f*p;if(f<100000)low+=p;}return {low_ratio:low/total,centroid_khz:weighted/total/1000};}
const rms=events.map(e=>Number(e.RMS)),energy=events.map(e=>Number(e.Eny)),counts=events.map(e=>Number(e.Counts));
const rmsMedian=median(rms),energyMedian=median(energy);
const waveFeatures=waves.map(w=>waveFeature(parseWave(w.Data,w.Samples)));
const lowMedian=median(waveFeatures.map(x=>x.low_ratio)),centroidMedian=median(waveFeatures.map(x=>x.centroid_khz));
const dR=(model.rms_fit_intercept-Math.log10(rmsMedian))/(-model.rms_fit_slope);
const dE=(model.energy_fit_intercept-Math.log10(energyMedian))/(-model.energy_fit_slope);
const references=model.rms_reference_ranges;
const source10Boundary=Math.sqrt(references.find(x=>x.distance_cm===0).median*references.find(x=>x.distance_cm===10).median);
let status='within_calibration_range',estimate=dR;
if (rmsMedian <= model.initial_detection_threshold_rms_median) { status='below_noise_or_beyond_50cm'; estimate=null; }
else if (rmsMedian >= source10Boundary) { status='near_source_point'; estimate=0; }
else { estimate=Math.max(10,Math.min(50,dR)); if(dR<10||dR>50) status='outside_calibration_range'; }
let nearest=null;for(const ref of references){const delta=Math.abs(Math.log10(rmsMedian)-Math.log10(ref.median));if(!nearest||delta<nearest.delta)nearest={distance_cm:ref.distance_cm,delta};}
const result={model_dataset:model.dataset,input_pridb:absPridb,event_count:events.length,waveform_count:waves.length,waveform_read_errors:waveformReadErrors,features:{rms_median:rmsMedian,energy_median:energyMedian,counts_median:median(counts),low_ratio_0_100khz_median:lowMedian,spectral_centroid_khz_median:centroidMedian,rms_iqr_ratio:(quantile(rms,.75)-quantile(rms,.25))/rmsMedian},estimates:{rms_distance_cm:dR,energy_distance_cm:dE,reported_distance_cm:estimate,nearest_reference_distance_cm:nearest?.distance_cm},status,quality_checks:{energy_rms_difference_cm:Math.abs(dR-dE),enough_events:events.length>=50,energy_rms_consistent:Math.abs(dR-dE)<=5,waveform_complete:waves.length===events.length,warning:waveformReadErrors?'tradb尾部有无法读取的波形，频谱辅助参数仅基于可读取波形；RMS和能量距离仍可用。':(Math.abs(dR-dE)>5?'能量与RMS估计差异超过5 cm，先检查探头耦合和采集条件。':null)}};
console.log(JSON.stringify(result,null,2));
