#ifndef INDEX_HTML_H
#define INDEX_HTML_H

#include <pgmspace.h>

static const char INDEX_HTML[] PROGMEM = R"HTMLDOC(<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>VL53L5CX 8×8 ToF 测距可视化</title>
<style>
  :root{
    --ink:#1a1a1a; --sub:#6b6b6b; --line:#d8d8d8; --bg:#ffffff;
    --accent:#2c5f8a; --warn:#c0392b;
  }
  *{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
  body{
    margin:0;background:var(--bg);color:var(--ink);
    font-family:"Helvetica Neue",Helvetica,Arial,"PingFang SC","Microsoft YaHei",sans-serif;
    font-size:14px;line-height:1.5;
  }
  .wrap{max-width:960px;margin:0 auto;padding:28px 22px 60px}
  header{border-bottom:2px solid var(--ink);padding-bottom:14px;margin-bottom:26px}
  h1{
    font-family:Georgia,"Times New Roman","Songti SC",serif;
    font-size:21px;font-weight:600;margin:0 0 6px;letter-spacing:.2px;
  }
  .sub{color:var(--sub);font-size:12.5px}
  .fig{margin:0 0 34px}
  .fig-cap{
    font-family:Georgia,"Times New Roman",serif;
    font-size:12.5px;color:var(--sub);margin-top:10px;
    padding-top:8px;border-top:1px solid var(--line);
  }
  .fig-cap b{color:var(--ink);font-weight:600}
  .fig-row{display:flex;gap:26px;align-items:flex-start;flex-wrap:wrap}
  .heat-wrap{position:relative}
  canvas{display:block}
  #heat{image-rendering:pixelated;border:1px solid var(--line)}
  .cbar{display:flex;flex-direction:column;align-items:center;gap:4px;font-size:11px;color:var(--sub)}
  #cbarCanvas{border:1px solid var(--line)}
  .cbar-lab{writing-mode:vertical-rl;font-size:11px;color:var(--sub)}
  .cbar-col{display:flex;flex-direction:column;align-items:center;gap:4px}
  .cbar-num{font-size:10px;color:var(--sub);font-variant-numeric:tabular-nums}
  .readout{display:flex;gap:22px;flex-wrap:wrap;margin:14px 0 0}
  .metric{border-left:2px solid var(--accent);padding:2px 0 2px 12px;min-width:120px}
  .metric .k{font-size:11px;color:var(--sub);letter-spacing:.5px;text-transform:uppercase}
  .metric .v{font-family:Georgia,serif;font-size:26px;font-weight:600;line-height:1.15}
  .metric .u{font-size:12px;color:var(--sub);font-weight:400;margin-left:3px}
  .metric.warn{border-color:var(--warn)}
  .metric.warn .v{color:var(--warn)}
  #trend{border:1px solid var(--line);width:100%;height:170px}
  table{border-collapse:collapse;font-size:12px;font-variant-numeric:tabular-nums}
  td,th{border:1px solid var(--line);padding:3px 7px;text-align:center;min-width:34px}
  th{color:var(--sub);font-weight:500;background:#fafafa}
  td.hit{background:#eef4fa;font-weight:600}
  .rowhead{color:var(--sub);background:#fafafa;font-weight:500}
  .toolbar{display:flex;gap:10px;margin-top:14px}
  button{
    font:inherit;font-size:12.5px;padding:7px 16px;border:1px solid var(--ink);
    background:#fff;color:var(--ink);border-radius:3px;cursor:pointer;transition:.15s;
  }
  button:hover{background:var(--ink);color:#fff}
  button:disabled{opacity:.4;cursor:default}
  .status{display:inline-flex;align-items:center;gap:6px;font-size:12px;color:var(--sub);margin-left:auto}
  .dot{width:8px;height:8px;border-radius:50%;background:#bbb}
  .dot.on{background:#27ae60}
  .dot.off{background:var(--warn)}
  footer{margin-top:40px;padding-top:14px;border-top:1px solid var(--line);
    font-size:11.5px;color:var(--sub);font-family:Georgia,serif}
</style>
</head>
<body>
<div class="wrap">

  <header>
    <h1>VL53L5CX 8×8 飞行时间（ToF）测距可视化</h1>
    <div class="sub">ESP32-S3 · I²C @400 kHz · TCA9548A CH0 · 15 Hz 采样 · 64 区深度</div>
  </header>

  <section class="fig">
    <div class="fig-row">
      <div class="heat-wrap"><canvas id="heat" width="480" height="480"></canvas></div>
      <div class="cbar">
        <div class="cbar-lab">距离 (mm)</div>
        <div class="cbar-col">
          <div class="cbar-num">300</div>
          <canvas id="cbarCanvas" width="26" height="440"></canvas>
          <div class="cbar-num">0</div>
        </div>
      </div>
    </div>
    <div class="readout">
      <div class="metric" id="mFront">
        <div class="k">前方最近距离</div>
        <div class="v"><span id="vFront">—</span><span class="u">mm</span></div>
      </div>
      <div class="metric">
        <div class="k">有效区</div>
        <div class="v"><span id="vValid">0</span><span class="u">/64</span></div>
      </div>
      <div class="metric">
        <div class="k">帧率</div>
        <div class="v"><span id="vFps">0</span><span class="u">Hz</span></div>
      </div>
      <div class="metric">
        <div class="k">累计帧</div>
        <div class="v" style="font-size:18px"><span id="vFrames">0</span></div>
      </div>
      <div class="status"><span class="dot off" id="dot"></span><span id="stext">连接中…</span></div>
    </div>
    <div class="fig-cap"><b>图 1.</b> 8×8 ToF 深度热力图。颜色编码目标距离，越暗表示越近。白色描边为前方扇形感兴趣区（ROI，行 4–7，列 1–6）。画面已按传感器实际朝向校正：物理前方显示于上方。</div>
  </section>

  <section class="fig">
    <canvas id="trend" width="900" height="170"></canvas>
    <div class="fig-cap"><b>图 2.</b> 前方 ROI 最小距离时间序列（最近 120 个采样点，约 60 s）。</div>
  </section>

  <section class="fig">
    <div class="fig-row" style="align-items:flex-start">
      <div id="tableBox"></div>
      <div style="flex:1;min-width:200px">
        <div class="sub" style="margin-bottom:8px">单元格为有效距离 (mm)，空白表示无有效回波。</div>
        <div class="toolbar">
          <button id="btnCsv">导出 CSV</button>
          <button id="btnPng">导出热力图 PNG</button>
        </div>
      </div>
    </div>
    <div class="fig-cap"><b>表 1.</b> 8×8 距离矩阵实时快照（mm）。</div>
  </section>

  <footer>Wallbot · ToF Ranging Testbed — 独立测试固件，数据供论文引用</footer>
</div>

<script>
(function(){
  "use strict";
  var N=8, CELL=60, DMAX=300;
  var heat=document.getElementById("heat"), hctx=heat.getContext("2d");
  var cbar=document.getElementById("cbarCanvas"), cctx=cbar.getContext("2d");
  var trend=document.getElementById("trend"), tctx=trend.getContext("2d");
  var CW=heat.width, CH=heat.height;
  hctx.imageSmoothingEnabled=false;

  /* 距离色标: 蓝(远) → 青 → 绿 → 黄 → 红(近) */
  var STOPS=[
    [0.00,[ 40, 70,160]], [0.25,[ 40,180,180]], [0.50,[ 80,200, 90]],
    [0.75,[250,200, 40]], [1.00,[220, 40, 30]]
  ];
  function viridis(t){
    t=Math.max(0,Math.min(1,t));
    for(var i=0;i<STOPS.length-1;i++){
      var a=STOPS[i], b=STOPS[i+1];
      if(t>=a[0] && t<=b[0]){
        var f=(t-a[0])/(b[0]-a[0]);
        return "rgb("+
          Math.round(a[1][0]+(b[1][0]-a[1][0])*f)+","+
          Math.round(a[1][1]+(b[1][1]-a[1][1])*f)+","+
          Math.round(a[1][2]+(b[1][2]-a[1][2])*f)+")";
      }
    }
    return "rgb(220,40,30)";
  }

  /* 距离→颜色: DRED 及以内纯红, DRED..DMAX 线性红→蓝 */
  var DRED=70;
  function colorFor(dval){
    if(dval<=0) return null;
    var t;
    if(dval<=DRED) t=1;
    else t=1-(dval-DRED)/(DMAX-DRED);
    return viridis(t);
  }

  /* colorbar: 顶=远(蓝), 底=近(红), 底部 DRED 段纯红 */
  (function(){
    var h=cbar.height, w=cbar.width;
    for(var y=0;y<h;y++){
      var dval=(1-y/(h-1))*DMAX;
      cctx.fillStyle=colorFor(dval)||viridis(1);
      cctx.fillRect(0,y,w,1);
    }
  })();

  var hist=[], lastData=null;

  function drawHeat(d,s){
    var cw=CW/N, ch=CH/N;
    hctx.clearRect(0,0,CW,CH);
    for(var r=0;r<N;r++){
      for(var c=0;c<N;c++){
        var i=r*N+c, dval=d[i];
        var x=c*cw, y=(7-r)*ch;   /* 垂直翻转: 物理前方(row7)显示在画布顶部 */
        if(dval>0){
          hctx.fillStyle=colorFor(dval);
        }else{
          hctx.fillStyle="#f2f2f2";
        }
        hctx.fillRect(x,y,cw,ch);
      }
    }
    /* grid */
    hctx.strokeStyle="rgba(255,255,255,.35)";
    hctx.lineWidth=1;
    for(var k=0;k<=N;k++){
      hctx.beginPath();hctx.moveTo(k*cw,0);hctx.lineTo(k*cw,CH);hctx.stroke();
      hctx.beginPath();hctx.moveTo(0,k*ch);hctx.lineTo(CW,k*ch);hctx.stroke();
    }
    /* ROI 描边: 物理 row4-7 (翻转后位于画布顶部), col1-6 */
    hctx.strokeStyle="#ffffff";
    hctx.lineWidth=3;
    hctx.strokeRect(1*cw, 0*ch, (6-1+1)*cw, 4*ch);
    hctx.strokeStyle="rgba(0,0,0,.55)";
    hctx.lineWidth=1;
    hctx.strokeRect(1*cw, 0*ch, (6-1+1)*cw, 4*ch);
  }

  function drawTrend(){
    var w=trend.width, h=trend.height, pad=30;
    tctx.clearRect(0,0,w,h);
    tctx.fillStyle="#fff"; tctx.fillRect(0,0,w,h);
    /* axes */
    tctx.strokeStyle="#d8d8d8"; tctx.lineWidth=1;
    tctx.beginPath();tctx.moveTo(pad,6);tctx.lineTo(pad,h-pad);tctx.lineTo(w-6,h-pad);tctx.stroke();
    tctx.fillStyle="#6b6b6b"; tctx.font="11px Helvetica";
    tctx.fillText("mm",4,14);
    tctx.fillText("t →",w-22,h-8);
    var ymax=DMAX;
    tctx.fillText(String(ymax),4,pad);
    tctx.fillText("0",4,h-pad+4);
    if(hist.length<2) return;
    var n=hist.length;
    var xs=function(i){ return pad + (w-pad-6)*(i/(n-1)); };
    var ys=function(v){ return (h-pad) - (h-pad-6)*(Math.min(v,ymax)/ymax); };
    tctx.strokeStyle="#2c5f8a"; tctx.lineWidth=1.8;
    tctx.beginPath();
    var started=false;
    for(var i=0;i<n;i++){
      if(hist[i]<=0) continue;
      var x=xs(i), y=ys(hist[i]);
      if(!started){tctx.moveTo(x,y);started=true;} else tctx.lineTo(x,y);
    }
    tctx.stroke();
  }

  function buildTable(){
    var box=document.getElementById("tableBox");
    var html="<table><tr><th></th>";
    for(var c=0;c<N;c++) html+="<th>C"+c+"</th>";
    html+="</tr>";
    /* 行序翻转: R7 在顶部 (物理前方), 与热力图一致 */
    for(var r=N-1;r>=0;r--){
      html+="<tr><td class='rowhead'>R"+r+"</td>";
      for(var c2=0;c2<N;c2++) html+="<td id='c"+r+"_"+c2+"'></td>";
      html+="</tr>";
    }
    html+="</table>";
    box.innerHTML=html;
  }
  buildTable();

  function updateTable(d){
    for(var r=0;r<N;r++)for(var c=0;c<N;c++){
      var td=document.getElementById("c"+r+"_"+c);
      var v=d[r*N+c];
      var inRoi=(r>=4&&r<=7&&c>=1&&c<=6);
      td.textContent = v>0 ? v : "";
      td.className = (v>0&&inRoi) ? "hit" : "";
    }
  }

  var dot=document.getElementById("dot"), stext=document.getElementById("stext");
  function setStatus(ok){
    dot.className="dot "+(ok?"on":"off");
    stext.textContent=ok?"在线":"连接中断";
  }

  function poll(){
    fetch("/tof/matrix",{cache:"no-store"})
      .then(function(r){return r.json();})
      .then(function(j){
        lastData=j;
        setStatus(true);
        drawHeat(j.d,j.s);
        updateTable(j.d);
        var valid=0; for(var i=0;i<64;i++) if(j.d[i]>0) valid++;
        document.getElementById("vValid").textContent=valid;
        document.getElementById("vFps").textContent=j.fps.toFixed(1);
        document.getElementById("vFrames").textContent=j.frames;
        var mf=document.getElementById("mFront");
        var vf=document.getElementById("vFront");
        if(j.front>0){ vf.textContent=j.front; mf.classList.remove("warn"); }
        else { vf.textContent="—"; mf.classList.add("warn"); }
        hist.push(j.front);
        if(hist.length>120) hist.shift();
        drawTrend();
      })
      .catch(function(){ setStatus(false); });
  }

  /* CSV 导出: 当前 8x8 + 状态 */
  document.getElementById("btnCsv").addEventListener("click",function(){
    if(!lastData) return;
    var lines=["row,col,distance_mm,target_status"];
    for(var r=0;r<N;r++)for(var c=0;c<N;c++){
      var i=r*N+c;
      lines.push(r+","+c+","+lastData.d[i]+","+lastData.s[i]);
    }
    download(lines.join("\n"),"tof_8x8.csv","text/csv");
  });

  /* 热力图 PNG 导出: 合成完整 Figure (热力图+行列号+ROI框+colorbar) */
  document.getElementById("btnPng").addEventListener("click",function(){
    if(!lastData) return;
    downloadDataURL(buildFigure(lastData.d),"tof_heatmap.png");
  });

  function buildFigure(d){
    var G=60;                 /* 每格像素, 与页面一致 */
    var grid=G*N;             /* 480 */
    var mL=54, mT=24, mB=24, mR=140;   /* 左/上/下/右留白, 右侧留给 colorbar */
    var W=mL+grid+mR, H=mT+grid+mB;
    var cv=document.createElement("canvas");
    cv.width=W; cv.height=H;
    var x=cv.getContext("2d");

    /* 白底 */
    x.fillStyle="#fff"; x.fillRect(0,0,W,H);

    /* 热力图格子 */
    for(var r=0;r<N;r++){
      for(var c=0;c<N;c++){
        var i=r*N+c, dval=d[i];
        var px=mL+c*G, py=mT+(7-r)*G;   /* 垂直翻转: 物理前方(row7)在顶部 */
        x.fillStyle = dval>0 ? colorFor(dval) : "#f2f2f2";
        x.fillRect(px,py,G,G);
      }
    }

    /* 网格线 */
    x.strokeStyle="rgba(255,255,255,.35)"; x.lineWidth=1;
    for(var k=0;k<=N;k++){
      x.beginPath(); x.moveTo(mL+k*G, mT); x.lineTo(mL+k*G, mT+grid); x.stroke();
      x.beginPath(); x.moveTo(mL, mT+k*G); x.lineTo(mL+grid, mT+k*G); x.stroke();
    }

    /* ROI 框: 物理 row4-7, col1-6 (翻转后位于顶部) */
    x.strokeStyle="#ffffff"; x.lineWidth=3;
    x.strokeRect(mL+1*G, mT+0*G, 5*G, 4*G);
    x.strokeStyle="rgba(0,0,0,.55)"; x.lineWidth=1;
    x.strokeRect(mL+1*G, mT+0*G, 5*G, 4*G);

    /* 外框 */
    x.strokeStyle="#333"; x.lineWidth=1;
    x.strokeRect(mL, mT, grid, grid);

    /* 行号 (画布自上而下 = R7..R0) */
    x.fillStyle="#333"; x.font="13px sans-serif";
    x.textAlign="right"; x.textBaseline="middle";
    for(var rr=0;rr<N;rr++){
      x.fillText("R"+rr, mL-8, mT+(7-rr)*G+G/2);
    }
    /* 列号 */
    x.textAlign="center"; x.textBaseline="top";
    for(var cc=0;cc<N;cc++){
      x.fillText("C"+cc, mL+cc*G+G/2, mT+grid+6);
    }

    /* colorbar (右侧) */
    var cbX=mL+grid+52, cbY=mT, cbW=26, cbH=grid;
    for(var yy=0;yy<cbH;yy++){
      var dval=(1-yy/(cbH-1))*DMAX; x.fillStyle=colorFor(dval)||viridis(1);
      x.fillRect(cbX, cbY+yy, cbW, 1);
    }
    x.strokeStyle="#333"; x.lineWidth=1;
    x.strokeRect(cbX, cbY, cbW, cbH);

    /* colorbar 刻度: 300(顶) .. 0(底), 每 50 一格 */
    x.fillStyle="#333"; x.font="12px sans-serif";
    x.textAlign="left"; x.textBaseline="middle";
    for(var v=0;v<=DMAX;v+=50){
      var ty=cbY+cbH-(v/DMAX)*cbH;
      x.beginPath(); x.moveTo(cbX+cbW, ty); x.lineTo(cbX+cbW+5, ty); x.stroke();
      x.fillText(v, cbX+cbW+8, ty);
    }

    /* colorbar 标题 (旋转) */
    x.save();
    x.translate(cbX+cbW+62, cbY+cbH/2);
    x.rotate(Math.PI/2);
    x.textAlign="center"; x.textBaseline="middle";
    x.font="13px sans-serif";
    x.fillText("距离 (mm)", 0, 0);
    x.restore();

    return cv.toDataURL("image/png");
  }

  function download(text,name,mime){
    var blob=new Blob([text],{type:mime});
    var url=URL.createObjectURL(blob);
    downloadDataURL(url,name);
    setTimeout(function(){URL.revokeObjectURL(url);},1000);
  }
  function downloadDataURL(url,name){
    var a=document.createElement("a");
    a.href=url; a.download=name; document.body.appendChild(a); a.click();
    document.body.removeChild(a);
  }

  buildTable();
  poll();
  setInterval(poll,500);
})();
</script>
</body>
</html>
)HTMLDOC";

#endif