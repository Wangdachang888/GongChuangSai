'use strict';
// Offline ULog43 analysis. Keep SI units internally; display angular rates in deg/s.
const $ = id => document.getElementById(id);
const WHEELS = ['fl', 'fr', 'rl', 'rr'];
const COLORS = {center:'#67baff', fl:'#ff7c8d', fr:'#ffc86a', rl:'#c4a0ff', rr:'#50dfae'};
const NAMES = {fl:'FL 左前轮', fr:'FR 右前轮', rl:'RL 左后轮', rr:'RR 右后轮'};
const DEG = 180 / Math.PI;
const DEFAULT_CHANNELS = {
  mode:0, time:5, phase:1, x:32, y:33, yaw:16, imu:17, xt:21, yt:22, yawt:23,
  vx:null, vy:null, vxTarget:12, vyTarget:13, wt:14, wa:15, speedTarget:19, lateral:39, yawError:42,
  // ULog43 stores FL target at ch27 and the remaining wheel targets at ch35..37.
  // Actual wheel speeds remain ch28..31 (FL/FR/RL/RR respectively).
  flt:27, fla:28, frt:35, fra:29, rlt:36, rla:30, rrt:37, rra:31,
  flo:null, fro:null, rlo:null, rro:null, fle:null, fre:null, rle:null, rre:null
};
const CHANNEL_LABELS = {time:'时间戳 / ms',phase:'阶段',x:'位置 X / m',y:'位置 Y / m',yaw:'底盘航向 / °',imu:'IMU 航向 / °',xt:'目标 X / m',yt:'目标 Y / m',yawt:'目标航向 / °',vx:'直接 vx / m/s',vy:'直接 vy / m/s',wt:'目标 ω / rad/s',wa:'实测 ω / rad/s',speedTarget:'目标 |v| / 0.073632',lateral:'目标横向偏差 / m',yawError:'航向误差 / °'};
for (const w of WHEELS) for (const [suffix,label] of Object.entries({t:'目标',a:'实测',o:'输出',e:'累计编码器'})) CHANNEL_LABELS[w+suffix] = `${NAMES[w]} ${label}`;
const RAW_LABELS = {0:'底盘模式（0 速度 / 1 位置）',1:'阶段（0 等待 / 1 直行 / 2 转弯 / 4 完成）',2:'方形任务状态',3:'方形任务步骤',4:'到达目标标记',5:'时间戳 ms',8:'阶段副本',14:'目标角速度 rad/s',15:'实测角速度 rad/s（旧转换日志可能未填充）',16:'底盘航向 °',17:'IMU 航向 °',18:'目标角速度 °/s',19:'平移目标速度大小 / 0.073632（无方向）',20:'四轮绝对轮速均值 / 1750 / 0.073632（不是 vx）',21:'目标 X m',22:'目标 Y m',23:'目标航向 °',24:'世界坐标 X 误差 m',25:'世界坐标 Y 误差 m',26:'航向误差 °',27:'左前轮目标 pulse/s',28:'左前轮实测 pulse/s',29:'右前轮实测 pulse/s',30:'左后轮实测 pulse/s',31:'右后轮实测 pulse/s',32:'位置 X m',33:'位置 Y m',34:'累计路程 m',35:'右前轮目标 pulse/s',36:'左后轮目标 pulse/s',37:'右后轮目标 pulse/s',38:'四轮控制输出绝对值最大值',39:'目标位置在车体系的横向偏差 m',41:'目标角速度副本 rad/s',42:'航向误差副本 °'};
var config = {radius:.04,ppr:440,wheelbase:.186,track:.22,dt:.05,omegaMode:'wheels',channels:{...DEFAULT_CHANNELS},metadata:{firmware:'',test:'',content:'',environment:'',battery:'',notes:''}};
var S = {rows:[],times:[],samples:[],points:[],cur:0,tab:'transport',medianDt:.05,gaps:0,name:'',
  alignment:{x:2.25,y:.15,angle:180},visible:{center:true,fl:true,fr:true,rl:true,rr:true},
  map:{x:1.2,y:1.2,spanX:2.6,spanY:2.6,zoom:1,scale:1,w:0,h:0},
  wave:{start:0,end:1,gain:1,mode:'body',w:0,h:0},
  pid:[],pidBurst:null,segments:[],selectedSegment:null,quality:null,tolerances:{lateral:.03,yaw:3,omega:10},
  playback:{running:false,rate:1,anchorMs:0,anchorTime:0,request:null,lastPaint:0}};
const finite = Number.isFinite;
const clamp = (x,a,b) => Math.max(a,Math.min(b,x));
const fmt = (x,n=3) => finite(x) ? x.toFixed(n) : '—';
const signed = (x,n=3) => finite(x) ? (x > 0 ? '+' : '') + x.toFixed(n) : '—';
const escapeHtml = s => String(s).replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
function value(row,key,cfg=config) { const i=cfg.channels[key]; return row && Number.isInteger(i) && i>=0 && finite(row[i]) ? row[i] : NaN; }
function pulseToMeter(cfg=config) { return 2*Math.PI*cfg.radius/cfg.ppr; }
function kinematics(pulses,cfg=config) {
  if (!pulses.every(finite)) return {vx:NaN,vy:NaN,omega:NaN};
  const [a,b,c,d]=pulses.map(v=>v*pulseToMeter(cfg)), lever=(cfg.wheelbase+cfg.track)/2;
  return {vx:(a+b+c+d)/4,vy:(-a+b+c-d)/4,omega:(-a+b-c+d)/(4*lever)};
}
function actual(row,cfg=config) {
  const wheels=WHEELS.map(w=>value(row,w+'a',cfg)), k=kinematics(wheels,cfg);
  return {vx:cfg.channels.vx===null?k.vx:value(row,'vx',cfg),vy:cfg.channels.vy===null?k.vy:value(row,'vy',cfg),
    omega:cfg.omegaMode==='logged'?value(row,'wa',cfg):k.omega,wheelOmega:k.omega,wheels};
}
function targetBody(row,cfg=config) {
  const vx=value(row,'vxTarget',cfg),vy=value(row,'vyTarget',cfg),mode=value(row,'mode',cfg);
  if(!finite(vx)||!finite(vy)) return {vx:NaN,vy:NaN};
  /* Position mode logs world-frame targets; speed mode logs body-frame targets. */
  if(mode===1){const yaw=value(row,'yaw',cfg)/DEG,c=Math.cos(yaw),s=Math.sin(yaw);return {vx:vx*c+vy*s,vy:-vx*s+vy*c};}
  return {vx,vy};
}
function sample(row,cfg=config) {
  const a=actual(row,cfg),t=targetBody(row,cfg);return {...a,targetVx:t.vx,targetVy:t.vy,speed:Math.hypot(a.vx,a.vy),omegaDeg:a.omega*DEG,
    targetOmega:value(row,'wt',cfg)*DEG,targetSpeed:value(row,'speedTarget',cfg)*.073632,
    lateral:value(row,'lateral',cfg),yawError:value(row,'yawError',cfg),
    omegaError:(value(row,'wt',cfg)-a.omega)*DEG};
}
function parse(text) {
  const rows=[],lines=text.replace(/^\uFEFF/,'').split(/\r?\n/);let skipped=0,comments=0,headers=0;
  const bad=[];let source='native',header=null;
  for(let lineNo=0;lineNo<lines.length;lineNo++){
    const raw=lines[lineNo],line=raw.trim();if(!line){continue;}
    if(line.startsWith('#')||line.startsWith('//')){comments++;if(/converted\s+from\s+ULog26/i.test(line))source='converted';continue;}
    const parts=line.split(',');
    const numeric=parts.length>0&&parts.every(x=>!x.trim()||/^(nan|null)$/i.test(x.trim())||finite(Number(x)));
    if(!numeric){
      if(!rows.length&&parts.some(x=>/[a-zA-Z_]/.test(x))){headers++;header=parts.map(x=>x.trim());source='named';continue;}
      skipped++;if(bad.length<100)bad.push({line:lineNo+1,reason:'非数值行'});continue;
    }
    if(parts.length<43&&source!=='named'){skipped++;if(bad.length<100)bad.push({line:lineNo+1,reason:`列数 ${parts.length}，少于 43`});continue;}
    // Empty / NaN cells stay missing, never silently turn into zero.
    const row=parts.map(x=>!x.trim()||/^(nan|null)$/i.test(x.trim())?NaN:Number(x));
    if(!row.some(finite)){skipped++;if(bad.length<100)bad.push({line:lineNo+1,reason:'整行无有效数值'});continue;}rows.push(row);
  }
  if(!rows.length)throw new Error('没有找到 43 列数值日志。请导入本项目 ULog43 CSV；不要直接导入旧 26 列格式。');
  return {rows,skipped,comments,headers,bad,source,header,totalLines:lines.length};
}
// PID Burst is deliberately separate from ULog43: it is a high-rate, named-column
// export used after the vehicle has stopped. It never enters S.rows or the task view.
function normalizeBurstName(name) {
  return String(name || '').toLowerCase().replace(/\uFEFF/g,'').replace(/[^a-z0-9]+/g,'_').replace(/^_+|_+$/g,'');
}
function parsePidBurstCsv(text) {
  const lines=String(text||'').replace(/^\uFEFF/,'').split(/\r?\n/),comments=[],rows=[];let header=null,skipped=0;
  for(const raw of lines){let line=raw.trim();if(!line)continue;
    if(line.startsWith('#BTLOG,PID_BEGIN,')){header=line.split(',').slice(2).map(x=>x.trim());continue;}
    if(line==='#BTLOG,PID_END'||line.startsWith('#BTLOG,PID_END,')){comments.push(line);continue;}
    if(line.startsWith('#BTLOG,PID,'))line=line.slice('#BTLOG,PID,'.length);
    else if(line[0]==='#'||line.startsWith('//')){comments.push(line);continue;}
    const parts=line.split(',').map(x=>x.trim());
    if(!header){if(parts.some(x=>/[a-zA-Z_]/.test(x))){header=parts;continue;}skipped++;continue;}
    if(parts.length!==header.length){skipped++;continue;}
    const row=header.map((_,i)=>{const x=parts[i]||'';return !x||/^(nan|null)$/i.test(x)?NaN:Number(x);});
    if(row.some(finite))rows.push(row);else skipped++;
  }
  if(!header)throw new Error('PID Burst CSV 必须包含字段名表头。');
  if(!rows.length)throw new Error('PID Burst CSV 没有有效数值行。');
  const names=header.map(normalizeBurstName),find=aliases=>{for(const alias of aliases){const i=names.indexOf(normalizeBurstName(alias));if(i>=0)return i;}return null;};
  const fields={time:find(['timestamp_ms','time_ms','timestamp','t_ms','time','t'])};
  for(const w of WHEELS){
    fields[w+'_target']=find([`${w}_target`,`${w}_setpoint`,`${w}_reference`,`${w}_ref`,`target_${w}`,`${w}_commanded`]);
    fields[w+'_actual']=find([`${w}_actual`,`${w}_measured`,`${w}_speed`,`${w}_feedback`,`${w}_measurement`,`actual_${w}`]);
    fields[w+'_output']=find([`${w}_output`,`${w}_control`,`${w}_cmd`,`${w}_command`,`${w}_pwm`,`${w}_u`]);
    fields[w+'_error']=find([`${w}_error`,`${w}_err`,`error_${w}`]);
  }
  fields.vin=find(['vin','battery_v','voltage','bus_voltage']);fields.flags=find(['flags','status','state']);
  if(!WHEELS.some(w=>fields[w+'_target']!==null&&fields[w+'_actual']!==null))throw new Error('PID Burst CSV 至少需要一组 *_target 和 *_actual 字段。');
  return {header,names,rows,fields,comments,skipped,totalLines:lines.length};
}
function burstNumber(row,i){return Number.isInteger(i)&&finite(row?.[i])?row[i]:NaN;}
// Method references and limits: tools/BT10_BLE_RECEIVER.md. No external library
// code is embedded. Plant identification uses total actuator output, not target.
const pidMedian=a=>{const b=a.filter(finite).sort((x,y)=>x-y);return b.length?b[Math.floor(b.length/2)]:NaN;};
const pidMean=a=>a.length?a.reduce((s,x)=>s+x,0)/a.length:NaN;
const pidStd=a=>{const m=pidMean(a);return Math.sqrt(pidMean(a.map(x=>(x-m)**2)));};
function pidResponseMetrics(samples,tol=.02,satLimit=1000,lowSpeed=true) {
  const good=samples.filter(s=>finite(s.target)&&finite(s.actual)),maxTarget=good.reduce((a,s)=>Math.max(a,Math.abs(s.target)),0);
  const scale=Math.max(5,Math.sqrt(pidMean(good.map(s=>s.target*s.target)))||0),dts=samples.slice(1).map((s,i)=>s.t-samples[i].t),dt=pidMedian(dts.filter(x=>x>0))||.01;
  let iae=0,ise=0,duration=0,weighted=0,weights=0,tv=0,saturated=0,outputs=0,maxAbsError=0;
  for(let i=0;i<samples.length;i++){const s=samples[i];if(finite(s.output)){outputs++;if(satLimit>0&&Math.abs(s.output)>=satLimit*.999)saturated++;}
    if(!finite(s.target)||!finite(s.actual))continue;const e=s.target-s.actual;maxAbsError=Math.max(maxAbsError,Math.abs(e));
    const h=i&&dts[i-1]>0&&dts[i-1]<=dt*1.8?dts[i-1]:0,w=lowSpeed&&Math.abs(s.target)<=maxTarget*.35?2:1;
    iae+=Math.abs(e)*h;ise+=e*e*h;duration+=h;weighted+=w*Math.abs(e)*h;weights+=w*h;
    if(i&&finite(samples[i-1].output)&&finite(s.output))tv+=Math.abs(s.output-samples[i-1].output);
  }
  // Only flat target segments have a meaningful step overshoot/settling time.
  // Deceleration lag is tracking error, not positive-step overshoot.
  const segments=[];let start=0;const epsilon=Math.max(.5,maxTarget*.002);
  const finish=end=>{if(end<start)return;const p=samples[start],last=samples[end];if(!finite(p.target)||last.t-p.t<.15)return;
    const before=start?samples[start-1].target:p.actual,delta=p.target-before,band=Math.max(1,Math.abs(delta)*tol,Math.abs(p.target)*tol);
    let lastOutside=start-1,excess=0,tail=[];
    for(let j=start;j<=end;j++){const s=samples[j];if(!finite(s.actual)||Math.abs(s.actual-p.target)>band)lastOutside=j;
      if(Math.abs(delta)>Math.max(2,maxTarget*.05)&&finite(s.actual))excess=Math.max(excess,(s.actual-p.target)*Math.sign(delta));
      if(s.t>=last.t-(last.t-p.t)*.2&&finite(s.actual))tail.push(Math.abs(p.target-s.actual));}
    const settled=lastOutside+1<=end&&last.t-samples[Math.max(start,lastOutside+1)].t>=.1;
    segments.push({start:p.t,end:last.t,target:p.target,delta,steadyError:pidMean(tail),overshootPct:Math.abs(delta)>Math.max(2,maxTarget*.05)?100*excess/Math.abs(delta):null,settlingTime:settled?samples[Math.max(start,lastOutside+1)].t-p.t:null});};
  for(let i=1;i<samples.length;i++)if(!finite(samples[i].target)||!finite(samples[i-1].target)||Math.abs(samples[i].target-samples[start].target)>epsilon||dts[i-1]>dt*1.8){finish(i-1);start=i;}finish(samples.length-1);
  const overs=segments.map(s=>s.overshootPct).filter(finite),settles=segments.map(s=>s.settlingTime).filter(finite),steady=segments.map(s=>s.steadyError).filter(finite);
  const trackingRatio=weights?weighted/weights/scale:NaN,saturationRatio=outputs&&satLimit>0?saturated/outputs:NaN,overshootPct=overs.length?overs.reduce((a,b)=>Math.max(a,b),0):NaN;
  const cost=finite(trackingRatio)?trackingRatio+Math.max(0,overshootPct||0)/200+(saturationRatio||0)*.5+(outputs>1&&satLimit>0?tv/(outputs-1)/satLimit*.02:0):Infinity;
  return {n:good.length,iae,ise,maxAbsError,maxTarget,overshoot:maxTarget*(overshootPct||0)/100,overshootPct,settlingTime:settles.length?settles.reduce((a,b)=>Math.max(a,b),0):NaN,steadyError:pidMean(steady),saturationRatio,trackingRatio,cost,performanceScore:finite(cost)?Math.round(100/(1+4*cost)):null,segments,duration};
}
function analyzePidBurst(data,options={}) {
  const rate=clamp(Number(options.rate)||100,1,2000),rawTimes=data.rows.map((r,i)=>data.fields.time===null?i/rate:burstNumber(r,data.fields.time)/1000),positive=rawTimes.slice(1).map((t,i)=>t-rawTimes[i]).filter(x=>x>0),dt=pidMedian(positive)||1/rate;
  let nonMonotonic=0,missing=0,gaps=0,jitter=0;const times=[],issues=[];
  rawTimes.forEach((t,i)=>{if(!finite(t))missing++;if(i){const h=t-rawTimes[i-1];if(h<=0){nonMonotonic++;if(issues.length<10)issues.push({row:i+1,previous:rawTimes[i-1],time:t,kind:'rollback_or_duplicate'});}if(h>dt*1.8){gaps++;if(issues.length<10)issues.push({row:i+1,previous:rawTimes[i-1],time:t,kind:'gap'});}else if(h>0&&Math.abs(h-dt)>Math.max(.00101,dt*.15))jitter++;}times.push(finite(t)&&(!i||t>times[i-1])?t:(i?times[i-1]+dt:0));});
  const tolerance=clamp(Number(options.tolerance)||.02,.001,.5),satLimit=Math.max(0,Number(options.satLimit)||0),wheels={};
  for(const w of WHEELS){const samples=data.rows.map((r,i)=>({t:times[i],target:burstNumber(r,data.fields[w+'_target']),actual:burstNumber(r,data.fields[w+'_actual']),output:burstNumber(r,data.fields[w+'_output'])}));
    wheels[w]={samples,...pidResponseMetrics(samples,tolerance,satLimit),hasTarget:data.fields[w+'_target']!==null,hasActual:data.fields[w+'_actual']!==null,hasOutput:data.fields[w+'_output']!==null};}
  const span=rawTimes.at(-1)-rawTimes[0];
  return {times,wheels,rate,tolerance,satLimit,rows:data.rows.length,quality:{dt,measuredRate:span>0&&!nonMonotonic&&!missing?(rawTimes.length-1)/span:NaN,medianRate:1/dt,issues,nonMonotonic,missing,gaps,jitter,inferredTime:data.fields.time===null,skipped:data.skipped||0}};
}
function pidLinearSolve(matrix,vector){
  const a=matrix.map((r,i)=>[...r,vector[i]]),n=vector.length;
  for(let j=0;j<n;j++){let p=j;for(let i=j+1;i<n;i++)if(Math.abs(a[i][j])>Math.abs(a[p][j]))p=i;
    if(Math.abs(a[p][j])<1e-10)return null;[a[j],a[p]]=[a[p],a[j]];const v=a[j][j];for(let k=j;k<=n;k++)a[j][k]/=v;
    for(let i=0;i<n;i++)if(i!==j){const f=a[i][j];for(let k=j;k<=n;k++)a[i][k]-=f*a[j][k];}}
  return a.map(r=>r[n]);
}
function pidRobustRegression(rows,friction){
  const features=rows.map(r=>friction?[r.y,r.u,Math.sign(r.u)]:[r.y,r.u]),p=features[0].length;
  const means=Array.from({length:p},(_,j)=>pidMean(features.map(r=>r[j]))),scales=means.map((_,j)=>pidStd(features.map(r=>r[j])));
  if(scales.some(x=>x<1e-6))return null;const x=features.map(r=>[1,...r.map((v,j)=>(v-means[j])/scales[j])]);let weights=rows.map(()=>1),coef;
  for(let it=0;it<3;it++){const a=Array.from({length:p+1},()=>Array(p+1).fill(0)),b=Array(p+1).fill(0);
    rows.forEach((r,i)=>{for(let j=0;j<=p;j++){b[j]+=weights[i]*x[i][j]*r.next;for(let k=0;k<=p;k++)a[j][k]+=weights[i]*x[i][j]*x[i][k];}});
    for(let j=1;j<=p;j++)a[j][j]+=1e-5;coef=pidLinearSolve(a,b);if(!coef)return null;
    const residual=rows.map((r,i)=>r.next-x[i].reduce((s,v,j)=>s+v*coef[j],0)),sigma=Math.max(1e-6,pidMedian(residual.map(Math.abs))*1.4826);
    weights=residual.map(e=>Math.min(1,1.5*sigma/Math.max(1e-9,Math.abs(e))));}
  const raw=coef.slice(1).map((v,j)=>v/scales[j]);return {a:raw[0],b:raw[1],friction:friction?raw[2]:0,bias:coef[0]-raw.reduce((s,v,j)=>s+v*means[j],0)};
}
function pidModelErrors(model,samples,start,end,scale){
  let y=samples[start-1].actual,free=0,one=0,n=0;const residual=[];
  for(let k=start;k<end;k++){const u=samples[k-1-model.delay].output,force=model.b*u+model.friction*Math.sign(u)+model.bias;
    const yp=model.a*samples[k-1].actual+force;y=model.a*y+force;const e=samples[k].actual-yp;
    one+=e*e;free+=(samples[k].actual-y)**2;n++;residual.push(e);}
  const mean=pidMean(residual),den=residual.reduce((s,e)=>s+(e-mean)**2,0),corr=residual.slice(1).reduce((s,e,i)=>s+(e-mean)*(residual[i]-mean),0);
  return {oneStepNrmse:Math.sqrt(one/n)/scale,freeRunNrmse:Math.sqrt(free/n)/scale,residualLag1:den>1e-9?corr/den:0,n};
}
function pidOutputErrorFit(samples,end,dt,delay,tau,friction){
  // Variable projection: for a fixed tau/delay, free-run output is linear in
  // process gain, Coulomb term and bias. Unlike ARX this does not regress on
  // noisy measured speed at every step (GEKKO's model-vs-meas distinction).
  const a=Math.exp(-dt/tau),x=[],ys=[];let fu=0,fs=0,fc=0,initial=samples[0].actual;
  for(let k=1;k<end;k++){const u=samples[Math.max(0,k-1-delay)].output;fu=a*fu+(1-a)*u;fs=a*fs+(1-a)*Math.sign(u);fc=a*fc+1-a;initial*=a;
    if(k>delay){x.push(friction?[fu,fs,fc]:[fu,fc]);ys.push(samples[k].actual-initial);}}
  const n=x[0].length,scales=Array.from({length:n},(_,j)=>Math.sqrt(pidMean(x.map(r=>r[j]*r[j]))));if(scales.some(v=>v<1e-6))return null;
  const z=x.map(r=>r.map((v,j)=>v/scales[j]));let weights=x.map(()=>1),coef;
  for(let it=0;it<2;it++){const mat=Array.from({length:n},()=>Array(n).fill(0)),vec=Array(n).fill(0);
    for(let i=0;i<x.length;i++)for(let j=0;j<n;j++){vec[j]+=weights[i]*z[i][j]*ys[i];for(let k=0;k<n;k++)mat[j][k]+=weights[i]*z[i][j]*z[i][k];}
    for(let j=0;j<n;j++)mat[j][j]+=1e-5;coef=pidLinearSolve(mat,vec);if(!coef)return null;
    const e=z.map((r,i)=>ys[i]-r.reduce((s,v,j)=>s+v*coef[j],0)),sigma=Math.max(.1,pidMedian(e.map(Math.abs))*1.4826);weights=e.map(v=>Math.min(1,1.5*sigma/Math.max(1e-6,Math.abs(v))));}
  coef=coef.map((v,j)=>v/scales[j]);if(!(coef[0]>0&&coef[0]<100))return null;
  return {a,b:coef[0]*(1-a),friction:friction?coef[1]*(1-a):0,bias:coef[friction?2:1]*(1-a),delay,dt,tau,gain:coef[0],deadTime:delay*dt,method:'output-error'};
}
// Firmware order: measured y[k] -> PID -> applied u[k]; the plant sees u[k-1].
function pidControllerStep(state,r,y,params,control){
  const h=control.periodMs/1000,e=r-y,sign=Math.abs(r)>control.ffMinSpeed?Math.sign(r):0;
  const ff=sign?r*control.ffGain+sign*control.deadzone:0,p=params.kp*e,d=params.kd*(e-state.previousError)/h,pre=ff+p+d;
  state.previousError=e;
  if(pre<control.outputMax&&pre> -control.outputMax)state.integral=clamp(state.integral+e*h,-control.integralMax,control.integralMax);
  state.kick=sign&&Math.abs(y)<control.stuckRatio*Math.abs(r)?Math.min(control.kickMax,state.kick+control.kickStep):state.kick*.97;
  if(state.kick<1)state.kick=0;
  let next=pre+params.ki*state.integral+sign*state.kick;if(sign&&next*sign<0)next=0;
  const limited=clamp(next,-control.outputMax,control.outputMax);
  state.output+=clamp(limited-state.output,-control.slew,control.slew);
  return {ff,p,d,pre,integral:state.integral,kick:state.kick,limited,output:state.output};
}
function pidControllerInitial(s,params,control){
  const e=s.target-s.actual,sign=Math.abs(s.target)>control.ffMinSpeed?Math.sign(s.target):0;
  return {output:s.output,previousError:e,kick:0,integral:params.ki>0?clamp((s.output-(sign?s.target*control.ffGain+sign*control.deadzone:0)-params.kp*e)/params.ki,-control.integralMax,control.integralMax):0};
}
function auditPidController(metric,control){
  const samples=metric.samples,h=control.periodMs/1000,state=pidControllerInitial(samples[0],control,control);
  let error=0,n=0,aligned=true,maxError=0;const regimes={forward:{n:0,trackingError:0,saturated:0,stall:0},reverse:{n:0,trackingError:0,saturated:0,stall:0},zero:{n:0,trackingError:0,saturated:0,stall:0}};
  for(let i=1;i<samples.length;i++){const s=samples[i],gap=s.t-samples[i-1].t;
    if(![s.target,s.actual,s.output].every(finite)||Math.abs(gap-h)>.00101){aligned=false;continue;}
    const predicted=pidControllerStep(state,s.target,s.actual,control,control).output;
    if(i>40){const e=Math.abs(predicted-s.output);error+=e;maxError=Math.max(maxError,e);n++;}
    const regime=regimes[s.target>control.ffMinSpeed?'forward':s.target< -control.ffMinSpeed?'reverse':'zero'];regime.n++;regime.trackingError+=Math.abs(s.target-s.actual);if(Math.abs(s.output)>=control.outputMax*.999)regime.saturated++;
    if(Math.abs(s.target)>Math.max(20,metric.maxTarget*.05)&&Math.abs(s.actual)<Math.max(10,Math.abs(s.target)*.1)&&Math.abs(s.output)>control.outputMax*.3)regime.stall++;
  }
  for(const r of Object.values(regimes)){r.meanAbsError=r.n?r.trackingError/r.n:null;r.saturationRatio=r.n?r.saturated/r.n:null;r.stallRatio=r.n?r.stall/r.n:null;delete r.trackingError;}
  return {aligned,initialStateEstimated:true,meanAbsOutputError:n?error/n:null,normalizedError:n?error/n/control.outputMax:null,maxAbsOutputError:n?maxError:null,regimes,meaning:'逐帧使用实测轮速；不插值补造控制周期。初始积分/kick仍为估计，跳过前40帧不代表状态已知；不是电机模型验证',stallMeaning:'非零目标、接近零轮速且输出>30%限幅仅提示停滞；先检查反馈/驱动/机械，不因评分低强推PID'};
}
function pidRegimeErrors(model,samples,start,end,scale){
  const groups={forward:[],reverse:[],low:[],high:[]},max=samples.reduce((v,s)=>Math.max(v,Math.abs(s.target)),0);let y=samples[start-1].actual;
  for(let k=start;k<end;k++){const u=samples[k-1-model.delay].output;y=model.a*y+model.b*u+model.friction*Math.sign(u)+model.bias;
    const e=(samples[k].actual-y)/scale,r=samples[k].target;
    if(Math.abs(r)>5){groups[r>0?'forward':'reverse'].push(e);groups[Math.abs(r)<=max*.35?'low':'high'].push(e);}}
  return Object.fromEntries(Object.entries(groups).map(([k,v])=>[k,{n:v.length,freeRunNrmse:v.length?Math.sqrt(pidMean(v.map(e=>e*e))):null}]));
}
let pidValidation=null;
function validatePidExperiment(plant,analysis,wheel){
  const q=analysis.quality||{},samples=analysis.wheels[wheel]?.samples||[],reasons=[];
  if(!plant.model)return {valid:false,reasons:['训练模型不可用']};
  if(q.inferredTime||q.nonMonotonic||q.gaps||q.jitter||q.missing||q.skipped||Math.abs(q.dt-plant.model.dt)>.0001||samples.length<160||samples.some(s=>![s.t,s.target,s.actual,s.output].every(finite)))return {valid:false,reasons:['验证文件须为独立完整200Hz目标/实测/输出日志，不拼接、不重采样']};
  if(pidStd(samples.map(s=>s.target))<=3||pidStd(samples.map(s=>s.output))<=1||pidStd(samples.map(s=>s.actual))<=5)return {valid:false,reasons:['验证试验激励/轮速变化不足，静止文件不能证明模型泛化']};
  const start=plant.model.delay+1,test=pidModelErrors(plant.model,samples,start,samples.length,plant.scale),regimes=pidRegimeErrors(plant.model,samples,start,samples.length,plant.scale);
  if(test.freeRunNrmse>.25||test.oneStepNrmse>.15)reasons.push('跨试验预测未通过：自由运行≤25%、单步≤15%');
  for(const [name,r] of Object.entries(regimes))if(r.n>=40&&r.freeRunNrmse>.25)reasons.push(name+'分组自由运行误差超过25%');
  return {valid:!reasons.length,reasons,test,regimes,meaning:'冻结原模型，用另一文件的实际输出预测；不重新拟合，也不证明新PID已改善'};
}
const pidPlantCache=new WeakMap(),pidCandidateCache=new WeakMap();
let pidPlotState=null;
function identifyPidPlant(analysis,wheel){const key=analysis.wheels[wheel];if(pidPlantCache.has(key))return pidPlantCache.get(key);const result=computePidPlant(analysis,wheel);pidPlantCache.set(key,result);return result;}
function computePidPlant(analysis,wheel){
  const m=analysis.wheels[wheel],q=analysis.quality||{},reasons=[];
  if(!m?.hasOutput)reasons.push('缺少该轮实际输出，不能用目标速度冒充电机输入');
  if(q.inferredTime||q.missing||q.nonMonotonic||q.gaps||q.jitter||q.skipped)reasons.push('时间戳缺失、重复、跳帧或采样不均匀，请使用完整200Hz实时PID CSV；100Hz缓存仅用于诊断');
  if(m?.n<160||!(q.dt>0&&Math.abs(q.dt-.005)<.0001))reasons.push('辨识须至少160帧完整200Hz实时目标/实测/输出数据；100Hz缓存不能恢复5ms控制过程');
  if(q.issues?.length)reasons.push(wheel.toUpperCase()+'异常区间：'+q.issues.map(v=>'第'+v.row+'数据帧 '+fmt(v.previous,3)+'→'+fmt(v.time,3)+'s').join('；')+'；分别导出每次采集，不能拼接复位前后日志');
  let samples=(m?.samples||[]).slice(-12000);if(samples.some(s=>![s.t,s.target,s.actual,s.output].every(finite)))reasons.push('目标、实测或输出存在缺失值');
  const targetStd=pidStd(samples.map(s=>s.target)),outputStd=pidStd(samples.map(s=>s.output));let scale=Math.max(5,pidStd(samples.map(s=>s.actual))||0);
  if(samples.some(v=>Math.abs(v.target)>5)&&samples.every(v=>Math.abs(v.actual)<.1))reasons.push(wheel.toUpperCase()+'实测轮速全为零：先检查编码器/驱动/机械，不能靠改PID解决缺失反馈');
  if(!(targetStd>Math.max(3,(m?.maxTarget||0)*.025)&&outputStd>1&&scale>5))reasons.push('目标或控制输出激励不足，恒速/静止数据不能唯一辨识动态模型');
  if(reasons.length)return {valid:false,reasons};
  // Preserve each observed controller sample, not an interpolated 5ms waveform.
  // HAL ms timestamps have 5/6ms quantisation; model time is firmware control time.
  samples=samples.map((v,i)=>({...v,observedTime:v.t,t:samples[0].t+i*q.dt}));
  const train=Math.floor(samples.length*.6),validation=Math.floor(samples.length*.8),models=[];
  scale=Math.max(5,pidStd(samples.slice(0,train).map(s=>s.actual))||0);
  if(scale<=5)return {valid:false,reasons:['训练时段轮速变化不足，不能用后续测试时段替代辨识激励']};
  for(let delay=0;delay<=Math.min(20,Math.floor(.12/q.dt));delay++)for(const friction of [false,true]){
    const rows=[];for(let k=delay+1;k<train;k++)rows.push({y:samples[k-1].actual,u:samples[k-1-delay].output,next:samples[k].actual});
    const fit=pidRobustRegression(rows,friction);if(!fit||!(fit.a>0&&fit.a<.9995&&fit.b>0))continue;
    const model={...fit,delay,dt:q.dt,tau:-q.dt/Math.log(fit.a),gain:fit.b/(1-fit.a),deadTime:delay*q.dt,method:'ARX'};
    if(model.tau<q.dt*1.5||model.tau>5||model.gain>100)continue;
    const v=pidModelErrors(model,samples,train,validation,scale);if(finite(v.freeRunNrmse))models.push({...model,validation:v});}
  for(const tau of [.01,.016,.025,.04,.063,.1,.16,.25,.4,.63,1,1.6,2.5]){
    if(tau<q.dt*1.5)continue;for(let delay=0;delay<=Math.min(20,Math.floor(.12/q.dt));delay+=2)for(const friction of [false,true]){
      const model=pidOutputErrorFit(samples,train,q.dt,delay,tau,friction);if(!model)continue;const v=pidModelErrors(model,samples,train,validation,scale);if(finite(v.freeRunNrmse))models.push({...model,validation:v});}}
  models.sort((a,b)=>a.validation.freeRunNrmse-b.validation.freeRunNrmse);
  if(!models.length)return {valid:false,reasons:['未找到稳定正增益模型；可能存在编码器方向、非线性或闭环辨识偏差']};
  const model=models[0],test=pidModelErrors(model,samples,validation,samples.length,scale);
  if(model.validation.freeRunNrmse>.25||test.freeRunNrmse>.25||test.oneStepNrmse>.15)reasons.push('独立时段自由运行预测未通过（归一化误差须≤25%，单步≤15%）');
  const regimes=pidRegimeErrors(model,samples,validation,samples.length,scale);
  for(const [name,r] of Object.entries(regimes))if(r.n>=40&&r.freeRunNrmse>.25)reasons.push(name+'测试分组预测误差超过25%');
  const scope={directions:[...new Set(samples.slice(0,train).filter(s=>Math.abs(s.target)>5).map(s=>s.target>0?'forward':'reverse'))],targetRange:[Math.min(...samples.slice(0,train).map(s=>s.target)),Math.max(...samples.slice(0,train).map(s=>s.target))],clock:'firmware_control_period',used:samples.length,omitted:Math.max(0,m.samples.length-samples.length)};
  return {valid:!reasons.length,reasons,model,test,regimes,scope,scale,samples,split:{train,validation,test:samples.length-validation},fitPercent:clamp(100*(1-test.freeRunNrmse),0,100),limitations:['闭环日志辨识可能有噪声偏差，模型通过不代表实车保证','模型已包含测速低通的有效动态，不重复叠加LPF','单轮有效模型不包含载重变化、打滑、四轮耦合与电池变化']};
}
function simulatePidCandidate(plant,params,control,variation={gain:1,tau:1,delay:0}){
  const samples=plant.samples,model=plant.model,h=control.periodMs/1000;
  if(!(h>0&&Math.abs(model.dt-h)<h*.01))return null;
  const a=Math.exp(-h/(model.tau*variation.tau)),b=model.gain*variation.gain*(1-a),factor=(1-a)/(1-model.a),delay=Math.max(0,Math.round((model.deadTime+variation.delay)/h));
  const history=Array(delay+1).fill(samples[0].output),pred=[{...samples[0]}],state=pidControllerInitial(samples[0],control,control);let y=samples[0].actual;
  for(let i=1;i<samples.length;i++){
    const delayed=history.shift();
    y=a*y+b*delayed+factor*(model.friction*Math.sign(delayed)+model.bias);
    if(!finite(y)||Math.abs(y)>Math.max(200,plant.scale*40))return null;
    const u=pidControllerStep(state,samples[i].target,y,params,control).output;history.push(u);
    pred.push({t:samples[i].t,observedTime:samples[i].observedTime,target:samples[i].target,actual:y,output:u});
  }
  return {samples:pred,...pidResponseMetrics(pred,.02,control.outputMax)};
}
function evaluatePidProposal(plant,base,control,candidate,baselineRuns){
  const scenarios=[{gain:1,tau:1,delay:0},{gain:.8,tau:1.2,delay:plant.model.dt},{gain:1.2,tau:.8,delay:0}];
  const runs=scenarios.map(v=>simulatePidCandidate(plant,candidate,control,v));if(runs.some(r=>!r))return {...candidate,score:null,eligible:false,reason:'模型仿真发散或采样周期不支持'};
  const improvements=runs.map((r,i)=>100*(baselineRuns[i].cost-r.cost)/Math.max(.001,baselineRuns[i].cost)),improvement=pidMedian(improvements),worstImprovement=Math.min(...improvements);
  const risky=runs.some((r,i)=>r.saturationRatio>baselineRuns[i].saturationRatio+.03||(r.overshootPct||0)>(baselineRuns[i].overshootPct||0)+5);
  return {...candidate,score:runs[0].performanceScore,prediction:{trackingRatio:runs[0].trackingRatio,saturationRatio:runs[0].saturationRatio,overshootPct:runs[0].overshootPct,improvement,worstImprovement,cost:runs[0].cost},eligible:improvement>=2&&worstImprovement>=0&&!risky,reason:(candidate.reason||'')+'；模型预测改善 '+fmt(improvement,1)+'%，压力场景最差 '+fmt(worstImprovement,1)+'%'+(risky?'；超调/饱和风险增加':'')};
}
function buildPidCandidates(analysis,base={},options={}){
  const key=WHEELS.map(w=>analysis.wheels[w]).find(m=>m?.n);if(!key)return computePidCandidates(analysis,base,options);
  const signature=JSON.stringify({base,control:options.control}),old=pidCandidateCache.get(key);if(old?.signature===signature&&old.validation===options.validation)return old.result;
  const result=computePidCandidates(analysis,base,options);pidCandidateCache.set(key,{signature,validation:options.validation,result});return result;
}
function computePidCandidates(analysis,base={},options={}){
  const wheel=WHEELS.find(w=>analysis.wheels[w]?.n),m=analysis.wheels[wheel];
  const summary={overshoot:m?.overshootPct,settling:m?.settlingTime,sat:m?.saturationRatio,steady:m?.steadyError,steadyRatio:m?.maxTarget?m.steadyError/m.maxTarget:null};
  if(!wheel)return {candidates:[],summary,plant:{valid:false,reasons:['没有有效轮速样本']}};
  const plant={...identifyPidPlant(analysis,wheel)},control=options.control,controllerAudit=control?auditPidController(m,control):null;
  if(controllerAudit)plant.controllerAudit=controllerAudit;
  if(options.validation){plant.external=validatePidExperiment(plant,options.validation,wheel);if(!plant.external.valid){plant.valid=false;plant.reasons=[...(plant.reasons||[]),...plant.external.reasons];}}
  plant.limitations=[...(plant.limitations||[]),options.validation?'跨文件结果只验证电机模型，不证明新参数实车改善':'尚无跨试验验证，只允许作为本轮工况下待复测候选'];
  if(!plant.valid||!control)return {candidates:[],summary,plant,warnings:control?[]:['缺少唯一的车辆实际PID与前馈/限幅快照，不能验证候选']};
  if(!controllerAudit.aligned||!finite(controllerAudit.normalizedError)||controllerAudit.normalizedError>.08)return {candidates:[],summary,plant,warnings:['实测反馈驱动的控制输出重放偏差超过8%限幅，参数快照/初始状态/采样不足以验证控制器，暂停推荐']};
  const scenarios=[{gain:1,tau:1,delay:0},{gain:.8,tau:1.2,delay:plant.model.dt},{gain:1.2,tau:.8,delay:0}],baselineRuns=scenarios.map(v=>simulatePidCandidate(plant,base,control,v));
  if(baselineRuns.some(r=>!r))return {candidates:[],summary,plant,warnings:['基准控制器仿真发散或采样周期不匹配']};
  const replayError=Math.sqrt(pidMean(baselineRuns[0].samples.map((s,i)=>(s.actual-plant.samples[i].actual)**2)))/plant.scale;
  plant.controllerReplayNrmse=replayError;
  if(replayError>.25)return {candidates:[],summary,plant,warnings:['实际PID基准闭环重放误差超过25%，模型或初始状态不可信，暂停推荐']};
  const bounded=p=>Object.fromEntries(['kp','ki','kd'].map((k,i)=>{const lo=Math.max(0,base[k]*.8),hi=Math.min([10,8,3][i],Math.max(base[k]*1.2,k==='kd'?base.kp*control.periodMs/1000*.1:.0001));return [k,clamp(p[k],lo,hi)];}));
  const raw=[];for(const p of [.8,.9,1,1.1,1.2])for(const i of [.8,1,1.2])raw.push({name:'局部搜索 P×'+p+' I×'+i,...bounded({kp:base.kp*p,ki:base.ki*i,kd:base.kd}),reason:'保留实车前馈、条件积分、同向约束、kick和输出slew'});
  // SIMC PI: Kc=tau/(K*(lambda+theta)), Ti=min(tau,4*(lambda+theta)).
  // Absolute SIMC values are projected into the same small-step trust region.
  for(const factor of [1,2,4]){const lambda=Math.max(plant.model.deadTime,plant.model.tau*.5,control.periodMs/1000*4)*factor,kp=plant.model.tau/(plant.model.gain*(lambda+plant.model.deadTime)),ti=Math.min(plant.model.tau,4*(lambda+plant.model.deadTime));raw.push({name:'SIMC PI λ×'+factor,...bounded({kp,ki:kp/ti,kd:0}),reason:'SIMC模型种子；按实际固件并联式Ki换算后限制单次改变量'});}
  for(const d of [.025,.05,.1])for(const p of [.9,1.1])raw.push({name:'联合P/D小步搜索',...bounded({...base,kp:base.kp*p,kd:base.kd>0?base.kd*(1+d):base.kp*control.periodMs/1000*d}),reason:'微分上限沿用原步幅边界；低速量化噪声下仍须复测'});
  if(base.kd>0)raw.push({name:'降低微分噪声',...bounded({...base,kd:base.kd*.8}),reason:'当前固件对误差微分，保留其实现并检查减小D的效果'});
  const unique=[...new Map(raw.filter(c=>['kp','ki','kd'].some(k=>Math.abs(c[k]-base[k])>1e-8)).map(c=>[['kp','ki','kd'].map(k=>c[k].toFixed(8)).join(','),c])).values()];
  const evaluated=unique.map(c=>evaluatePidProposal(plant,base,control,c,baselineRuns));
  let seed=evaluated.reduce((a,b)=>(b.prediction?.worstImprovement??-Infinity)>(a.prediction?.worstImprovement??-Infinity)?b:a,evaluated[0]);
  for(const step of [.1,.05]){if(!seed)break;const neighbors=[];for(const k of ['kp','ki','kd'])for(const sign of [-1,1]){const delta=k==='kd'?Math.max(base.kd,base.kp*control.periodMs/1000*.1)*step:Math.max(base[k],.0001)*step;neighbors.push(evaluatePidProposal(plant,base,control,{name:'有界坐标细化',...bounded({...seed,[k]:seed[k]+sign*delta}),reason:'Kp/Ki/Kd联合局部优化，保持原步幅与压力门控'},baselineRuns));}evaluated.push(...neighbors);seed=[seed,...neighbors].sort((a,b)=>(b.prediction?.worstImprovement??-Infinity)-(a.prediction?.worstImprovement??-Infinity))[0];}
  const candidates=[...new Map(evaluated.map(c=>[['kp','ki','kd'].map(k=>c[k].toFixed(8)).join(','),c])).values()].sort((a,b)=>(b.eligible-a.eligible)||((b.prediction?.worstImprovement??-Infinity)-(a.prediction?.worstImprovement??-Infinity))).slice(0,6);
  return {candidates,summary,plant,baselineRuns,baselinePrediction:{score:baselineRuns[0].performanceScore,cost:baselineRuns[0].cost},warnings:candidates.some(c=>c.eligible)?[]:['没有候选在全部压力场景中改善；保留原参数，不为提高分数强行推荐']};
}
function buildAiPidAdvice(analysis,base={},options={}){
  const result=buildPidCandidates(analysis,base,options),metric=WHEELS.map(w=>analysis.wheels[w]).find(m=>m?.n);
  return {...result,ranked:result.candidates,confidence:result.plant.valid?Math.round(Math.min(90,result.plant.fitPercent)):0,warnings:[...(result.plant.reasons||[]),...(result.warnings||[]),...(result.plant.limitations||[])],summary:{...result.summary,lowSpeed:options.lowSpeed!==false,ffAware:options.feedforward!==false,measuredScore:metric?.performanceScore,trackingRatio:metric?.trackingRatio}};
}
function pidEvidencePayload(context,local){
  const metric=WHEELS.map(w=>context.analysis.wheels[w]).find(m=>m?.n),samples=metric?.samples||[],stride=Math.max(1,Math.ceil(samples.length/160));
  const series=[];for(let i=0;i<samples.length;i+=stride){const chunk=samples.slice(i,i+stride),s=chunk[0];series.push({t:s.t,target:s.target,actual:s.actual,output:s.output,minActual:Math.min(...chunk.map(s=>s.actual)),maxActual:Math.max(...chunk.map(s=>s.actual)),maxAbsOutput:Math.max(...chunk.map(s=>Math.abs(s.output)))});}
  const p=local.plant;return {version:2,axis:context.axis,recordedParameters:context.known?context.used:null,baseline:context.base,summary:local.summary,quality:context.analysis.quality,controllerAudit:p.controllerAudit,model:{valid:p.valid,reasons:p.reasons,parameters:p.model,test:p.test,regimes:p.regimes,scope:p.scope,external:p.external,split:p.split,controllerReplayNrmse:p.controllerReplayNrmse,limitations:p.limitations},segments:(metric?.segments||[]).slice(-30),series,seriesStride:stride,baselinePrediction:local.baselinePrediction,candidates:local.ranked.map(({name,kp,ki,kd,score,reason,prediction,eligible})=>({name,kp,ki,kd,score,reason,prediction,eligible})),lowSpeed:true,feedforward:true};
}
async function requestDeepSeekAdvice(){
  const status=$('pid-deepseek-status'),out=$('pid-deepseek-advice');if(!S.pidBurst){status.textContent='请先导入 PID 数据。';return;}
  const validationState=pidValidation,state=S.pidBurst,context=pidAnalysisContext(),local=buildAiPidAdvice(context.analysis,context.base,{control:context.known?context.used:null,validation:pidValidation?.analysis}),payload=pidEvidencePayload(context,local);
  if(!context.axis){status.textContent='本模块分析四轮速度环；请先选择 FL/FR/RL/RR。位置环仍支持手动在线读写。';return;}
  status.textContent='正在发送实测分段、时序摘要和模型验证证据给 DeepSeek…';
  try{const endpoint=String($('pid-deepseek-url')?.value||'http://127.0.0.1:8787').replace(/\/$/,''),response=await fetch(endpoint+'/v1/pid-advice',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(payload)}),result=await response.json();if(!response.ok)throw new Error(result.error||`HTTP ${response.status}`);
    if(pidValidation!==validationState||S.pidBurst!==state||S.pidBurst.analysis!==context.sourceAnalysis||pidAnalysisContext().axis!==context.axis){status.textContent='分析对象已改变，已忽略旧 AI 结果。';return;}
    const proposed=Array.isArray(result.proposed)?result.proposed:[],checked=[];
    if(local.baselineRuns&&context.known)for(const c of proposed){if(!pidProposalInBounds(c,context.base,context.used))continue;checked.push(evaluatePidProposal(local.plant,context.base,context.used,{...c,name:'AI · '+c.name},local.baselineRuns));}
    const accepted=checked.filter(c=>c.eligible);pidOnline.candidates.push(...accepted);
    const start=pidOnline.candidates.length-accepted.length;
    out.innerHTML='<h4>DeepSeek 证据分析</h4><p>'+escapeHtml(result.diagnosis||'无诊断')+'</p>'+(result.ranked||[]).map(x=>'<p><b>'+escapeHtml(x.name)+'</b>：'+escapeHtml(x.reason)+'</p>').join('')+(result.warnings||[]).map(x=>'<p class="warning">'+escapeHtml(x)+'</p>').join('')+checked.map(c=>'<p>'+escapeHtml(c.name)+'：'+escapeHtml(c.reason)+(c.eligible?'（通过本地模型检查）':'（拒绝填入）')+'</p>').join('')+accepted.map((c,i)=>'<button data-ai-candidate="'+(start+i)+'">填入 '+escapeHtml(c.name)+'，仍需停车写入</button>').join('')+'<p>建议补采：'+escapeHtml(result.next_experiment||'无')+'</p><p class="note">AI 判断不替代实车验证，也不改变本地评分。数据不足时只诊断，不开放候选填入。</p>';
    out.querySelectorAll('[data-ai-candidate]').forEach(b=>b.onclick=()=>usePidCandidate(Number(b.dataset.aiCandidate)));status.textContent='DeepSeek 分析完成；AI 提案 '+checked.length+' 组，本地检查通过 '+accepted.length+' 组。';
  }catch(err){status.textContent='DeepSeek 不可用：'+err.message+'；本地分析仍可用。';}
}
function pidProposalInBounds(c,base,control){return c&&['kp','ki','kd'].every((k,i)=>finite(c[k])&&c[k]>=Math.max(0,base[k]*.8)-1e-8&&c[k]<=Math.min([10,8,3][i],Math.max(base[k]*1.2,k==='kd'?base.kp*control.periodMs/1000*.1:.0001))+1e-8);}

async function connectDeepSeek() {
  const input=$('pid-deepseek-url'), status=$('pid-deepseek-status'), button=$('pid-deepseek');
  const endpoint=String(input?.value||'').replace(/\/$/,'');
  if(!/^https?:\/\//i.test(endpoint)){if(status)status.textContent='代理地址必须以 http:// 或 https:// 开头。';return false;}
  if(status)status.textContent='正在检查本地代理…';
  try { const response=await fetch(endpoint+'/health',{cache:'no-store'}); const data=await response.json(); if(!response.ok||!data.ok)throw new Error(data.error||`HTTP ${response.status}`); if(!data.key_configured)throw new Error('代理未配置 DEEPSEEK_API_KEY'); if(Number(data.advisor_version)<3||!data.advisor_version)throw new Error('AI代理仍是旧版，请重新运行 tools/start_deepseek_pid.ps1 完成升级'); if(button)button.disabled=false; if(status)status.textContent='DeepSeek v3 已连接，可分析时序证据。'; return true; }
  catch(err){if(button)button.disabled=true;if(status)status.textContent='连接失败：'+err.message+'；请先启动本机代理。';return false;}
}function renderPidBurst(){
  const box=$('pid-burst-metrics'),cand=$('pid-burst-candidates'),status=$('pid-burst-status'),ai=$('pid-ai-advice');if(!box||!cand||!status)return;
  pidOnline.candidates=[];pidOnline.candidateAxis=null;if($('pid-deepseek-advice'))$('pid-deepseek-advice').innerHTML='';
  const state=S.pidBurst;if(!state){pidPlotState=null;drawPidModelTrace();status.textContent='尚未导入高频 PID CSV。';box.innerHTML='<div class="pid-burst-empty">请导入带字段名的 PID CSV。</div>';cand.innerHTML='';if(ai)ai.innerHTML='导入数据后生成证据分析。';return;}
  const a=state.analysis,q=a.quality;status.textContent=`已加载 ${state.name}：${a.rows} 帧，时长 ${fmt(a.times.at(-1)-a.times[0],2)} s，实测 ${fmt(q.measuredRate,1)} Hz。跳帧 ${q.gaps}，乱序/重复 ${q.nonMonotonic}，缺失时间 ${q.missing}，抖动 ${q.jitter}。${q.nonMonotonic?'跨复位时序无效，显示时间仅用于查看字段，不能用于响应/频率分析。':''}`;
  box.innerHTML=WHEELS.map(w=>{const m=a.wheels[w];return `<div class="pid-burst-wheel"><h3 style="color:${COLORS[w]}">${NAMES[w]}</h3><table>${tableRows([['有效样本',m.n],['IAE',fmt(m.iae,4)],['ISE',fmt(m.ise,4)],['归一化跟踪误差',fmt(m.trackingRatio*100,1)+' %'],['本轮实测表现',m.performanceScore===null?'不可计算':m.performanceScore+'/100'],['阶跃超调',finite(m.overshootPct)?fmt(m.overshootPct,1)+' %':'无有效平台阶跃'],['稳定时间',finite(m.settlingTime)?fmt(m.settlingTime,3)+' s':'无平台或未稳定'],['平台尾段误差',fmt(m.steadyError,3)],['输出饱和占比',finite(m.saturationRatio)?fmt(m.saturationRatio*100,1)+' %':'未记录']])}</table></div>`;}).join('');
  const context=pidAnalysisContext(),base=context.base,result=buildAiPidAdvice(context.analysis,base,{control:context.known?context.used:null,validation:pidValidation?.analysis});
  const external=result.plant.external; $('pid-validation-status').textContent=pidValidation?('验证文件：'+pidValidation.name+'；'+(external?.valid?'跨文件预测通过':'未通过/模型不可用')+(external?.test?'；自由预测误差 '+fmt(external.test.freeRunNrmse*100,1)+'%':'')+'。原模型未重新拟合。'):'未加载独立验证文件；候选仅适用于本轮已覆盖工况，尚无跨试验改善证据。';
  drawPidModelTrace(context,result);
  for(const k of ['kp','ki','kd']){const input=$('pid-burst-'+k);input.disabled=!!context.known;if(context.known)input.value=String(base[k]);}
  pidOnline.candidates=result.candidates;pidOnline.candidateAxis=context.canApply?context.axis:null;
  cand.innerHTML='<h3>模型候选 · '+(context.axis||'位置环请手动编辑，速度环辨识不适用')+'</h3><p class="note">'+(context.known?'基准来自本轮实际参数，版本 '+context.used.revision:'未记录唯一的实际参数快照，只提供数据诊断。')+'；候选以自身闭环预测比较原参数；所有结果均须实车复测。</p>'+result.candidates.map((c,i)=>'<div class="pid-candidate"><h3>'+escapeHtml(c.name)+'</h3><strong>Kp '+fmt(c.kp,4)+' · Ki '+fmt(c.ki,4)+' · Kd '+fmt(c.kd,5)+'</strong><small>'+escapeHtml(c.reason)+'</small><p>预测表现 '+(c.score===null?'不可计算':c.score+'/100')+' · '+(c.eligible?'通过本地模型检查':'暂不推荐')+'</p><button data-pid-candidate="'+i+'" '+(context.canApply&&c.eligible?'':'disabled')+'>填入 '+(context.axis||'速度环')+' 写入区</button></div>').join('');
  cand.querySelectorAll('[data-pid-candidate]').forEach(b=>b.onclick=()=>usePidCandidate(Number(b.dataset.pidCandidate)));
  const p=result.plant;
  if(ai)ai.innerHTML='<div class="ai-summary"><strong>系统辨识与候选验证 v2</strong><span>'+(p.valid?'独立预测验证通过':'模型不可用于整定')+'</span></div>'+(p.model?'<p>有效模型 K='+fmt(p.model.gain,4)+'，τ='+fmt(p.model.tau,4)+' s，延迟 '+fmt(p.model.deadTime*1000,1)+' ms；最后20%未参与选模的测试段，自由预测归一化误差 '+fmt(p.test.freeRunNrmse*100,1)+'%，单步误差 '+fmt(p.test.oneStepNrmse*100,1)+'%。</p>':'')+(finite(p.controllerReplayNrmse)?'<p>原PID闭环重放归一化误差 '+fmt(p.controllerReplayNrmse*100,1)+'%；±20%增益/时间常数与额外一帧延迟作为压力检查，不代表统计置信区间。</p>':'')+(p.scope?'<p>模型训练方向：'+escapeHtml(p.scope.directions.join('/'))+'；目标范围 '+p.scope.targetRange.map(x=>fmt(x,1)).join(' ~ ')+' pulse/s；使用 '+p.scope.used+' 帧，省略早期 '+p.scope.omitted+' 帧。不代表其他地面/载重适用。</p>':'')+(p.regimes?'<p>测试分组自由预测误差：'+Object.entries(p.regimes).map(([k,v])=>k+' '+v.n+'帧 / '+(v.freeRunNrmse===null?'未覆盖':fmt(v.freeRunNrmse*100,1)+'%')).join('；')+'</p>':'')+(p.controllerAudit?'<p>使用实测轮速重放控制输出，平均偏差 '+fmt(p.controllerAudit.meanAbsOutputError,2)+' / '+context.used.outputMax+'；'+escapeHtml(p.controllerAudit.meaning)+'</p><table>'+tableRows(Object.entries(p.controllerAudit.regimes).map(([name,r])=>[({forward:'正向',reverse:'反向',zero:'零目标'})[name],r.n+' 帧 · 平均误差 '+fmt(r.meanAbsError,1)+' · 饱和 '+fmt(r.saturationRatio*100,1)+'% · 低速停滞 '+fmt(r.stallRatio*100,1)+'%']))+'</table><p class="note">'+escapeHtml(p.controllerAudit.stallMeaning)+'</p>':'')+(result.baselinePrediction?'<p>原参数模型预测表现 '+result.baselinePrediction.score+'/100。该分数与上方本轮实测表现分开，AI不会修改本地分数。</p>':'')+result.warnings.map(w=>'<p class="warning">'+escapeHtml(w)+'</p>').join('')+'<p class="note">低速误差加权；仅在平稳目标段统计阶跃超调和稳定时间。200Hz逐帧对齐，不插值恢复丢失控制周期；100Hz缓存只诊断。最多使用最后12000帧辨识；模型未知、回放不符或无稳健改善时保留原参数。DeepSeek可分析证据和提出小步候选，须通过同一检查才可填表。</p>';
}

function drawPidModelTrace(context,result){
  if(context&&result)pidPlotState={context,result};else if(pidPlotState)({context,result}=pidPlotState);
  if(!context||!result){canvasContext('pid-analysis-speed');canvasContext('pid-analysis-output');return;}
  const metric=context.analysis?.wheels[context.axis?.toLowerCase()];
  const best=result.candidates.find(c=>c.eligible),forecast=best&&result.baselineRuns?simulatePidCandidate(result.plant,best,context.used):null;
  const series=[{samples:metric?.samples||[],key:'target',color:'#fbbf24'},{samples:metric?.samples||[],key:'actual',color:'#38bdf8'}];
  if(result.baselineRuns)series.push({samples:result.baselineRuns[0].samples.map(s=>({...s,t:s.observedTime??s.t})),key:'actual',color:'#c084fc'});
  if(forecast)series.push({samples:forecast.samples.map(s=>({...s,t:s.observedTime??s.t})),key:'actual',color:'#50dfae'});
  for(const [id,output] of [['pid-analysis-speed',false],['pid-analysis-output',true]]){
    const ctx=canvasContext(id);if(!ctx)continue;const {g,w,h}=ctx;
    g.fillStyle='#a9bfd4';g.font='12px sans-serif';g.fillText(output?'本文件实际输出（蓝） · 原参数预测（紫） · 最优通过候选预测（绿）':'本文件目标（黄） · 实测（蓝） · 原参数预测（紫） · 候选预测（绿）',12,18);
    if(!metric?.samples?.length)continue;
    const lines=output?series.filter(s=>s.key==='actual').map(s=>({...s,key:'output'})):series;
    let lo=0,hi=1;for(const line of lines)for(const s of line.samples)if(finite(s[line.key])){lo=Math.min(lo,s[line.key]);hi=Math.max(hi,s[line.key]);}
    const t0=metric.samples[0].t,span=Math.max(.001,metric.samples.at(-1).t-t0),range=Math.max(1,hi-lo);
    g.fillText(fmt(lo,1)+' … '+fmt(hi,1)+' · '+fmt(span,2)+' s',12,h-6);
    for(const line of lines){g.strokeStyle=line.color;g.lineWidth=1.4;g.beginPath();const stride=Math.max(1,Math.ceil(line.samples.length/6000));let connected=false;
      for(let i=0;i<line.samples.length;i+=stride){const chunk=line.samples.slice(i,i+stride).filter(s=>finite(s.t)&&finite(s[line.key]));if(!chunk.length){connected=false;continue;}
        const low=chunk.reduce((a,b)=>a[line.key]<b[line.key]?a:b),high=chunk.reduce((a,b)=>a[line.key]>b[line.key]?a:b),points=stride===1?chunk:[low,high].sort((a,b)=>a.t-b.t);
        for(const s of points){const x=40+(s.t-t0)/span*(w-54),y=30+(hi-s[line.key])/range*(h-60);if(!connected)g.moveTo(x,y);else g.lineTo(x,y);connected=true;}}
      g.stroke();}
  }
}

function loadPidValidation(text,name){
  if(!S.pidBurst)throw new Error('先导入用于辨识的主PID文件');
  const data=parsePidBurstCsv(text);
  if(JSON.stringify(data.rows)===JSON.stringify(S.pidBurst.data.rows))throw new Error('验证文件与主文件相同，不能作为独立试验');
  pidValidation={name,analysis:analyzePidBurst(data,{rate:200,satLimit:1000})};renderPidBurst();
}
function loadPidBurst(text,name='pid_burst.csv') {const data=parsePidBurstCsv(text),rate=Number($('pid-burst-rate')?.value)||100,sat=Number($('pid-burst-sat')?.value)||0,tol=Number($('pid-burst-tol')?.value)||.02;pidValidation=null;S.pidBurst={data,name,recorded:pidRecordedParams(data.comments),analysis:analyzePidBurst(data,{rate,satLimit:sat,tolerance:tol})};renderPidBurst();notify(`已加载独立 PID Burst：${name}。它不会改变 ULog43 任务日志。`);}

const PID_AXES=['FL','FR','RL','RR','X','Y','TH'];
const PID_MOTOR_AXES=PID_AXES.slice(0,4);
const PID_VALUE_KEYS=['kp','ki','kd','integralMax','outputMax','periodMs','ffGain','deadzone','ffMinSpeed','ramp','slew','lpf','kickStep','kickMax','stuckRatio'];
const pidOnline={current:{},revision:null,pending:null,nextId:Date.now()%1000000000,busy:false,previous:{},saved:{},candidates:[],candidateAxis:null};
function decodePidParam(line){
  const f=line.split(',');if(!['PID_PARAM','PID_USED'].includes(f[1])||f.length!==22||!PID_AXES.includes(f[4])||!['READ','SET'].includes(f[5]))return null;
  const a=f.slice(6).map(Number);if(!a.every(finite)||f.slice(6).some(x=>x==='')||![0,1].includes(a[15]))return null;
  const id=Number(f[2]),revision=Number(f[3]);if(!Number.isInteger(id)||!Number.isInteger(revision)||id<0||revision<0)return null;
  return {kind:f[1],id,revision,axis:f[4],operation:f[5],editable:a[15]===1,...Object.fromEntries(PID_VALUE_KEYS.map((k,i)=>[k,a[i]]))};
}
function pidRecordedParams(comments){
  const axes={},mixed=new Set();
  for(const line of comments){const p=decodePidParam(line);if(!p||p.kind!=='PID_USED')continue;const old=axes[p.axis];if(old&&[...PID_VALUE_KEYS,'revision'].some(k=>old[k]!==p[k]))mixed.add(p.axis);axes[p.axis]=p;}
  return {axes,mixed:[...mixed]};
}
function receivePidParamLine(line){
  if(line.startsWith('#BTLOG,PID_USED,')){
    const p=decodePidParam(line);if(p){pidStream.used[p.axis]=p;return true;}return false;
  }
  const isError=line.startsWith('#BTLOG,PID_ERROR,');
  if(!isError&&!line.startsWith('#BTLOG,PID_PARAM,'))return false;
  const pending=pidOnline.pending,id=Number(line.split(',')[2]);
  if(!pending||id!==pending.id)return true;
  if(isError){clearTimeout(pending.timer);pidOnline.pending=null;pending.reject(new Error(line.split(',').slice(3).join(',')));return true;}
  const p=decodePidParam(line);if(!p||p.axis!==pending.axis||p.operation!==pending.operation)return true;
  clearTimeout(pending.timer);pidOnline.pending=null;pidOnline.current[p.axis]=p;pidOnline.revision=p.revision;pending.resolve(p);return true;
}
function pidParamDisconnected(){
  const pending=pidOnline.pending;if(pending){clearTimeout(pending.timer);pending.reject(new Error('蓝牙已断开；写入结果未知，请重新读取。'));}
  pidOnline.pending=null;pidOnline.current={};pidOnline.revision=null;renderPidParams();
}
function renderPidParams(){
  $('pid-param-table').innerHTML='<table><thead><tr><th>车辆当前通道</th><th>Kp</th><th>Ki</th><th>Kd</th><th>周期 ms</th><th>积分限幅</th><th>输出限幅</th><th>版本</th><th>可写</th></tr></thead><tbody>'+PID_AXES.map(axis=>{const p=pidOnline.current[axis];return '<tr><td>'+axis+'</td>'+(['kp','ki','kd','periodMs','integralMax','outputMax','revision'].map(k=>'<td>'+(p?String(p[k]):'未读取')+'</td>').join(''))+'<td>'+(p&&p.revision===pidOnline.revision?(p.editable?'固件最终检查':'否'):'待读取')+'</td></tr>';}).join('')+'</tbody></table>';
  const selected=pidOnline.current[$('pid-param-axis').value];
  $('pid-param-context').textContent=selected?'当前只读配置：周期 '+selected.periodMs+' ms；前馈增益 '+selected.ffGain+'，死区 '+selected.deadzone+'，前馈速度阈值 '+selected.ffMinSpeed+'，目标斜坡 '+selected.ramp+'，输出变化限幅 '+selected.slew+'，测速低通 '+selected.lpf+'，kick '+selected.kickStep+'/'+selected.kickMax+'。':'';
  $('pid-param-write').disabled=pidOnline.busy;$('pid-param-read').disabled=pidOnline.busy;
  for(const id of ['pid-param-axis','pid-param-kp','pid-param-ki','pid-param-kd','pid-param-rollback','pid-param-restore','pid-motors-fill','pid-motors-current','pid-motors-saved','pid-motors-previous','pid-motors-write',...['kp','ki','kd'].map(k=>'pid-motors-'+k),...PID_MOTOR_AXES.flatMap(axis=>['kp','ki','kd'].map(k=>'pid-motor-'+axis+'-'+k))])$(id).disabled=pidOnline.busy;
}
function fillPidParamForm(p){if(!p)return;for(const k of ['kp','ki','kd'])$('pid-param-'+k).value=String(p[k]);}
function selectPidParamAxis(){const axis=$('pid-param-axis').value;fillPidParamForm(pidOnline.current[axis]);renderPidParams();if(S.pidBurst)renderPidBurst();}
function pidExchange(operation,axis,values,revision){
  if(!bt10.device?.gatt?.connected||!bt10.writeCharacteristic)return Promise.reject(new Error('请先直接连接 BT10。'));
  if(pidOnline.pending)return Promise.reject(new Error('上一条参数命令尚未结束。'));
  const id=++pidOnline.nextId;
  return new Promise((resolve,reject)=>{
    const pending={id,axis,operation:operation==='GET'?'READ':operation,resolve,reject,timer:setTimeout(()=>{if(pidOnline.pending!==pending)return;pidOnline.pending=null;pidOnline.current={};renderPidParams();reject(new Error('等待固件应答超时；不会自动重发写入，请重新读取确认实际值。'));},5000)};
    pidOnline.pending=pending;
    const command='@PID '+operation+' '+id+' '+(operation==='SET'?revision+' ':'')+axis+(operation==='SET'?' '+['kp','ki','kd'].map(k=>Number(values[k]).toPrecision(8)).join(' '):'')+'\n';
    bt10WriteText(command).catch(e=>{if(pidOnline.pending!==pending)return;clearTimeout(pending.timer);pidOnline.pending=null;reject(e);});
  });
}
async function withPidOnlineWork(work){
  if(pidOnline.busy)return;pidOnline.busy=true;renderPidParams();
  try{return await work();}catch(e){$('pid-param-status').textContent='操作未确认：'+e.message;notify('在线 PID：'+e.message,true);return null;}
  finally{pidOnline.busy=false;renderPidParams();}
}
function readVehiclePid(){return withPidOnlineWork(async()=>{
  $('pid-param-status').textContent='正在读取车辆全部七组实际参数…';let revision=null;
  for(const axis of PID_AXES){const p=await pidExchange('GET',axis);if(revision!==null&&p.revision!==revision)throw new Error('读取期间参数版本变化，请重新读取。');revision=p.revision;}
  fillPidParamForm(pidOnline.current[$('pid-param-axis').value]);
  $('pid-param-status').textContent='已读取车辆实际参数，版本 '+revision+'。前馈、限幅等只读；请选通道后写入。';
  if(S.pidBurst)renderPidBurst();return pidOnline.current;
});}
function validateOnlinePid(axis,p){const max=PID_AXES.indexOf(axis)<4?[10,8,3]:[20,5,3];return PID_AXES.includes(axis)&&['kp','ki','kd'].every((k,i)=>finite(p[k])&&p[k]>=0&&p[k]<=max[i]);}
function savePidPreset(){try{localStorage.setItem('mecanum-pid-presets-v1',JSON.stringify(pidOnline.saved));}catch(e){notify('参数已写入，但本机保存失败：'+e.message,true);}}
function writeVehiclePid(){return withPidOnlineWork(async()=>{
  const axis=$('pid-param-axis').value,p=Object.fromEntries(['kp','ki','kd'].map(k=>[k,$('pid-param-'+k).value.trim()===''?NaN:Number($('pid-param-'+k).value)]));
  if(!validateOnlinePid(axis,p))throw new Error('参数必须为范围内的非负有限数值：速度环 Kp≤10、Ki≤8，位置环 Kp≤20、Ki≤5，Kd≤3。');
  $('pid-param-status').textContent='正在读取写入前参数并检查停车状态…';
  const before=await pidExchange('GET',axis);if(!before.editable)throw new Error('车辆尚不允许写入；请停止采集、等待缓存上传，并保持停车至少 500 ms。');
  const applied=await pidExchange('SET',axis,p,before.revision);
  const verified=await pidExchange('GET',axis);
  if(applied.revision!==verified.revision||['kp','ki','kd'].some(k=>Math.abs(verified[k]-p[k])>Math.max(1e-7,Math.abs(p[k])*2e-6)))throw new Error('读回值或版本不匹配，禁止视为写入成功。');
  pidOnline.previous[axis]=before;pidOnline.saved[axis]={kp:verified.kp,ki:verified.ki,kd:verified.kd};savePidPreset();fillPidParamForm(verified);
  $('pid-param-status').textContent=axis+' 已写入并读回确认：Kp '+verified.kp+' / Ki '+verified.ki+' / Kd '+verified.kd+'，版本 '+verified.revision+'。仅当前 RAM 生效；其他通道不变。';
  return verified;
});}
function loadPidPreset(previous=false){const axis=$('pid-param-axis').value,p=(previous?pidOnline.previous:pidOnline.saved)[axis];if(!p)return notify('该通道没有'+(previous?'可回滚':'本机保存')+'的参数。',true);fillPidParamForm(p);$('pid-param-status').textContent='已将 '+axis+' 的'+(previous?'上一组':'本机保存')+'参数填入，点击“停车写入并读回确认”后才生效。';}
function pidMotorStatus(axis,text){$('pid-motor-'+axis+'-status').textContent=text;}
function fillMotorPidForms(values){
  for(const axis of PID_MOTOR_AXES){for(const k of ['kp','ki','kd'])$('pid-motor-'+axis+'-'+k).value=String(values[axis][k]);pidMotorStatus(axis,'待写入（编辑值）');}
}
function fillAllMotorPid(){
  if(pidOnline.busy)return;
  const p=Object.fromEntries(['kp','ki','kd'].map(k=>[k,$('pid-motors-'+k).value.trim()===''?NaN:Number($('pid-motors-'+k).value)]));
  if(!validateOnlinePid('FL',p))return notify('统一填入失败：速度环 Kp 0…10、Ki 0…8、Kd 0…3，三项均不能为空。',true);
  fillMotorPidForms(Object.fromEntries(PID_MOTOR_AXES.map(axis=>[axis,p])));
  $('pid-param-status').textContent='同一组手动参数已填入四轮，尚未写入；单轮候选不能视为已适配其他轮子。';
}
function loadMotorPidForms(source){
  if(pidOnline.busy)return;
  const values=source==='current'?pidOnline.current:source==='previous'?pidOnline.previous:pidOnline.saved;
  const missing=PID_MOTOR_AXES.filter(axis=>!values[axis]||!validateOnlinePid(axis,values[axis])||(source==='current'&&values[axis].revision!==pidOnline.revision));
  if(missing.length)return notify('四轮载入未执行：'+missing.join('/')+' 缺少有效值'+(source==='current'?'或版本已过期，请先读取车辆全部 PID。':'。'),true);
  fillMotorPidForms(values);
  $('pid-param-status').textContent='已载入四轮'+({current:'读回值',previous:'各轮上次写入前值',saved:'本机保存值'}[source])+'，尚未写入。';
}
function pidGainsMatch(actual,expected){return ['kp','ki','kd'].every(k=>Math.abs(actual[k]-expected[k])<=Math.max(1e-7,Math.abs(expected[k])*2e-6));}
function writeMotorPids(){return withPidOnlineWork(async()=>{
  // Snapshot and validate all four edits before sending any SET. The existing
  // firmware protocol is single-axis; revisions detect intervening changes.
  const requested=Object.fromEntries(PID_MOTOR_AXES.map(axis=>[axis,Object.fromEntries(['kp','ki','kd'].map(k=>[k,$('pid-motor-'+axis+'-'+k).value.trim()===''?NaN:Number($('pid-motor-'+axis+'-'+k).value)]))]));
  const invalid=PID_MOTOR_AXES.filter(axis=>!validateOnlinePid(axis,requested[axis]));
  if(invalid.length)throw new Error(invalid.join('/')+' 参数无效；四轮均未写入。速度环 Kp 0…10、Ki 0…8、Kd 0…3，三项均不能为空。');
  const before={},confirmed=[];let revision=null,attempted=null,stage='preflight';
  for(const axis of PID_MOTOR_AXES)pidMotorStatus(axis,'本轮未写入');
  try{
    $('pid-param-status').textContent='四轮预检：读取实际参数和停车许可…';
    for(const axis of PID_MOTOR_AXES){
      const p=await pidExchange('GET',axis);before[axis]=p;
      if(!p.editable)throw new Error(axis+' 不允许写入，请停止采集、等待缓存上传，并停车至少 500 ms。');
      if(revision!==null&&revision!==p.revision)throw new Error('预检期间参数版本变化，请重新读取。');revision=p.revision;
    }
    stage='write';
    for(const axis of PID_MOTOR_AXES){
      attempted=axis;pidMotorStatus(axis,'正在写入 / 等待读回');
      $('pid-param-status').textContent='正在写入 '+axis+'；已逐轮确认 '+confirmed.length+'/4…';
      const applied=await pidExchange('SET',axis,requested[axis],revision);
      const verified=await pidExchange('GET',axis);
      if(applied.revision!==((revision+1)>>>0)||verified.revision!==applied.revision||!pidGainsMatch(applied,requested[axis])||!pidGainsMatch(verified,requested[axis]))throw new Error(axis+' 读回值或版本不匹配。');
      revision=verified.revision;confirmed.push(axis);attempted=null;
      pidOnline.previous[axis]=before[axis];pidOnline.saved[axis]={kp:verified.kp,ki:verified.ki,kd:verified.kd};savePidPreset();
      pidMotorStatus(axis,'已逐轮读回；等待整组复核');
    }
    stage='final';
    for(const axis of PID_MOTOR_AXES){const p=await pidExchange('GET',axis);if(p.revision!==revision||!pidGainsMatch(p,requested[axis]))throw new Error('整组复核时 '+axis+' 的值或版本变化。');}
    for(const axis of PID_MOTOR_AXES)pidMotorStatus(axis,'整组读回确认 · 版本 '+revision);
    fillPidParamForm(pidOnline.current[$('pid-param-axis').value]);
    $('pid-param-status').textContent='四轮 PID 已全部写入并在同一版本 '+revision+' 读回确认。仅 RAM 生效，复位后需重新写入；请按原流程进行实车复测。';
    return pidOnline.current;
  }catch(e){
    if(stage==='final')for(const axis of confirmed)pidMotorStatus(axis,'曾逐轮读回；整组最终结果未确认');
    else for(const axis of confirmed)pidMotorStatus(axis,'此前已逐轮读回；整组未完成');
    if(attempted)pidMotorStatus(attempted,'写入未确认；需重新读取实际值');
    // A lost reply can hide a successful write or a device reset. Do not retain
    // stale values as current, retry writes, or silently roll back other motors.
    pidOnline.current={};pidOnline.revision=null;
    throw new Error('四轮操作中止：'+e.message+' '+(confirmed.length?'此前逐轮确认：'+confirmed.join('/')+'。':'尚无逐轮确认。')+(stage==='preflight'?'预检未通过，未发送任何写入。':'已停止后续写入；可能部分生效，请先重新读取全部 PID，再决定后续操作。'));
  }
});}
function usePidCandidate(index){const c=pidOnline.candidates[index],axis=pidOnline.candidateAxis,context=pidAnalysisContext();if(!c?.eligible||!axis||context.axis!==axis||!context.canApply)return; $('pid-param-axis').value=axis;fillPidParamForm(c);$('pid-param-status').textContent='已填入 '+axis+' 候选 '+c.name+'；尚未写入。停车后点击写入并核对回传。';}
function pidAnalysisContext(){
  const axis=$('pid-param-axis').value,inner=PID_AXES.indexOf(axis)<4,recorded=S.pidBurst?.recorded;
  const used=inner?recorded?.axes[axis]:null,known=used&&!recorded.mixed.includes(axis);
  const base=known?{kp:used.kp,ki:used.ki,kd:used.kd}:{kp:Number($('pid-burst-kp').value),ki:Number($('pid-burst-ki').value),kd:Number($('pid-burst-kd').value)};
  const analysis=S.pidBurst?.analysis,selected=inner?axis.toLowerCase():null;
  const metric=selected?analysis?.wheels[selected]:null,canApply=known&&metric?.n>=20&&metric.maxTarget>1e-6;
  return {axis:inner?axis:null,base,known,canApply,used,sourceAnalysis:analysis,analysis:analysis&&selected?{...analysis,wheels:Object.fromEntries(WHEELS.map(w=>[w,w===selected?analysis.wheels[w]:{n:0}]))}:analysis};
}

const PID_HEADER='timestamp_ms,fl_target,fl_actual,fl_output,fr_target,fr_actual,fr_output,rl_target,rl_actual,rl_output,rr_target,rr_actual,rr_output';
const pidStream={live:[],stored:[],liveCount:0,used:{},lastSeq:null,gaps:0,crc:0,expected:null,stats:'',lastDraw:0,complete:false};
function pidCrc(bytes){let crc=65535;for(const b of bytes){crc^=b<<8;for(let i=0;i<8;i++)crc=((crc&32768)?(crc<<1)^0x1021:crc<<1)&65535;}return crc;}
function createPidWireDecoder(onText){
  let pending=[],resync=false;
  return {feed(bytes){pending.push(...bytes);
    while(pending.length){
      if(resync){let at=-1;for(let i=0;i<pending.length;i++){if((pending[i]===165&&pending[i+1]===90)||String.fromCharCode(...pending.slice(i,i+7))==='#BTLOG,'){at=i;break;}}if(at<0){pending=pending.slice(-6);return;}pending.splice(0,at);resync=false;}
      if(pending[0]===165){
        if(pending.length<2)return;
        if(pending[1]!==90){pending.shift();continue;}
        if(pending.length<36)return;
        const f=pending.slice(0,36),crc=pidCrc(f.slice(2,34));
        if(f[2]!==1||![1,2].includes(f[3])||crc!==(f[34]|f[35]<<8)){pidStream.crc++;pending.shift();resync=true;continue;}
        pending.splice(0,36);
        const seq=f[4]|f[5]<<8,ms=(f[6]+f[7]*256+f[8]*65536+f[9]*16777216)>>>0,row=[ms];
        for(let i=0;i<12;i++){let v=f[10+i*2]|f[11+i*2]<<8;if(v&32768)v-=65536;row.push(v/10);}
        onText((f[3]===1?'#BTLOG,PID_LIVE,'+seq+',':'#BTLOG,PID,')+row.join(',')+'\n');
        continue;
      }
      const nl=pending.indexOf(10),magic=pending.indexOf(165);
      if(magic>=0&&(nl<0||magic<nl)){pending.splice(0,magic);continue;}
      if(nl<0){if(pending.length>4096)pending=[];return;}
      onText(String.fromCharCode(...pending.splice(0,nl+1)));
    }
  },reset(){pending=[];resync=false;}};
}
function resetPidStream(){Object.assign(pidStream,{live:[],stored:[],liveCount:0,used:{},lastSeq:null,gaps:0,crc:0,expected:null,stats:'',lastDraw:0,complete:false});}
function encodePidUsed(p){return '#BTLOG,PID_USED,0,'+p.revision+','+p.axis+',READ,'+PID_VALUE_KEYS.map(k=>p[k]).join(',')+','+(p.editable?1:0)+'\n';}
function pidCsv(){return Object.values(pidStream.used).map(encodePidUsed).join('')+PID_HEADER+'\n'+pidStream.stored.map(r=>r.join(',')).join('\n')+'\n'+(pidStream.stats?pidStream.stats+'\n':'');}
function receivePidLine(line){
  if(line.startsWith('#BTLOG,PID_ARMED')){resetPidStream();$('pid-live-status').textContent='已开始：持续 200 Hz 实时；100 Hz 循环保留最近 20 秒。手动停止或整车日志结束后冻结缓存。';return true;}
  if(line.startsWith('#BTLOG,PID_REJECTED')){$('pid-live-status').textContent='启动被拒绝：请停车、等待开机至少 2 秒，并等待上次上传完成。';return true;}
  if(line.startsWith('#BTLOG,PID_BEGIN')){pidStream.stored=[];pidStream.complete=false;return true;}
  const live=line.startsWith('#BTLOG,PID_LIVE,'),stored=line.startsWith('#BTLOG,PID,');
  if(live||stored){
    const a=line.split(',').slice(2).map(Number);let row=a;
    if(live){const seq=a.shift();row=a;if(pidStream.lastSeq!==null)pidStream.gaps+=Math.max(0,((seq-pidStream.lastSeq)&65535)-1);pidStream.lastSeq=seq;}
    if(row.length!==13||!row.every(finite))return true;
    if(live)pidStream.liveCount++;
    const dest=live?pidStream.live:pidStream.stored;dest.push(row);if(dest.length>(live?4000:2000))dest.shift();
    const now=performance.now();if(now-pidStream.lastDraw>=100){pidStream.lastDraw=now;renderPidStream();}
    return true;
  }
  if(line.startsWith('#BTLOG,PID_STATS,')){pidStream.stats=line;const m=line.match(/count=(\d+)/);pidStream.expected=m?Number(m[1]):null;return true;}
  if(line==='#BTLOG,PID_END'){
    pidStream.complete=pidStream.expected!==null&&pidStream.expected===pidStream.stored.length;
    if(pidStream.stored.length)loadPidBurst(pidCsv(),'停车上传_PID_100Hz.csv');
    renderPidStream();return true;
  }
  return false;
}
function renderPidStream(){
  const stored=pidStream.stored,rows=stored.length?stored:pidStream.live,wheel=$('pid-live-wheel').value||'fl',wi=WHEELS.indexOf(wheel),base=1+Math.max(0,wi)*3;
  $('pid-live-status').textContent='实时累计 '+pidStream.liveCount+' 帧（图表保留 '+pidStream.live.length+' 帧，完整实收在仓库） · 实时缺口 '+pidStream.gaps+' · CRC 错误 '+pidStream.crc+' · 停车上传 '+stored.length+'/2000 帧 / 应收 '+pidStream.expected+' · '+(pidStream.complete?'上传完成':'采集/等待上传，断线时可能不完整')+' · '+pidStream.stats;
  for(const [id,cols,colors,title] of [['pid-live-speed',[base,base+1],['#fbbf24','#38bdf8'],'目标速度（黄） / 实测速度（蓝），pulse/s'],['pid-live-output',[base+2],['#c084fc'],'实际控制输出（含前馈、PID、kick 与限幅）']]){
    const ctx=canvasContext(id);if(!ctx)continue;const {g,w,h}=ctx;
    g.fillStyle='#a9bfd4';g.font='12px sans-serif';g.fillText(title,12,18);if(rows.length<2)continue;
    const t0=rows[0][0],span=Math.max(1,rows.at(-1)[0]-t0),values=rows.flatMap(r=>cols.map(c=>r[c])),lo=Math.min(0,...values),hi=Math.max(1,...values),range=Math.max(1,hi-lo);
    g.fillText(fmt(lo,1)+' … '+fmt(hi,1)+' · '+fmt(span/1000,2)+' s',12,h-6);
    cols.forEach((col,k)=>{g.strokeStyle=colors[k];g.lineWidth=1.5;g.beginPath();rows.forEach((r,i)=>{const x=40+(r[0]-t0)/span*(w-54),y=30+(hi-r[col])/range*(h-60);if(i===0||r[0]-rows[i-1][0]>30)g.moveTo(x,y);else g.lineTo(x,y);});g.stroke();});
  }
}
function downloadPidStream(){if(!pidStream.stored.length){notify('尚未收到停车上传的 PID 数据。',true);return;}const a=document.createElement('a'),url=URL.createObjectURL(new Blob([pidCsv()],{type:'text/csv;charset=utf-8'}));a.href=url;a.download='pid_100Hz_'+Date.now()+'.csv';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);}
async function sendPidCommand(command){
  if(command==='START'&&pidOnline.busy){notify('参数操作未结束，请等待读回确认后再开始采集。',true);return;}
  if(!bt10.writeCharacteristic){notify('请先直接连接 BT10；电脑日志源只接收数据。',true);return;}
  try{await bt10WriteText('@PID '+command+'\n');$('pid-live-status').textContent='命令已发送，等待车辆确认；不会让车辆运动或修改 PID。';}catch(e){notify('PID 命令发送失败：'+e.message,true);}
}
function pidWireSelfTest(){
  if(bt10.device||realtime.socket){notify('请断开实时连接后运行模拟自测，避免混入实车数据。',true);return;}
  resetPidStream();
  const decoder=createPidWireDecoder(text=>text.trim().split(/\r?\n/).forEach(receivePidLine));
  const encode=(kind,seq,ms)=>{
    const f=new Uint8Array(36),v=new DataView(f.buffer);f[0]=165;f[1]=90;f[2]=1;f[3]=kind;v.setUint16(4,seq,true);v.setUint32(6,ms,true);
    for(let w=0;w<4;w++){const target=100+20*Math.sin(ms/800),actual=target*(1-Math.exp(-ms/150));[target,actual,actual*2].forEach((x,i)=>v.setInt16(10+(w*3+i)*2,Math.round(x*10),true));}
    v.setUint16(34,pidCrc(f.slice(2,34)),true);return f;
  };
  receivePidLine('#BTLOG,PID_ARMED');
  for(let i=0;i<6000;i++){const f=encode(1,i+1,(i+1)*5);decoder.feed(f.slice(0,7));decoder.feed(f.slice(7,19));decoder.feed(f.slice(19));}
  receivePidLine('#BTLOG,PID_BEGIN,'+PID_HEADER);
  for(let i=0;i<2000;i++)decoder.feed(encode(2,i,10000+(i+1)*10));
  receivePidLine('#BTLOG,PID_STATS,count=2000,live_drop=0,clipped=0,overrun=0');
  receivePidLine('#BTLOG,PID_END');
  notify('网页协议自测完成：6000 帧实时 / 最近 20 秒 2000 帧停车上传，模拟连续 30 秒；未经过真实 BT10。');
}
function namedChannels(header) {
  const out={...DEFAULT_CHANNELS},norm=s=>String(s||'').toLowerCase().replace(/[\s()\[\]°/.-]+/g,'_');
  const aliases={time:['time','timestamp','time_ms','ms'],phase:['phase','state_phase'],x:['x','x_pos','position_x'],y:['y','y_pos','position_y'],yaw:['yaw','theta','heading'],imu:['imu_yaw'],xt:['x_target','target_x'],yt:['y_target','target_y'],yawt:['yaw_target','theta_target'],vx:['vx','vx_actual'],vy:['vy','vy_actual'],wt:['omega_target','w_target','wz_target'],wa:['omega_actual','w_actual','wz_actual'],speedTarget:['speed_target','v_target'],lateral:['lateral_error','elat'],yawError:['yaw_error','heading_error'],flt:['fl_target','wheel_fl_target'],fla:['fl_actual','wheel_fl_speed','wheel_fl_actual'],frt:['fr_target','wheel_fr_target'],fra:['fr_actual','wheel_fr_speed','wheel_fr_actual'],rlt:['rl_target','wheel_rl_target'],rla:['rl_actual','wheel_rl_speed','wheel_rl_actual'],rrt:['rr_target','wheel_rr_target'],rra:['rr_actual','wheel_rr_speed','wheel_rr_actual']};
  const names=header.map(norm);for(const [key,list] of Object.entries(aliases)){const i=names.findIndex(n=>list.includes(n));if(i>=0)out[key]=i;else if(key!=='time'&&key!=='phase'&&key!=='x'&&key!=='y'&&key!=='yaw')out[key]=null;}
  return out;
}
function qualityReport(parsed,time) {
  const rows=parsed.rows,deltas=[];let repeated=0,backward=0,missing=0,invalid=0;
  rows.forEach((r,i)=>{for(let c=0;c<r.length;c++)if(!finite(r[c]))missing++;if(i){const d=time.times[i]-time.times[i-1];if(d<=0){repeated++;if(d<0)backward++;}else deltas.push(d);}});
  const sorted=deltas.slice().sort((a,b)=>a-b),q=(p)=>sorted.length?sorted[Math.min(sorted.length-1,Math.floor((sorted.length-1)*p))]:NaN;
  const max=sorted.length?sorted[sorted.length-1]:NaN;
  return {format:parsed.source==='converted'?'ULog26 转换为 43 列':parsed.source==='named'?'带字段名日志':'原生/未知 ULog43',totalLines:parsed.totalLines,validRows:rows.length,comments:parsed.comments,headers:parsed.headers,invalidRows:parsed.skipped,bad:parsed.bad,missing,repeated,backward,medianDt:time.medianDt,p95:q(.95),maxDt:max,gaps:time.gaps,duration:time.times.at(-1)||0,source:parsed.source};
}
function timeline(rows,cfg=config) {
  if(cfg.channels.time===null)return {times:rows.map((_,i)=>i*cfg.dt),medianDt:cfg.dt,gaps:0,fallback:true};
  const times=[],deltas=[];let previous=NaN,offset=0,first=0;
  rows.forEach((row,i)=>{
    const stamp=value(row,'time',cfg);
    if(!finite(stamp)||stamp<0)throw new Error(`第 ${i+1} 帧时间戳无效。检查 ms 通道；无时间戳时可在“通道与标定”留空时间通道。`);
    if(i && stamp<previous && previous-stamp>2147483648)offset+=4294967296;
    const t=stamp+offset;if(i===0)first=t;
    const relative=(t-first)/1000;
    if(i && relative<=times[i-1])throw new Error(`第 ${i+1} 帧时间戳重复或倒退，无法可靠回放。请分开不同开机日志，或核对时间通道。`);
    times.push(relative);if(i)deltas.push(relative-times[i-1]);previous=stamp;
  });
  const sorted=deltas.slice().sort((a,b)=>a-b),medianDt=sorted.length?sorted[Math.floor(sorted.length/2)]:cfg.dt;
  return {times,medianDt,gaps:deltas.filter(d=>d>medianDt*3).length,fallback:false};
}
function lowerBound(values,x) {let a=0,b=values.length;while(a<b){const m=(a+b)>>1;if(values[m]<x)a=m+1;else b=m;}return a;}
function frameAt(t) {return clamp(lowerBound(S.times,t+1e-9)-1,0,Math.max(0,S.times.length-1));}
function nearestFrame(t) {const i=clamp(lowerBound(S.times,t),0,S.times.length-1);return i>0&&Math.abs(S.times[i-1]-t)<Math.abs(S.times[i]-t)?i-1:i;}
function duration() {return S.times.length?S.times[S.times.length-1]:0;}
function phaseName(row) {const v=value(row,'phase');return ({0:'启动等待',1:'平移',2:'转弯',4:'任务完成后静止'})[v] || (finite(v)?'阶段 '+v:'阶段未记录');}
function detectSegments() {
  const out=[];if(!S.rows.length)return out;let start=0,last=phaseName(S.rows[0]);
  for(let i=1;i<S.rows.length;i++){const label=phaseName(S.rows[i]);if(label!==last){out.push({a:start,b:i-1,label:last,kind:last,source:'固件阶段'});start=i;last=label;}}
  out.push({a:start,b:S.rows.length-1,label:last,kind:last,source:'固件阶段'});
  // Keep short phases visible, but merge consecutive tiny noise changes into the neighbor.
  return out.map((s,i)=>({...s,duration:S.times[s.b]-S.times[s.a],target:`${fmt(value(S.rows[s.b],'xt'))}, ${fmt(value(S.rows[s.b],'yt'))} m`}));
}
function segmentWindow(s) {if(!s)return;S.selectedSegment=s;setWindow(S.times[s.a],Math.max(S.times[s.a]+S.medianDt,S.times[s.b]));go(s.a);}
function notify(message,warn=false) {$('notice').textContent=message;$('notice').classList.toggle('warn',warn);}
function load(text,name='日志.csv') {
  try {
    const parsed=parse(text),importConfig=parsed.source==='named'?{...config,channels:{...config.channels,...namedChannels(parsed.header||[])}}:config,time=timeline(parsed.rows,importConfig); // Validate before replacing a working log.
    pausePlayback();if(parsed.source==='named')config.channels=importConfig.channels;else config.channels={...DEFAULT_CHANNELS};S.rows=parsed.rows;S.name=name;Object.assign(S,time);S.quality=qualityReport(parsed,time);S.cur=0;
    rebuild();S.wave.start=0;S.wave.end=duration()||config.dt;
    $('slider').max=S.rows.length-1;fitMap();updateWindow();renderEvents();renderQuality();render();
    const warnings=[];
    if(parsed.skipped)warnings.push(`跳过 ${parsed.skipped} 行异常记录（原始行号已保留）`);
    if(time.fallback)warnings.push(`没有使用时间戳，按 ${config.dt}s / 帧估算时间`);
    if(time.gaps)warnings.push(`${time.gaps} 处时间间隔超过中位数的 3 倍，轨迹和波形断线显示`);
    const logged=S.rows.map(r=>value(r,'wa'));
    if(logged.every(v=>v===0)&&S.samples.some(a=>Math.abs(a.wheelOmega)>.05))warnings.push('日志角速度通道全为 0，可能是旧日志占位；建议使用四轮运动学');
    if(S.samples.some(a=>![a.vx,a.vy,a.omega].every(finite)))warnings.push('部分速度数据缺失，已保留为空，不以 0 替代');
    if(S.quality.format==='ULog26 转换为 43 列')warnings.push('检测为转换日志；转换补零字段按未记录处理');
    notify(`已加载 ${name}：${S.quality.format}。${warnings.join('；') || '请确认日志来自同一套固件。'}`,warnings.length>0);
    activateTab('transport');
  } catch(error) {notify('未导入：'+error.message,true);}
}
// Optional live input from the Windows BT10 collector. The collector forwards
// newline-delimited #BTLOG metadata and ULog43 CSV rows over a local WebSocket.
const realtime={socket:null,buffer:'',pending:[],flushRequest:0,ended:false};
// Serialize writes; store each chunk once instead of rewriting session text.
const archive={db:null,session:null,pending:'',flushTimer:null,lastId:null,work:Promise.resolve(),error:''};
function archiveQueue(task){
  const result=archive.work.then(task);
  archive.work=result.catch(e=>{archive.error=String(e.message||e);notify('日志保存失败：'+archive.error+'；待写数据保留在内存，请保持网页打开并重试。',true);});
  return archive.work.then(()=>result.catch(()=>null));
}
function archiveRequest(req){return new Promise((resolve,reject)=>{req.onsuccess=()=>resolve(req.result);req.onerror=()=>reject(req.error);});}
function archiveTransaction(db,names,write){return new Promise((resolve,reject)=>{
  const tx=db.transaction(names,'readwrite');tx.oncomplete=()=>resolve();tx.onerror=()=>reject(tx.error);tx.onabort=()=>reject(tx.error||new Error('存储事务已中止'));
  try{write(tx);}catch(e){tx.abort();reject(e);}
});}
function archiveOpen(){return new Promise((resolve,reject)=>{
  if(archive.db)return resolve(archive.db);
  const r=indexedDB.open('mecanum-log-archive',2);
  r.onupgradeneeded=()=>{const db=r.result;if(!db.objectStoreNames.contains('logs'))db.createObjectStore('logs',{keyPath:'id',autoIncrement:true});if(!db.objectStoreNames.contains('chunks'))db.createObjectStore('chunks',{keyPath:['logId','part']}).createIndex('log','logId');};
  r.onblocked=()=>notify('日志仓库升级等待中，请关闭其他打开的旧版调试网页。',true);
  r.onsuccess=()=>{archive.db=r.result;archive.db.onversionchange=()=>{archive.db.close();archive.db=null;};resolve(archive.db);};r.onerror=()=>reject(r.error);
});}
async function archiveStart(){
  const db=await archiveOpen(),now=new Date(),rec={format:2,start:now.toISOString(),name:'BT10_'+now.toISOString().replace(/[:.]/g,'-'),frames:0,liveFrames:0,pid:false,parts:0,ended:false};
  for(const line of archive.pending.split(/\r?\n/))archiveCountLine(rec,line);
  await archiveTransaction(db,['logs'],tx=>{const r=tx.objectStore('logs').add(rec);r.onsuccess=()=>{rec.id=r.result;};});
  archive.session={id:rec.id,rec};archive.lastId=rec.id;
  clearInterval(archive.flushTimer);archive.flushTimer=setInterval(archiveFlush,1000);archiveRender();
}
async function archiveFlushPending(){
  if(!archive.pending)return;
  if(!archive.session)await archiveStart();
  const session=archive.session,chunk=archive.pending,rec={...session.rec,parts:session.rec.parts+1},db=await archiveOpen();
  await archiveTransaction(db,['logs','chunks'],tx=>{tx.objectStore('chunks').put({logId:session.id,part:session.rec.parts,text:chunk});tx.objectStore('logs').put(rec);});
  archive.pending=archive.pending.slice(chunk.length);session.rec=rec;archive.error='';
}
function archiveFlush(){return archiveQueue(archiveFlushPending);}
function archiveCountLine(rec,line){
  rec.frames+=(line.split(',').length>=43?1:0);
  if(line.startsWith('#BTLOG,PID_LIVE,')){rec.liveFrames++;rec.pid=true;}
  if(line.startsWith('#BTLOG,PID_BEGIN')){rec.pid=true;rec.pidCache=true;}
}
function archiveLine(line){return archiveQueue(async()=>{
  // Retain lines even when opening/upgrading the DB fails; flush can retry.
  archive.pending+=line;
  if(!archive.session)await archiveStart();else archiveCountLine(archive.session.rec,line);
  if(archive.pending.length>=65536)await archiveFlushPending();
});}
function archiveFinish(ended){return archiveQueue(async()=>{
  await archiveFlushPending();if(!archive.session)return;
  const rec={...archive.session.rec,ended,end:new Date().toISOString()},db=await archiveOpen();
  await archiveTransaction(db,['logs'],tx=>tx.objectStore('logs').put(rec));
  clearInterval(archive.flushTimer);archive.session=null;archiveRender();return rec.id;
});}
async function archiveList(){const db=await archiveOpen();return (await archiveRequest(db.transaction('logs').objectStore('logs').getAll())).reverse();}
async function archiveParts(id){
  const db=await archiveOpen(),rec=await archiveRequest(db.transaction('logs').objectStore('logs').get(id));if(!rec)return null;
  const parts=rec.format===2?(await archiveRequest(db.transaction('chunks').objectStore('chunks').index('log').getAll(id))).sort((a,b)=>a.part-b.part).map(x=>x.text):[rec.text||''];
  return {rec,parts};
}
async function archiveRender(){const box=$('archive-list');if(!box)return;try{
  const list=await archiveList();if(!list.length){box.textContent='暂无日志；连接 BT10 后会自动分块保存。';return;}
  box.innerHTML=list.map(x=>'<div class="archive-item"><div class="archive-meta"><b>'+x.name+'</b><small>'+x.start+' · 整车 '+x.frames+' 帧 · 实时 PID '+(x.liveFrames||0)+' 帧 · '+(x.ended?'已完成':'未完成/断线保留')+'</small></div><div class="archive-actions"><button data-archive-view="'+x.id+'">查看</button><button data-archive-download="'+x.id+'">下载会话</button>'+((x.pidCache||x.text?.includes('#BTLOG,PID_BEGIN'))?'<button data-archive-pid="'+x.id+'">缓存 PID 分析</button>':'')+((x.pid||x.text?.includes('#BTLOG,PID_LIVE,'))?'<button data-archive-live="'+x.id+'">下载实时 PID</button>':'')+'<button data-archive-delete="'+x.id+'">删除</button></div></div>').join('');
  box.querySelectorAll('[data-archive-view]').forEach(b=>b.onclick=()=>archiveLoad(Number(b.dataset.archiveView)));
  box.querySelectorAll('[data-archive-download]').forEach(b=>b.onclick=()=>archiveDownload(Number(b.dataset.archiveDownload)));
  box.querySelectorAll('[data-archive-live]').forEach(b=>b.onclick=()=>downloadPidLive(Number(b.dataset.archiveLive)));
  box.querySelectorAll('[data-archive-pid]').forEach(b=>b.onclick=async()=>{const rec=await archiveGet(Number(b.dataset.archivePid));if(rec){try{loadPidBurst(rec.text,rec.name+'_PID.csv');activateTab('pid');}catch(e){notify('尚无停车缓存，请先停止采集并等待上传。',true);}}});
  box.querySelectorAll('[data-archive-delete]').forEach(b=>b.onclick=()=>archiveDelete(Number(b.dataset.archiveDelete)));
}catch(e){box.textContent='日志仓库不可用：'+e.message;}}
function archiveGet(id){return archiveQueue(async()=>{await archiveFlushPending();const x=await archiveParts(id);return x?{...x.rec,text:x.parts.join('')}:null;});}
async function archiveLoad(id){const x=await archiveGet(id);if(x?.text)load(x.text,x.name+'.csv');}
function downloadParts(parts,name){const a=document.createElement('a'),url=URL.createObjectURL(new Blob(parts,{type:'text/csv;charset=utf-8'}));a.href=url;a.download=name;a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);}
function archiveDownload(id){return archiveQueue(async()=>{await archiveFlushPending();const x=await archiveParts(id??archive.session?.id??archive.lastId);if(!x)return false;downloadParts(x.parts,x.rec.name+'.csv');return true;});}
function pidLiveParts(parts){
  const result=[PID_HEADER+'\n'];let remainder='';
  const convert=line=>{if(line.startsWith('#BTLOG,PID_USED,'))return line+'\n';if(!line.startsWith('#BTLOG,PID_LIVE,'))return '';const a=line.split(',').slice(3);return a.length===13&&a.every(x=>x!==''&&finite(Number(x)))?a.join(',')+'\n':'';};
  for(const part of parts){const lines=(remainder+part).split(/\r?\n/);remainder=lines.pop()||'';const rows=lines.map(convert).join('');if(rows)result.push(rows);}
  if(remainder){const row=convert(remainder);if(row)result.push(row);}
  return result;
}
function downloadPidLive(id){return archiveQueue(async()=>{await archiveFlushPending();const x=await archiveParts(id??archive.session?.id??archive.lastId);if(!x)return notify('尚无已保存的 PID 实时数据。',true);const parts=pidLiveParts(x.parts);if(parts.length===1)return notify('该会话没有实时 PID 数据。',true);downloadParts(parts,x.rec.name+'_PID_live_200Hz.csv');});}
function archiveDelete(id){return archiveQueue(async()=>{
  if(archive.session?.id===id)return notify('请先结束或断开当前记录，再删除。',true);
  const db=await archiveOpen();await archiveTransaction(db,['logs','chunks'],tx=>{tx.objectStore('logs').delete(id);const r=tx.objectStore('chunks').index('log').openCursor(id);r.onsuccess=()=>{const c=r.result;if(c){c.delete();c.continue();}};});archiveRender();
});}
function archiveClear(){return archiveQueue(async()=>{if(archive.session)return notify('请先结束或断开当前记录，再清空仓库。',true);const db=await archiveOpen();await archiveTransaction(db,['logs','chunks'],tx=>{tx.objectStore('logs').clear();tx.objectStore('chunks').clear();});archive.lastId=null;archiveRender();});}
const bt10={device:null,characteristic:null,writeCharacteristic:null,decoder:null,ended:false,txWork:Promise.resolve()};
function bt10WriteText(text){
  const ch=bt10.writeCharacteristic;
  const result=bt10.txWork.then(async()=>{if(!ch||bt10.writeCharacteristic!==ch)throw new Error('蓝牙写入连接已变化。');const bytes=new TextEncoder().encode(text);for(let i=0;i<bytes.length;i+=20)await ch.writeValueWithResponse(bytes.slice(i,i+20));});
  bt10.txWork=result.catch(()=>{});return result;
}
const serialEcho={port:null,reader:null,writer:null,running:false};
const localSim={running:false,timer:null};
const BT10_SERVICE='0000ffe0-0000-1000-8000-00805f9b34fb';
const BT10_NOTIFY='0000ffe1-0000-1000-8000-00805f9b34fb';
const BT10_WRITE='0000ffe2-0000-1000-8000-00805f9b34fb';
function btStatus(text,kind=''){ $('bt-connect').textContent=kind==='connected'?'断开 BT10':'直接连接 BT10'; $('ws-status').textContent=text; }
function downloadBt10Log(){return archiveDownload();}
function bt10DecodedText(text){receiveRealtimeText(text);if(text.trim()==='#BTLOG,END'&&!bt10.ended){bt10.ended=true;downloadBt10Log().then(ok=>btStatus(ok?'BT10 日志已结束，已生成会话下载':'BT10 日志已结束；保存未成功，请查看提示并重试','connected'));}}
function bt10Notification(event){const v=event.target.value;bt10.wire.feed(new Uint8Array(v.buffer,v.byteOffset,v.byteLength));}
function bt10TestRows(){ const rows=['#BTLOG,BEGIN']; for(let i=0;i<20;i++){const t=i*100,x=i*.01,y=Math.sin(i*.2)*.1; const a=[0,1,1,0,0,t,0,0,1,0,.5,.2,.5,.2,0,0,i*.5,i*.5,0,.5385,.5385,x,y,i*.5,0,0,0,100,100,100,100,100,x,y,x,100,100,100,20,0,24,12,0]; rows.push(a.map(v=>typeof v==='number'&&v%1?v.toFixed(4):String(v)).join(','));} rows.push('#BTLOG,END'); return rows; }
async function sendBt10SelfTest(){ if(!bt10.writeCharacteristic){btStatus('请先连接 BT10','error');return;} const button=$('bt-selftest'); button.disabled=true; try{for(const line of bt10TestRows()){await bt10WriteText(line+'\r\n'); await new Promise(resolve=>setTimeout(resolve,60));} btStatus('自测已发送，等待回传','connected'); notify('BT10 自测已发送；需要 UART 回传或回环才能在网页收到测试曲线。');}catch(e){btStatus('自测发送失败：'+e.message,'error');}finally{button.disabled=false;} }
function toggleLocalSim(){
  if(localSim.running){localSim.running=false;clearInterval(localSim.timer);localSim.timer=null;$('local-sim').textContent='本地模拟';realtimeStatus('本地模拟已停止');return;}
  resetRealtimeSession(); localSim.running=true; $('local-sim').textContent='停止模拟'; realtimeStatus('本地模拟运行中','connected');
  const rows=bt10TestRows().filter(line=>!line.startsWith('#')); let i=0;
  localSim.timer=setInterval(()=>{if(i>=rows.length){clearInterval(localSim.timer);localSim.timer=null;localSim.running=false;$('local-sim').textContent='本地模拟';receiveRealtimeText('#BTLOG,END\r\n');realtimeStatus('本地模拟完成','connected');return;}receiveRealtimeText(rows[i++]+'\r\n');},80);
}
async function serialEchoLoop(){ while(serialEcho.running&&serialEcho.port?.readable){ serialEcho.reader=serialEcho.port.readable.getReader(); try{ while(serialEcho.running){ const {value,done}=await serialEcho.reader.read(); if(done)break; if(value?.length){ await serialEcho.writer.write(value); } } } finally { serialEcho.reader.releaseLock(); serialEcho.reader=null; } } }
async function toggleSerialEcho(){
  if(serialEcho.running){ serialEcho.running=false; try{await serialEcho.reader?.cancel();}catch(_){ } try{serialEcho.writer?.releaseLock();}catch(_){ } try{await serialEcho.port?.close();}catch(_){ } serialEcho.port=null; serialEcho.writer=null; $('serial-echo').textContent='串口自动回显'; btStatus('串口回显已停止'); return;
  }
  if(!navigator.serial){btStatus('当前浏览器不支持 Web Serial','error');return;}
  try{ const port=await navigator.serial.requestPort(); await port.open({baudRate:115200,dataBits:8,stopBits:1,parity:'none',bufferSize:4096}); serialEcho.port=port; serialEcho.writer=port.writable.getWriter(); serialEcho.running=true; $('serial-echo').textContent='停止串口回显'; btStatus('串口回显运行中','connected'); notify('网页已打开 BT10 串口，收到数据会自动原样回传。'); serialEchoLoop().catch(e=>btStatus('串口回显失败：'+e.message,'error')); }catch(e){btStatus('串口打开失败：'+e.message,'error');}
}
async function fullSelfTest(){
  const button=$('full-selftest'); button.disabled=true;
  try{
    if(!navigator.bluetooth||!navigator.serial) throw new Error('请使用 Chrome/Edge，并通过 localhost 或 HTTPS 打开网页');
    if(!bt10.device) await connectBt10();
    if(!bt10.device) throw new Error('BT10 BLE 未连接');
    if(!serialEcho.running) await toggleSerialEcho();
    if(!serialEcho.running) throw new Error('串口自动回显未启动');
    await sendBt10SelfTest();
    notify('一键自测已启动：等待 43 列日志回传和 END。');
  }catch(e){btStatus('一键自测失败：'+e.message,'error');}
  finally{button.disabled=false;}
}
async function connectBt10(){
  if(bt10.device){ bt10.device.gatt?.disconnect(); bt10.device=null; bt10.characteristic=null; bt10.writeCharacteristic=null; $('bt-selftest').disabled=true; btStatus('未连接'); return; }
  if(!navigator.bluetooth){ btStatus('当前浏览器不支持 Web Bluetooth','error'); return; }
  try{
    btStatus('选择 BT10…');
    const device=await navigator.bluetooth.requestDevice({filters:[{services:[BT10_SERVICE]}],optionalServices:[BT10_SERVICE]});
    let server,service,ch,lastError;
    for(let attempt=0;attempt<3;attempt++){
      try{
        if(attempt) await new Promise(resolve=>setTimeout(resolve,700));
        server=await device.gatt.connect();
        service=await server.getPrimaryService(BT10_SERVICE);
        ch=await service.getCharacteristic(BT10_NOTIFY);
        break;
      }catch(error){ lastError=error; try{device.gatt?.disconnect();}catch(_){ } if(attempt===2)throw lastError; }
    }
    bt10.device=device; bt10.characteristic=ch; bt10.writeCharacteristic=await service.getCharacteristic(BT10_WRITE); bt10.decoder=new TextDecoder(); bt10.wire=createPidWireDecoder(bt10DecodedText); resetPidStream(); bt10.ended=false;pidParamDisconnected(); resetRealtimeSession();
    ch.addEventListener('characteristicvaluechanged',bt10Notification); await ch.startNotifications();
    device.addEventListener('gattserverdisconnected',()=>{if(bt10.device===device){bt10.device=null;bt10.characteristic=null;bt10.writeCharacteristic=null;$('bt-selftest').disabled=true;pidParamDisconnected();archiveFinish(false);btStatus('BT10 已断开，日志已保留','error');}});
    $('bt-selftest').disabled=false; btStatus('BT10 已连接，等待日志','connected'); notify('BT10 已连接，日志将实时显示并在 END 后自动下载。');readVehiclePid();
  }catch(e){ bt10.device=null; btStatus('BT10 连接失败：'+e.message,'error'); }
}
function realtimeStatus(text,kind='') { const box=document.querySelector('.realtime-connect'); $('ws-status').textContent=text; box?.classList.toggle('connected',kind==='connected'); box?.classList.toggle('error',kind==='error'); }
function resetRealtimeSession() {
  pausePlayback(); config.channels={...DEFAULT_CHANNELS}; S.rows=[]; S.name='BT10 实时日志'; S.cur=0;
  Object.assign(S,{times:[],samples:[],points:[],quality:null,segments:[],selectedSegment:null,medianDt:config.dt,gaps:0});
  $('slider').max=0; rebuild(); fitMap(); updateWindow(); renderEvents(); renderQuality(); render();
}
function queueRealtimeRow(row) {
  if(row.length<43||!row.some(finite))return;
  realtime.pending.push(row);
  if(!realtime.flushRequest)realtime.flushRequest=requestAnimationFrame(flushRealtime);
}
function flushRealtime() {
  realtime.flushRequest=0;if(!realtime.pending.length)return;
  S.rows.push(...realtime.pending.splice(0));
  const time=timeline(S.rows,config);Object.assign(S,time);S.quality=qualityReport({rows:S.rows,totalLines:S.rows.length,comments:0,headers:0,skipped:0,bad:[],source:'native'},time);S.cur=S.rows.length-1;
  rebuild();S.wave.start=Math.max(0,duration()-config.dt*200);S.wave.end=duration()||config.dt;$('slider').max=Math.max(0,S.rows.length-1);fitMap();updateWindow();renderEvents();renderQuality();render();
}
function receiveRealtimeText(text) {
  realtime.buffer+=text;const lines=realtime.buffer.split(/\r?\n/);realtime.buffer=lines.pop()||'';
  for(const line of lines){const raw=line.trim();if(!raw)continue;if(raw.startsWith('#BTLOG,PID_PARAM,')||raw.startsWith('#BTLOG,PID_ERROR,')){receivePidParamLine(raw);continue;}archiveLine(raw+'\n');if(receivePidParamLine(raw))continue;if(receivePidLine(raw))continue;if(/^#BTLOG,END\b/i.test(raw)){realtime.ended=true;archiveFinish(true);realtimeStatus('已收到 END','connected');continue;}if(raw.startsWith('#')||raw.startsWith('//'))continue;
    try{const parsed=parse(raw+'\n');if(parsed.rows[0])queueRealtimeRow(parsed.rows[0]);}catch(_){/* Ignore partial/status lines from the collector. */}
  }
}
function disconnectRealtime() { if(realtime.socket){realtime.socket.onclose=null;realtime.socket.close();realtime.socket=null;}realtime.buffer='';realtime.pending=[];realtimeStatus('电脑日志源未连接；BT10 直连可单独使用');$('ws-connect').textContent='连接电脑日志源'; }
function connectRealtime() {
  if(realtime.socket){disconnectRealtime();return;}
  if(typeof WebSocket==='undefined'){realtimeStatus('当前浏览器不支持 WebSocket','error');return;}
  const url=$('ws-url').value.trim();if(!/^wss?:\/\//i.test(url)){realtimeStatus('地址必须以 ws:// 或 wss:// 开头','error');return;}
  try{const socket=new WebSocket(url);realtime.socket=socket;realtime.buffer='';realtime.ended=false;$('ws-connect').textContent='断开实时日志';realtimeStatus('连接中…');
    socket.onopen=()=>{resetRealtimeSession();realtimeStatus('已连接，等待 BT10 日志','connected');notify('已连接实时日志；等待 #BTLOG,BEGIN 和 ULog43 数据。');};
    socket.onmessage=e=>{if(typeof e.data==='string')receiveRealtimeText(e.data);else if(e.data?.text)e.data.text().then(receiveRealtimeText);};
    socket.onerror=()=>realtimeStatus('电脑日志源连接失败；BT10 直连不受影响','error');
    socket.onclose=()=>{realtime.socket=null;$('ws-connect').textContent='连接电脑日志源';if(!realtime.ended)realtimeStatus('电脑日志源已断开；BT10 直连仍可用','error');};
  }catch(e){realtime.socket=null;$('ws-connect').textContent='连接实时日志';realtimeStatus('连接失败：'+e.message,'error');}
}
function rebuild() {
  S.samples=S.rows.map(r=>sample(r));
  S.points=S.rows.map(row=>({center:fieldPoint(value(row,'x'),value(row,'y')),
    ...Object.fromEntries(WHEELS.map(w=>{const q=wheelPos(row,w);return [w,fieldPoint(q.x,q.y)];}))}));
  calculatePid();
  S.segments=detectSegments();
  $('meta').textContent=S.rows.length?`${S.name} · ${S.rows.length} 帧 · ${fmt(duration(),2)} s · 中位采样 ${fmt(S.medianDt*1000,1)} ms`:'本项目 ULog43 · 等待日志';
  $('source-info').textContent=`轮速换算：${fmt(pulseToMeter(),9)} m / pulse；旋转力臂：${fmt((config.wheelbase+config.track)/2,3)} m。vx / vy ${config.channels.vx===null&&config.channels.vy===null?'由四轮重建':'含手动直接通道'}；ω ${config.omegaMode==='wheels'?'由四轮重建':'来自日志 rad/s 实测通道'}。`;
}
function fieldPoint(x,y) {const a=S.alignment.angle/DEG,c=Math.cos(a),s=Math.sin(a);return [S.alignment.x+c*x-s*y,S.alignment.y+s*x+c*y];}
function wheelPos(row,w) {
  const yaw=value(row,'yaw')/DEG,x=value(row,'x'),y=value(row,'y');
  const dx=(w==='fl'||w==='fr'?1:-1)*config.wheelbase/2,dy=(w==='fl'||w==='rl'?1:-1)*config.track/2;
  return {x:x+Math.cos(yaw)*dx-Math.sin(yaw)*dy,y:y+Math.sin(yaw)*dx+Math.cos(yaw)*dy};
}
function canvasContext(id) {
  const cv=$(id),rect=cv.getBoundingClientRect();if(rect.width<=0||rect.height<=0)return null;
  const dpr=Math.min(window.devicePixelRatio||1,2);cv.width=Math.round(rect.width*dpr);cv.height=Math.round(rect.height*dpr);
  const g=cv.getContext('2d');g.setTransform(dpr,0,0,dpr,0,0);g.clearRect(0,0,rect.width,rect.height);
  return {g,w:rect.width,h:rect.height};
}
function fitMap() {
  let minX=Infinity,minY=Infinity,maxX=-Infinity,maxY=-Infinity;
  function include(p){if(p.every(finite)){minX=Math.min(minX,p[0]);maxX=Math.max(maxX,p[0]);minY=Math.min(minY,p[1]);maxY=Math.max(maxY,p[1]);}}
  if($('show-field').checked){include([0,0]);include([2.4,2.4]);}
  for(const points of S.points)for(const k of ['center',...WHEELS])if(S.visible[k])include(points[k]);
  if(!finite(minX)){include([0,0]);include([2.4,2.4]);}
  Object.assign(S.map,{x:(minX+maxX)/2,y:(minY+maxY)/2,spanX:Math.max(.5,maxX-minX)+.22,spanY:Math.max(.5,maxY-minY)+.22,zoom:1});
  drawPath();
}
function project(x,y) {const m=S.map;return [m.w/2+(x-m.x)*m.scale,m.h/2-(y-m.y)*m.scale];}
function unproject(px,py) {const m=S.map;return [m.x+(px-m.w/2)/m.scale,m.y-(py-m.h/2)/m.scale];}
function line(g,points,color,width=1,dash=[]) {
  g.strokeStyle=color;g.lineWidth=width;g.setLineDash(dash);g.beginPath();let pen=false;
  for(const p of points){if(!p||!p.every(finite)){pen=false;continue;}if(pen)g.lineTo(...p);else g.moveTo(...p);pen=true;}
  g.stroke();g.setLineDash([]);
}
function drawField(g) {
  if(!$('show-field').checked)return;
  const m=S.map;
  function rect(x,y,w,h,fill,stroke){const p=project(x,y+h);g.fillStyle=fill;g.fillRect(...p,w*m.scale,h*m.scale);if(stroke){g.strokeStyle=stroke;g.lineWidth=1;g.strokeRect(...p,w*m.scale,h*m.scale);}}
  function label(text,x,y,color='#8299b3',size=11){g.fillStyle=color;g.font=`${size}px "Microsoft YaHei",sans-serif`;g.textAlign='center';g.fillText(text,...project(x,y));}
  rect(0,0,2.4,2.4,'#172333','#66829b');
  for(let i=1;i<12;i++){line(g,[project(i*.2,0),project(i*.2,2.4)],'#203145');line(g,[project(0,i*.2),project(2.4,i*.2)],'#203145');}
  line(g,[project(1.2,0),project(1.2,2.4)],'#516176',1,[6,7]);line(g,[project(0,1.2),project(2.4,1.2)],'#516176',1,[6,7]);
  for(const x of [.55,1.4])for(const y of [.55,1.4]){rect(x,y,.45,.45,'#3a3530','#928066');label('障碍区',x+.225,y+.245,'#d9c5a7');label('450 × 450',x+.225,y+.17,'#968776',9);}
  label('通道 400 mm',1.2,1.31,'#a0b4c9',10);
  rect(2.1,2.1,.3,.3,'#20476c','#4e9bd0');rect(2.1,0,.3,.3,'#20476c','#4e9bd0');
  label('启停 1',2.25,2.23,'#b9e1ff');label('启停 2',2.25,.13,'#b9e1ff');
  rect(.02,.91,.17,.58,'#243a3e');label('暂存区',.28,1.25,'#8bd5bc');
  rect(.91,.02,.58,.17,'#29394c');label('粗加工区',1.2,.27,'#b5cce5');
  for(let i=0;i<3;i++)for(const p of [[.105,1.04+i*.16],[1.04+i*.16,.105]]){g.beginPath();g.arc(...project(...p),.045*m.scale,0,Math.PI*2);g.strokeStyle=['#f1838f','#7bb2ea','#88ccae'][i];g.stroke();}
  g.beginPath();g.arc(...project(1.2,2.4),.14*m.scale,0,Math.PI*2);g.fillStyle='#263f48';g.fill();g.strokeStyle='#71b9b8';g.stroke();
  label('原料区',1.2,2.14,'#9bcfd0');
  rect(2.32,1.05,.06,.3,'#40506a');label('码牌',2.17,1.2,'#b5c5de',10);
  label('场地 2400 × 2400 mm',1.2,-.065,'#8ca5be',10);
  label('0',.03,.05,'#94b0cb',10);label('X →',.38,.05,'#94b0cb',10);label('Y ↑',.06,.38,'#94b0cb',10);
}
function drawPath() {
  if(S.tab!=='transport')return;
  const ctx=canvasContext('path');if(!ctx)return;const {g,w,h}=ctx,m=S.map;
  m.w=w;m.h=h;m.scale=Math.min((w-34)/m.spanX,(h-34)/m.spanY)*m.zoom;
  drawField(g);
  // The logged target pose is a control target, not the original planned path.
  const targetPoints=S.rows.map(r=>{const x=value(r,'xt'),y=value(r,'yt');return finite(x)&&finite(y)?project(...fieldPoint(x,y)):null;});
  line(g,targetPoints,'#e7edf7',1,[7,5]);
  const target=S.rows[S.cur],tx=value(target,'xt'),ty=value(target,'yt');
  if(finite(tx)&&finite(ty)){g.fillStyle='#e7edf7';g.beginPath();g.arc(...project(...fieldPoint(tx,ty)),5,0,Math.PI*2);g.fill();}
  const progressive=$('trail').checked;
  for(const key of ['center',...WHEELS]) {
    if(!S.visible[key])continue;
    const points=[];S.points.forEach((p,i)=>{if(i&&S.times[i]-S.times[i-1]>S.medianDt*3)points.push(null);points.push(p[key].every(finite)?project(...p[key]):null);});
    g.globalAlpha=progressive?.24:.8;line(g,points,COLORS[key],key==='center'?2:1.5);g.globalAlpha=1;
    if(progressive){const past=[];for(let i=0;i<=S.cur&&i<S.points.length;i++){if(i&&S.times[i]-S.times[i-1]>S.medianDt*3)past.push(null);past.push(S.points[i][key].every(finite)?project(...S.points[i][key]):null);}line(g,past,COLORS[key],2);}
    const q=S.points[S.cur]?.[key];if(q?.every(finite)){g.beginPath();g.arc(...project(...q),key==='center'?5:4,0,Math.PI*2);g.fillStyle=COLORS[key];g.fill();}
  }
  const row=S.rows[S.cur];if(row&&S.points[S.cur].center.every(finite)) {
    // Chassis outline is context only; solo mode leaves only the selected wheel trail visible.
    const ps=['fl','fr','rr','rl','fl'].map(k=>S.points[S.cur][k]);line(g,ps.map(p=>project(...p)),'#b7d2eb66',1);
    const x=value(row,'x'),y=value(row,'y'),yaw=value(row,'yaw')/DEG;
    if(finite(yaw))line(g,[project(...fieldPoint(x,y)),project(...fieldPoint(x+.16*Math.cos(yaw),y+.16*Math.sin(yaw)))],'#e3efff',2);
  }
  g.fillStyle='#95aac2';g.font='12px sans-serif';g.textAlign='left';
  if(!S.rows.length)g.fillText('导入 CSV 叠加轨迹；场地无需数据即可查看',14,h-15);
  else if(!S.points.some(p=>p.center.every(finite)))g.fillText('缺少有效 X / Y 位置通道，请检查映射',14,h-15);
  $('map-zoom').textContent=Math.round(m.zoom*100)+'%';
}
function zoomMap(factor,px=S.map.w/2,py=S.map.h/2) {
  const m=S.map,world=unproject(px,py),next=clamp(m.zoom*factor,.25,30),scale=m.scale*next/m.zoom;
  if(!finite(scale)||scale<=0)return;
  m.x=world[0]-(px-m.w/2)/scale;m.y=world[1]+(py-m.h/2)/scale;m.zoom=next;drawPath();
}
function pickMap(px,py) {
  let best=-1,distance=18**2;
  S.points.forEach((p,i)=>{for(const key of ['center',...WHEELS])if(S.visible[key]&&p[key].every(finite)){const q=project(...p[key]),d=(q[0]-px)**2+(q[1]-py)**2;if(d<distance){distance=d;best=i;}}});
  if(best>=0)go(best);
}
function solo(w) {for(const key of ['center',...WHEELS]){S.visible[key]=key===w;$('show-'+key).checked=key===w;}drawPath();}
function waveDefinitions() {
  if(S.wave.mode==='wheels')return WHEELS.map((w,i)=>({name:NAMES[w],unit:'pulse/s',color:COLORS[w],get:(_,r)=>value(r,w+'a'),target:(_,r)=>value(r,w+'t')}));
  if(S.wave.mode==='errors')return [
    {name:'目标横向偏差',unit:'m',color:'#50dfae',get:a=>a.lateral,threshold:S.tolerances.lateral},
    {name:'航向误差',unit:'°',color:'#ffc86a',get:a=>a.yawError,threshold:S.tolerances.yaw},
    {name:'角速度误差',unit:'°/s',color:'#c4a0ff',get:a=>a.omegaError,threshold:S.tolerances.omega}];
  return [{name:'纵向 vx',unit:'m/s',color:'#67baff',get:a=>a.vx,target:a=>a.targetVx},
    {name:'横向 vy',unit:'m/s',color:'#50dfae',get:a=>a.vy,target:a=>a.targetVy},
    {name:'角速度 ω',unit:'°/s',color:'#c4a0ff',get:a=>a.omegaDeg,target:a=>a.targetOmega},
    {name:'平移 |v|',unit:'m/s',color:'#ffc86a',get:a=>a.speed,target:a=>a.targetSpeed}];
}
function windowIndices() {return [lowerBound(S.times,S.wave.start),lowerBound(S.times,S.wave.end+1e-9)];}
function waveX(time) {return 94+(time-S.wave.start)/(S.wave.end-S.wave.start)*Math.max(1,S.wave.w-110);}
function waveTime(px) {return S.wave.start+(px-94)/Math.max(1,S.wave.w-110)*(S.wave.end-S.wave.start);}
function drawStrip() {
  const ctx=canvasContext('strip');if(!ctx)return;const {g,w,h}=ctx;S.wave.w=w;S.wave.h=h;
  const defs=waveDefinitions(),rowH=(h-30)/defs.length,[start,end]=windowIndices();
  g.font='11px "Microsoft YaHei",sans-serif';
  defs.forEach((def,j)=>{
    const top=j*rowH+8,bottom=(j+1)*rowH-12,middle=(top+bottom)/2;
    let max=0,has=false;
    for(let i=start;i<end;i++){for(const v of [def.get(S.samples[i],S.rows[i]),def.target?.(S.samples[i],S.rows[i])])if(finite(v)){max=Math.max(max,Math.abs(v));has=true;}}
    max=Math.max(max*1.15,def.unit==='pulse/s'?10:def.unit==='°/s'?1:.01)/S.wave.gain;
    const y=v=>middle-v/max*(bottom-top)/2;
    g.fillStyle=def.color;g.textAlign='left';g.fillText(def.name,102,top+12);g.fillStyle='#8299b3';g.fillText(def.unit,8,middle+4);
    g.textAlign='right';g.fillText(fmt(max,max<1?3:1),88,top+10);g.fillText('0',88,middle+4);g.fillText(fmt(-max,max<1?3:1),88,bottom+2);
    line(g,[[94,middle],[w-16,middle]],'#34465e');line(g,[[94,bottom+7],[w-16,bottom+7]],'#23344a');
    g.save();g.beginPath();g.rect(94,top,w-110,bottom-top);g.clip();
    for(let k=0;k<=5;k++){const x=94+k*(w-110)/5;line(g,[[x,top],[x,bottom]],'#223147');}
    if(def.threshold){for(const sign of [-1,1])line(g,[[94,y(def.threshold*sign)],[w-16,y(def.threshold*sign)]],'#9b7949',1,[2,4]);}
    for(const isTarget of [false,true]) {
      if(isTarget&&!def.target)continue;
      const points=[];
      for(let i=Math.max(0,start-1);i<Math.min(S.rows.length,end+1);i++){
        if(i&&S.times[i]-S.times[i-1]>S.medianDt*3)points.push(null);
        const v=(isTarget?def.target:def.get)(S.samples[i],S.rows[i]);points.push(finite(v)?[waveX(S.times[i]),y(v)]:null);
      }
      line(g,points,isTarget?'#e4edf7':def.color,isTarget?1:1.7,isTarget?[5,4]:[]);
    }
    g.restore();
    if(!has){g.fillStyle='#71869e';g.textAlign='left';g.fillText(S.rows.length?'该通道未记录':'等待日志',105,middle-7);}
  });
  if(S.rows.length&&S.times[S.cur]>=S.wave.start&&S.times[S.cur]<=S.wave.end)line(g,[[waveX(S.times[S.cur]),0],[waveX(S.times[S.cur]),h-26]],'#ffffffaa',1);
  g.fillStyle='#8fa5bf';for(let k=0;k<=5;k++){g.textAlign=k===0?'left':k===5?'right':'center';g.fillText(fmt(S.wave.start+(S.wave.end-S.wave.start)*k/5,2)+' s',94+k*(w-110)/5,h-6);}
}
function setWindow(start,end) {
  const total=duration()||config.dt,minSpan=Math.min(total,Math.max(.01,S.medianDt*2));
  const span=clamp(end-start,minSpan,total);S.wave.start=clamp(start,0,Math.max(0,total-span));S.wave.end=S.wave.start+span;updateWindow();drawStrip();
}
function updateWindow() {
  $('wave-start').value=fmt(S.wave.start,3);$('wave-end').value=fmt(S.wave.end,3);$('wave-scale').textContent=`幅度 ${fmt(S.wave.gain,2)}×`;
  drawStability();
}
function zoomWave(factor,anchor=(S.wave.start+S.wave.end)/2) {
  const span=S.wave.end-S.wave.start,fraction=(anchor-S.wave.start)/span;const next=span/factor;
  setWindow(anchor-next*fraction,anchor+next*(1-fraction));$('wave-window').value='all';
}
function statistics(values) {
  let n=0,mean=0,m2=0,squares=0,max=0;
  for(const v of values)if(finite(v)){n++;const d=v-mean;mean+=d/n;m2+=d*(v-mean);squares+=v*v;max=Math.max(max,Math.abs(v));}
  return {n,mean:n?mean:NaN,rms:n?Math.sqrt(squares/n):NaN,std:n?Math.sqrt(m2/n):NaN,max:n?max:NaN};
}
function drawStability() {
  const [a,b]=windowIndices(),samples=S.samples.slice(a,b),v=statistics(samples.map(s=>s.speed));
  $('window-label').textContent=`${fmt(S.wave.start,2)}–${fmt(S.wave.end,2)} s · ${samples.length} 帧 · 按采样点统计`;
  let html='';
  for(const [name,key,tol,unit] of [['目标横向偏差','lateral',S.tolerances.lateral,'m'],['航向误差','yawError',S.tolerances.yaw,'°'],['角速度跟踪误差','omegaError',S.tolerances.omega,'°/s']]){
    const stats=statistics(samples.map(s=>s[key])),ok=stats.rms<=tol,color=ok?'#50dfae':'#ffc86a';
    html+=`<div class="stat"><div class="row"><span>${name} RMS</span><b>${fmt(stats.rms)} <small>${unit}</small></b></div><div class="stat-bar"><i style="width:${finite(stats.rms)?clamp(stats.rms/tol*50,0,100):0}%;background:${color}"></i></div><small class="${stats.n?(ok?'ok':'warn'):''}">${stats.n?(ok?'参考内':'高于参考'):'未记录'} · 参考 ${tol} ${unit} · 峰值 ${fmt(stats.max)} · 有效 ${stats.n} 帧</small></div>`;
  }
  html+=`<div class="stat"><div class="row"><span>平移速度 均值 / 峰值</span><b>${fmt(v.mean,2)} / ${fmt(v.max,2)}</b></div><small>m/s · 标准差 ${fmt(v.std)}（包含加减速）</small></div>`;
  $('stability').innerHTML=html;
  drawParking(a,b);
}
function drawParking(a,b) {
  const rows=S.rows,samples=S.samples;if(!rows.length){$('parking').innerHTML='<span class="note">等待日志</span>';return;}
  const threshold=.04,stableDuration=.5;let command=-1,stable=-1;
  // No stop command is logged in ULog43. Infer the last transition from moving to a stable low-speed state.
  for(let i=Math.max(1,a);i<Math.min(b,rows.length);i++){
    const before=samples[i-1]?.speed||0,now=samples[i]?.speed||0;
    if(command<0&&before>threshold*2&&now<=threshold*2)command=i;
    if(command>=0&&stable<0&&now<=threshold){let j=i;while(j<b&&samples[j]?.speed<=threshold)j++;if(j<b&&S.times[j]-S.times[i]>=stableDuration)stable=i;}
  }
  if(command<0){$('parking').innerHTML='<span class="note">当前窗口未发现停止响应（速度未从运动进入阈值）。</span>';return;}
  const end=stable>=0?stable:b-1,base=rows[command],last=rows[end];
  const dx=value(last,'x')-value(base,'x'),dy=value(last,'y')-value(base,'y'),d=finite(dx)&&finite(dy)?Math.hypot(dx,dy):NaN;
  const yawDelta=finite(value(last,'yaw'))&&finite(value(base,'yaw'))?value(last,'yaw')-value(base,'yaw'):NaN;
  $('parking').innerHTML='<table>'+tableRows([['停止指令','未记录 · 网站推断'],['进入低速阈值',fmt(S.times[command],2)+' s'],['持续稳定',stable>=0?fmt(S.times[stable],2)+' s':'未达到 '+stableDuration+' s'],['停止响应时间',stable>=0?fmt(S.times[stable]-S.times[command],2)+' s':'—'],['停车位移',finite(d)?fmt(d)+' m（估计）':'—'],['停车后航向变化',finite(yawDelta)?fmt(yawDelta,2)+' °（估计）':'—']])+'</table>';
}
function tableRows(entries) {return entries.map(([name,val])=>`<tr><td>${escapeHtml(name)}</td><td>${escapeHtml(val)}</td></tr>`).join('');}
function drawDirection(a) {
  const ctx=canvasContext('direction');if(!ctx)return;const {g,w,h}=ctx,cx=w/2,cy=h/2;
  g.strokeStyle='#304962';g.beginPath();g.arc(cx,cy,Math.min(w,h)*.32,0,Math.PI*2);g.stroke();
  line(g,[[cx,12],[cx,h-12]],'#314660');line(g,[[12,cy],[w-12,cy]],'#314660');
  g.font='9px "Microsoft YaHei",sans-serif';g.fillStyle='#8fa6bf';g.textAlign='center';g.fillText('前',cx,9);g.fillText('左',7,cy+3);
  if(!a||!finite(a.speed)||a.speed<.0001)return;
  const len=Math.min(w,h)*.3*Math.min(1,a.speed),dx=-a.vy/a.speed*len,dy=-a.vx/a.speed*len,x=cx+dx,y=cy+dy;
  line(g,[[cx,cy],[x,y]],'#67baff',2.5);const ang=Math.atan2(dy,dx);
  line(g,[[x-5*Math.cos(ang-.55),y-5*Math.sin(ang-.55)],[x,y],[x-5*Math.cos(ang+.55),y-5*Math.sin(ang+.55)]],'#67baff',2);
}
function drawCurrent() {
  const row=S.rows[S.cur],a=S.samples[S.cur];
  for(const [id,val,range] of [['vx',a?.vx,1],['vy',a?.vy,1],['omega',a?.omegaDeg,180]]){
    $(id).textContent=signed(val,id==='omega'?1:3);const pct=finite(val)?clamp(val/range,-1,1)*50:0;
    $(id+'-bar').style.left=(pct<0?50+pct:50)+'%';$(id+'-bar').style.width=Math.abs(pct)+'%';
  }
  $('omega-source').textContent=`°/s · ${config.omegaMode==='wheels'?'四轮运动学':'日志实测 rad/s 转换'}`;
  // The component captions remain explicit when users map direct velocity channels.
  $('vx').nextElementSibling.textContent='m/s · '+(config.channels.vx===null?'四轮运动学':'直接 vx 通道');
  $('vy').nextElementSibling.textContent='m/s · '+(config.channels.vy===null?'四轮运动学':'直接 vy 通道');
  $('speed').textContent=fmt(a?.speed);$('motion').textContent=!a?'m/s · 等待日志':!finite(a.speed)||!finite(a.omega)?'数据缺失':a.speed<.01?(Math.abs(a.omega)>.03?'原地旋转':'接近静止'):`m/s · ${a.vx>=0?'前':'后'} ${a.vy>=0?'左':'右'} · 箭头满量程 1 m/s`;
  $('phase').textContent=row?phaseName(row):'—';drawDirection(a);
  $('state').innerHTML=tableRows([['日志时间',fmt(S.times[S.cur],2)+' s'],['X / Y',fmt(value(row,'x'))+' / '+fmt(value(row,'y'))+' m'],['底盘 / IMU 航向',fmt(value(row,'yaw'),1)+' / '+fmt(value(row,'imu'),1)+' °'],['目标位置',fmt(value(row,'xt'))+' / '+fmt(value(row,'yt'))+' m'],['目标 |v|',fmt(a?.targetSpeed)+' m/s'],['目标 ω',fmt(a?.targetOmega,1)+' °/s'],['日志 ω',fmt(value(row,'wa')*DEG,1)+' °/s'],['四轮估计 ω',fmt(a?.wheelOmega*DEG,1)+' °/s']]);
  if(S.tab==='motor')drawMotors();if(S.tab==='pid')drawPid();if(S.tab==='channels')drawChannels();updatePlayback();
}
function drawMotors() {
  const row=S.rows[S.cur],a=S.samples[S.cur];let max=1;
  for(const w of WHEELS)if(finite(value(row,w+'a')))max=Math.max(max,Math.abs(value(row,w+'a')*pulseToMeter()));
  $('motor-cards').innerHTML=WHEELS.map(w=>{
    const act=value(row,w+'a'),target=value(row,w+'t'),v=act*pulseToMeter(),pct=finite(v)?v/max*50:0;
    const read=(key,unit)=>finite(value(row,key))?fmt(value(row,key),1)+' '+unit:'未记录';
    return `<div class="motor-item"><h3 style="color:${COLORS[w]}">${NAMES[w]}</h3><div class="wheel-speed" style="color:${COLORS[w]}">${signed(v)}</div><small>轮缘线速度 m/s（由 pulse/s 换算）</small><div class="wheel-bar"><i style="left:${pct<0?50+pct:50}%;width:${Math.abs(pct)}%;background:${COLORS[w]}"></i></div><table>${tableRows([['实测',read(w+'a','pulse/s')],['目标',read(w+'t','pulse/s')],['目标 − 实测',finite(target-act)?fmt(target-act,1)+' pulse/s':'未记录'],['控制输出',read(w+'o','原始单位')],['编码器计数',read(w+'e','pulse')]])}</table></div>`;
  }).join('');
  $('motor-health').innerHTML='<table>'+tableRows([['vx / vy',`${fmt(a?.vx)} / ${fmt(a?.vy)} m/s`],['四轮估计 ω',fmt(a?.wheelOmega*DEG,2)+' °/s'],['记录 ω − 四轮估计 ω',fmt((value(row,'wa')-a?.wheelOmega)*DEG,2)+' °/s'],['解释','两种 ω 有明显差异时，先检查旧转换占位、单位、轮序及几何参数']])+'</table>';
}
function calculatePid() {
  const axis=$('pid-axis').value||'omega';let integral=0,previous=NaN;
  S.pid=S.rows.map((r,i)=>{
    const a=S.samples[i];let target,act;
    if(axis==='omega'){target=a.targetOmega;act=a.omegaDeg;}
    else if(axis==='fl'){target=value(r,'flt');act=value(r,'fla');}
    else{target=value(r,axis+'t');act=value(r,axis);}
    const e=target-act,dt=i?S.times[i]-S.times[i-1]:0;
    let derivative=0;
    if(!finite(e)){previous=NaN;integral=0;return {target,act,e,integral:NaN,derivative:NaN};}
    if(finite(previous)&&dt>0&&dt<=S.medianDt*3){integral+=(e+previous)*.5*dt;derivative=(e-previous)/dt;}else integral=0;
    previous=e;return {target,act,e,integral,derivative};
  });
}
function drawPid() {
  const p=S.pid[S.cur],kp=Number($('pid-p').value),ki=Number($('pid-i').value),kd=Number($('pid-d').value);
  const pp=kp*p?.e,ii=ki*p?.integral,dd=kd*p?.derivative;
  $('pid-readout').innerHTML=[['目标',p?.target],['实测',p?.act],['误差',p?.e],['P 项',pp],['I 项',ii],['D 项',dd],['理想输出 P+I+D',pp+ii+dd]].map(([k,v])=>`<div><small>${k}</small><strong>${fmt(v)}</strong></div>`).join('');
}
function drawChannels() {
  const row=S.rows[S.cur];$('channel-table').innerHTML=row?row.map((v,i)=>{
    const labels=Object.keys(config.channels).filter(k=>config.channels[k]===i).map(k=>CHANNEL_LABELS[k]);
    const label=labels.length?labels.join(' / '):RAW_LABELS[i]||'未使用 / 占位';
    return `<tr><td>ch${i}</td><td>${escapeHtml(label)}</td><td>${fmt(v,6)}</td></tr>`;
  }).join(''):'<tr><td colspan="3">等待日志</td></tr>';
}
function renderEvents() {
  $('events').innerHTML=S.segments.map((s,i)=>`<button data-segment="${i}"><b>${fmt(S.times[s.a],2)}–${fmt(S.times[s.b],2)} s</b> · ${escapeHtml(s.label)} · ${fmt(s.duration,2)} s <small>· ${s.source}</small></button>`).join('');
  $('events').querySelectorAll('button').forEach(b=>b.onclick=()=>segmentWindow(S.segments[Number(b.dataset.segment)]));
}
function renderQuality() {
  const q=S.quality;if(!q)return;
  $('quality-format').textContent=q.format;
  const cards=[['有效行',q.validRows],['异常行',q.invalidRows],['注释行',q.comments],['表头行',q.headers],['缺失值',q.missing],['长缺口',q.gaps]];
  $('quality-summary').innerHTML=cards.map(([n,v])=>`<div class="quality-item"><small>${n}</small><b>${v}</b></div>`).join('');
  $('quality-details').innerHTML=tableRows([['文件总行数',q.totalLines],['有效时间范围',`${fmt(0,3)}–${fmt(q.duration,3)} s`],['采样间隔中位数',fmt(q.medianDt*1000,2)+' ms'],['采样间隔 P95',fmt(q.p95*1000,2)+' ms'],['最大采样间隔',fmt(q.maxDt*1000,2)+' ms'],['重复时间戳',q.repeated],['倒退时间戳',q.backward],['长间隔位置',q.gaps?`${q.gaps} 处（轨迹断线）`:'无']]);
  const bad=q.bad?.slice(0,8).map(x=>`第 ${x.line} 行：${x.reason}`).join('；');
  $('quality-note').textContent=bad?`异常行示例：${bad}${q.bad.length>8?'；其余见导入提示':''}`:'未发现格式异常行。';
  $('quality-source').textContent=q.source==='converted'?'文件头包含 “converted from ULog26”：这是转换日志，ch15 等补零字段不能当作独立实测。':q.source==='named'?'检测到字段名表头；当前列映射仍需在高级映射中确认。':'未检测到转换标记；请确认该文件确实来自本工程固件。';
}
function render() {drawCurrent();drawPath();drawStrip();}
function go(index,fromPlayback=false) {
  if(!S.rows.length)return;S.cur=clamp(Math.round(index),0,S.rows.length-1);
  if(S.playback.running&&!fromPlayback){S.playback.anchorTime=S.times[S.cur];S.playback.anchorMs=performance.now();}
  if($('wave-follow').checked){const t=S.times[S.cur],span=S.wave.end-S.wave.start;if(t<S.wave.start||t>S.wave.end)setWindow(t-span*.15,t+span*.85);}
  render();
}
function updatePlayback() {
  $('slider').value=S.cur;$('play-toggle').disabled=S.rows.length<2;$('play-restart').disabled=S.rows.length<2;
  $('play-toggle').textContent=S.playback.running?'暂停':'播放';
  $('play-time').textContent=S.rows.length?`${fmt(S.times[S.cur],2)} / ${fmt(duration(),2)} s · ${S.cur+1}/${S.rows.length}`:'未加载日志';
}
function pausePlayback() {const p=S.playback;p.running=false;if(p.request!==null)cancelAnimationFrame(p.request);p.request=null;updatePlayback();}
function playbackTick(now) {
  const p=S.playback;if(!p.running)return;p.request=null;
  const t=p.anchorTime+(now-p.anchorMs)/1000*p.rate;
  if(t>=duration()){go(S.rows.length-1,true);pausePlayback();return;}
  const i=frameAt(t);if(i!==S.cur&&now-p.lastPaint>=30){go(i,true);p.lastPaint=now;}
  p.request=requestAnimationFrame(playbackTick);
}
function startPlayback(restart=false) {
  if(S.rows.length<2)return;pausePlayback();if(restart||S.cur===S.rows.length-1)go(0);
  const p=S.playback;p.running=true;p.anchorMs=performance.now();p.anchorTime=S.times[S.cur];p.lastPaint=0;p.request=requestAnimationFrame(playbackTick);updatePlayback();
}
function activateTab(id) {
  S.tab=id;document.querySelectorAll('.tab').forEach(b=>b.classList.toggle('active',b.dataset.tab===id));
  document.querySelectorAll('.pane').forEach(p=>p.classList.toggle('active',p.id===id));
  setWavePreview(false);
  if(id==='quality')renderQuality();render();
  if(id==='pid'){renderPidStream();drawPidModelTrace();}
}
function showConfig() {
  $('radius').value=config.radius;$('ppr').value=config.ppr;$('wheelbase').value=config.wheelbase;$('track').value=config.track;$('fallback-dt').value=config.dt;$('omega-mode').value=config.omegaMode;
  $('mapping').innerHTML=Object.entries(config.channels).map(([key,v])=>`<label>${CHANNEL_LABELS[key]}<input id="map-${key}" type="number" min="0" step="1" value="${v===null?'':v}" aria-label="${CHANNEL_LABELS[key]}通道"></label>`).join('');
  for(const [key,id] of Object.entries({firmware:'meta-firmware',test:'meta-test',content:'meta-content',environment:'meta-environment',battery:'meta-battery',notes:'meta-notes'}))$(id).value=config.metadata?.[key]||'';
}
function readMetadata() {return {firmware:$('meta-firmware').value.trim(),test:$('meta-test').value.trim(),content:$('meta-content').value.trim(),environment:$('meta-environment').value.trim(),battery:$('meta-battery').value.trim(),notes:$('meta-notes').value.trim()};}
function saveAnalysisConfig() {
  config.metadata=readMetadata();
  const payload={config:{...config,channels:{...config.channels},metadata:{...config.metadata}},alignment:{...S.alignment}};
  try{localStorage.setItem('mecanum-analysis-config',JSON.stringify(payload));$('config-status').textContent='已保存到本机浏览器（人工填写）。';}catch(e){$('config-status').textContent='浏览器禁止本地存储，请手动记录。';}
}
function loadAnalysisConfig() {
  try{const payload=JSON.parse(localStorage.getItem('mecanum-analysis-config')||'null');if(!payload?.config)throw new Error('没有保存配置');config={...config,...payload.config,channels:{...DEFAULT_CHANNELS,...payload.config.channels},metadata:{...config.metadata,...payload.config.metadata}};if(payload.alignment)S.alignment={...S.alignment,...payload.alignment};showConfig();rebuild();fitMap();render();$('config-status').textContent='已载入保存配置。';}catch(e){$('config-status').textContent='没有可载入的保存配置。';}
}
function applyConfig(reset=false) {
  const next=reset?{radius:.04,ppr:440,wheelbase:.186,track:.22,dt:.05,omegaMode:'wheels',channels:{...DEFAULT_CHANNELS}}:{
    radius:Number($('radius').value),ppr:Number($('ppr').value),wheelbase:Number($('wheelbase').value),track:Number($('track').value),dt:Number($('fallback-dt').value),omegaMode:$('omega-mode').value,channels:{}};
  try {
    for(const key of ['radius','ppr','wheelbase','track','dt'])if(!finite(next[key])||next[key]<=0)throw new Error('几何、脉冲数与帧周期必须为正数。');
    if(!reset)for(const key of Object.keys(DEFAULT_CHANNELS)){
      const raw=$('map-'+key).value.trim(),v=raw===''?null:Number(raw);
      if(v!==null&&(!Number.isInteger(v)||v<0||v>4095))throw new Error('通道必须为 0–4095 的整数或空白。');next.channels[key]=v;
    }
    const time=S.rows.length?timeline(S.rows,next):{times:[],medianDt:next.dt,gaps:0};
    pausePlayback();config=next;Object.assign(S,time);rebuild();showConfig();fitMap();setWindow(0,duration()||config.dt);renderEvents();render();
    notify(`已应用通道与机械参数。${next.channels.time===null?'当前未使用时间戳，播放时间由帧周期估算。':''}${time.gaps?'存在时间缺口，已断线显示。':''}`);
  }catch(e){notify('参数未应用：'+e.message,true);}
}
// Pointer gestures distinguish clicks from drags and use CSS pixels at every DPR.
function bindGestures(id,kind) {
  const cv=$(id);let drag=null;
  const point=e=>{const r=cv.getBoundingClientRect();return [e.clientX-r.left,e.clientY-r.top];};
  cv.addEventListener('pointerdown',e=>{if(e.button!==0)return;const [x,y]=point(e);drag={x,y,startX:x,startY:y,moved:false,mapX:S.map.x,mapY:S.map.y,waveStart:S.wave.start,waveEnd:S.wave.end};cv.setPointerCapture?.(e.pointerId);});
  cv.addEventListener('pointermove',e=>{
    const [x,y]=point(e);
    if(kind==='wave'&&S.rows.length){const i=nearestFrame(clamp(waveTime(x),0,duration()));$('wave-hover').textContent=`${fmt(S.times[i],3)} s · `+waveDefinitions().map(d=>`${d.name} ${fmt(d.get(S.samples[i],S.rows[i]))} ${d.unit}`).join(' · ');}
    if(!drag)return;const dx=x-drag.startX,dy=y-drag.startY;if(Math.hypot(dx,dy)>4)drag.moved=true;if(!drag.moved)return;
    if(kind==='map'){S.map.x=drag.mapX-dx/S.map.scale;S.map.y=drag.mapY+dy/S.map.scale;drawPath();}
    else{const delta=-dx/(S.wave.w-110)*(drag.waveEnd-drag.waveStart);$('wave-follow').checked=false;setWindow(drag.waveStart+delta,drag.waveEnd+delta);}
  });
  cv.addEventListener('pointerup',e=>{if(!drag)return;const [x,y]=point(e);if(!drag.moved){if(kind==='map')pickMap(x,y);else if(S.rows.length)go(nearestFrame(clamp(waveTime(x),0,duration())));}drag=null;cv.releasePointerCapture?.(e.pointerId);});
  cv.addEventListener('pointercancel',()=>{drag=null;});
  cv.addEventListener('wheel',e=>{e.preventDefault();const [x,y]=point(e);const factor=e.deltaY<0?1.2:1/1.2;if(kind==='map')zoomMap(factor,x,y);else zoomWave(factor,clamp(waveTime(x),S.wave.start,S.wave.end));},{passive:false});
  cv.addEventListener('dblclick',()=>kind==='map'?fitMap():setWindow(0,duration()||config.dt));
}
function setWavePreview(expanded) {
  const body=document.body;if(!body)return;
  body.classList.toggle('transport-wave-mini',S.tab==='transport'&&!expanded);
  body.classList.toggle('transport-wave-expanded',S.tab==='transport'&&expanded);
  body.classList.remove('transport-wave-icon');
  $('wave-mini-toggle').textContent=expanded?'收起波形':'展开波形';
  requestAnimationFrame(()=>{drawPath();drawStrip();});
}
function setWaveMinimized(minimized) {
  const body=document.body;if(!body)return;
  body.classList.toggle('transport-wave-icon',S.tab==='transport'&&minimized);
  if(minimized){body.classList.remove('transport-wave-mini','transport-wave-expanded');$('wave-minimize').textContent='⌁';$('wave-minimize').title='恢复波形小窗';}
  else {$('wave-minimize').textContent='⌁';$('wave-minimize').title='缩小为波形图标';setWavePreview(false);}
  requestAnimationFrame(()=>{drawPath();drawStrip();});
}
function init() {
  showConfig();rebuild();renderPidBurst();
  document.querySelectorAll('.tab').forEach(b=>b.onclick=()=>activateTab(b.dataset.tab));
  $('ws-connect').onclick=connectRealtime;
  $('bt-connect').onclick=connectBt10;
  $('bt-selftest').onclick=sendBt10SelfTest;
  $('serial-echo').onclick=toggleSerialEcho;
  $('local-sim').onclick=toggleLocalSim;
  $('full-selftest').onclick=fullSelfTest;
  $('archive-refresh').onclick=archiveRender;$('archive-clear').onclick=archiveClear;archiveRender();
  $('file').onchange=async e=>{const file=e.target.files[0];if(file){try{load(await file.text(),file.name);}catch(err){notify('文件读取失败：'+err.message,true);}e.target.value='';}};
  document.addEventListener('dragover',e=>e.preventDefault());document.addEventListener('drop',async e=>{e.preventDefault();const file=e.dataTransfer?.files[0];if(file){try{load(await file.text(),file.name);}catch(err){notify('文件读取失败：'+err.message,true);}}});
  $('slider').oninput=()=>go(Number($('slider').value));$('play-toggle').onclick=()=>S.playback.running?pausePlayback():startPlayback();$('play-restart').onclick=()=>startPlayback(true);
  $('play-rate').onchange=()=>{if(S.playback.running){const p=S.playback;const t=p.anchorTime+(performance.now()-p.anchorMs)/1000*p.rate;p.anchorTime=Math.min(t,duration());p.anchorMs=performance.now();}S.playback.rate=Number($('play-rate').value);};
  document.addEventListener('visibilitychange',()=>{if(document.hidden){pausePlayback();archiveFlush();}});
  window.addEventListener('pagehide',()=>archiveFlush());
  document.addEventListener('keydown',e=>{if(e.key==='Escape'){$('map-card').classList.remove('expanded');$('map-expand').textContent='放大面板';setWavePreview(false);drawPath();}if(e.target?.matches('input,select,textarea,button,[contenteditable="true"]'))return;if(e.code==='Space'){e.preventDefault();S.playback.running?pausePlayback():startPlayback();}if(e.key==='ArrowLeft'){e.preventDefault();go(S.cur-1);}if(e.key==='ArrowRight'){e.preventDefault();go(S.cur+1);}});
  for(const key of ['center',...WHEELS])$('show-'+key).onchange=()=>{S.visible[key]=$('show-'+key).checked;drawPath();};
  document.querySelectorAll('[data-solo]').forEach(b=>b.onclick=()=>solo(b.dataset.solo));
  $('show-all').onclick=()=>{for(const key of ['center',...WHEELS]){S.visible[key]=true;$('show-'+key).checked=true;}drawPath();};
  $('show-field').onchange=()=>fitMap();$('trail').onchange=drawPath;
  $('map-plus').onclick=()=>zoomMap(1.3);$('map-minus').onclick=()=>zoomMap(1/1.3);$('map-fit').onclick=fitMap;
  $('map-expand').onclick=()=>{$('map-card').classList.toggle('expanded');$('map-expand').textContent=$('map-card').classList.contains('expanded')?'还原面板':'放大面板';drawPath();};
  $('wave-mini-toggle').onclick=()=>setWavePreview(!document.body.classList.contains('transport-wave-expanded'));
  $('wave-minimize').onclick=()=>setWaveMinimized(!document.body.classList.contains('transport-wave-icon'));
  $('map-fullscreen').onclick=async()=>{try{if(document.fullscreenElement)await document.exitFullscreen();else if($('map-card').requestFullscreen)await $('map-card').requestFullscreen();else throw new Error('不可用');}catch(e){$('map-card').classList.add('expanded');$('map-expand').textContent='还原面板';notify('浏览器未提供全屏，已放大面板；按 Esc 还原。');drawPath();}};
  document.addEventListener('fullscreenchange',drawPath);
  $('align').onclick=()=>{const x=Number($('origin-x').value),y=Number($('origin-y').value),angle=Number($('origin-angle').value);if(![x,y,angle].every(finite)){notify('配准参数必须是有限数值。',true);return;}S.alignment={x,y,angle};rebuild();fitMap();render();};
  $('apply-config').onclick=()=>applyConfig();$('reset-config').onclick=()=>applyConfig(true);
  $('save-config').onclick=saveAnalysisConfig;$('load-config').onclick=loadAnalysisConfig;
  $('wave-mode').onchange=()=>{S.wave.mode=$('wave-mode').value;S.wave.gain=1;updateWindow();drawStrip();$('wave-note').textContent=S.wave.mode==='body'?'vx / vy 目标已按 ch0 模式与航向换算到车体系；位置模式 ch12/ch13 原始值为世界系。':S.wave.mode==='wheels'?'四轮目标与实测均已记录（目标 ch27、ch35～37；实测 ch28～31），共用同一时间轴。各轨道纵轴自动定标，请对照数值比较。':'误差为目标减实测；横向偏差是目标位置投影，不等同于到规划轨迹的最短距离。虚线为自设参考值。';};
  $('wave-full').onclick=()=>{$('wave-window').value='all';setWindow(0,duration()||config.dt);};
  $('wave-window').onchange=()=>{const raw=$('wave-window').value;if(raw==='all')setWindow(0,duration()||config.dt);else{const span=Number(raw),t=S.times[S.cur]||0;setWindow(t-span*.2,t+span*.8);}};
  $('wave-x-in').onclick=()=>zoomWave(2);$('wave-x-out').onclick=()=>zoomWave(.5);
  const gain=factor=>{S.wave.gain=clamp(S.wave.gain*factor,.125,32);updateWindow();drawStrip();};
  $('wave-y-in').onclick=()=>gain(1.5);$('wave-y-out').onclick=()=>gain(1/1.5);$('wave-auto').onclick=()=>{S.wave.gain=1;updateWindow();drawStrip();};
  $('wave-apply').onclick=()=>{const a=Number($('wave-start').value),b=Number($('wave-end').value);if(!finite(a)||!finite(b)||b<=a){notify('时间窗口的终点必须大于起点。',true);return;}setWindow(a,b);};
  $('apply-tolerance').onclick=()=>{const next={lateral:Number($('tol-lateral').value),yaw:Number($('tol-yaw').value),omega:Number($('tol-omega').value)};if(!Object.values(next).every(v=>finite(v)&&v>0)){notify('参考值必须大于 0。',true);return;}S.tolerances=next;drawStability();drawStrip();};
  $('pid-axis').onchange=()=>{calculatePid();drawPid();};$('pid-apply').onclick=()=>{calculatePid();drawPid();};
  $('pid-param-read').onclick=()=>readVehiclePid();
  $('pid-param-write').onclick=()=>writeVehiclePid();
  $('pid-param-axis').onchange=selectPidParamAxis;
  $('pid-param-rollback').onclick=()=>loadPidPreset(true);
  $('pid-param-restore').onclick=()=>loadPidPreset(false);
  $('pid-motors-fill').onclick=fillAllMotorPid;
  $('pid-motors-current').onclick=()=>loadMotorPidForms('current');
  $('pid-motors-saved').onclick=()=>loadMotorPidForms('saved');
  $('pid-motors-previous').onclick=()=>loadMotorPidForms('previous');
  $('pid-motors-write').onclick=writeMotorPids;
  for(const axis of PID_MOTOR_AXES)for(const k of ['kp','ki','kd'])$('pid-motor-'+axis+'-'+k).oninput=()=>pidMotorStatus(axis,'待写入（编辑值）');
  try{const saved=JSON.parse(localStorage.getItem('mecanum-pid-presets-v1')||'{}');pidOnline.saved=Object.fromEntries(Object.entries(saved).filter(([axis,p])=>p&&validateOnlinePid(axis,p)));}catch(_){}
  renderPidParams();
  $('pid-live-start').onclick=()=>sendPidCommand('START');
  $('pid-live-stop').onclick=()=>sendPidCommand('STOP');
  $('pid-live-test').onclick=pidWireSelfTest;
  $('pid-live-download').onclick=downloadPidStream;
  $('pid-live-download-live').onclick=()=>downloadPidLive();
  $('pid-live-wheel').onchange=renderPidStream;
  $('pid-validation-file').onchange=async e=>{const file=e.target.files[0];if(file){try{loadPidValidation(await file.text(),file.name);}catch(err){notify('跨试验验证失败：'+err.message,true);}e.target.value='';}};
  $('pid-validation-clear').onclick=()=>{pidValidation=null;renderPidBurst();};
  $('pid-burst-file').onchange=async e=>{const file=e.target.files[0];if(file){try{loadPidBurst(await file.text(),file.name);}catch(err){notify('PID Burst 导入失败：'+err.message,true);}e.target.value='';}};
  $('pid-burst-analyze').onclick=()=>{if(!S.pidBurst){notify('请先导入 PID Burst CSV。',true);return;}try{const data=S.pidBurst.data;S.pidBurst.analysis=analyzePidBurst(data,{rate:Number($('pid-burst-rate').value),satLimit:Number($('pid-burst-sat').value),tolerance:Number($('pid-burst-tol').value)});renderPidBurst();}catch(err){notify('PID Burst 分析失败：'+err.message,true);}};
  $('pid-burst-clear').onclick=()=>{S.pidBurst=null;pidValidation=null;renderPidBurst();};
  $('pid-deepseek-connect').onclick=()=>connectDeepSeek();
  $('pid-deepseek').onclick=()=>requestDeepSeekAdvice();
  for(const id of ['pid-burst-rate','pid-burst-sat','pid-burst-tol','pid-burst-kp','pid-burst-ki','pid-burst-kd'])$(id).onchange=()=>{if(S.pidBurst&&id!=='pid-burst-kp'&&id!=='pid-burst-ki'&&id!=='pid-burst-kd'){$('pid-burst-analyze').click();}else if(S.pidBurst)renderPidBurst();};
  bindGestures('path','map');bindGestures('strip','wave');
  window.addEventListener('resize',()=>{drawPath();drawStrip();drawDirection(S.samples[S.cur]);if(S.tab==='pid'){renderPidStream();drawPidModelTrace();}});
  if(typeof ResizeObserver!=='undefined'){const observer=new ResizeObserver(()=>{drawPath();drawStrip();});observer.observe($('wave-wrap'));observer.observe($('path').parentElement);}
  fitMap();updateWindow();render();setWavePreview(false);
}
init();
