'use strict';
// Offline ULog43 analysis. Keep SI units internally; display angular rates in deg/s.
const $ = id => document.getElementById(id);
const WHEELS = ['fl', 'fr', 'rl', 'rr'];
const COLORS = {center:'#67baff', fl:'#ff7c8d', fr:'#ffc86a', rl:'#c4a0ff', rr:'#50dfae'};
const NAMES = {fl:'FL 左前轮', fr:'FR 右前轮', rl:'RL 左后轮', rr:'RR 右后轮'};
const DEG = 180 / Math.PI;
const DEFAULT_CHANNELS = {
  time:5, phase:1, x:32, y:33, yaw:16, imu:17, xt:21, yt:22, yawt:23,
  vx:null, vy:null, wt:14, wa:15, speedTarget:19, lateral:39, yawError:42,
  flt:27, fla:28, frt:null, fra:29, rlt:null, rla:30, rrt:null, rra:31,
  flo:null, fro:null, rlo:null, rro:null, fle:null, fre:null, rle:null, rre:null
};
const CHANNEL_LABELS = {time:'时间戳 / ms',phase:'阶段',x:'位置 X / m',y:'位置 Y / m',yaw:'底盘航向 / °',imu:'IMU 航向 / °',xt:'目标 X / m',yt:'目标 Y / m',yawt:'目标航向 / °',vx:'直接 vx / m/s',vy:'直接 vy / m/s',wt:'目标 ω / rad/s',wa:'实测 ω / rad/s',speedTarget:'目标 |v| / 0.073632',lateral:'目标横向偏差 / m',yawError:'航向误差 / °'};
for (const w of WHEELS) for (const [suffix,label] of Object.entries({t:'目标',a:'实测',o:'输出',e:'累计编码器'})) CHANNEL_LABELS[w+suffix] = `${NAMES[w]} ${label}`;
const RAW_LABELS = {0:'底盘模式（0 速度 / 1 位置）',1:'阶段（0 等待 / 1 直行 / 2 转弯 / 4 完成）',2:'方形任务状态',3:'方形任务步骤',4:'到达目标标记',5:'时间戳 ms',8:'阶段副本',14:'目标角速度 rad/s',15:'实测角速度 rad/s（旧转换日志可能未填充）',16:'底盘航向 °',17:'IMU 航向 °',18:'目标角速度 °/s',19:'平移目标速度大小 / 0.073632（无方向）',20:'四轮绝对轮速均值 / 1750 / 0.073632（不是 vx）',21:'目标 X m',22:'目标 Y m',23:'目标航向 °',24:'世界坐标 X 误差 m',25:'世界坐标 Y 误差 m',26:'航向误差 °',27:'左前轮目标 pulse/s',28:'左前轮实测 pulse/s',29:'右前轮实测 pulse/s',30:'左后轮实测 pulse/s',31:'右后轮实测 pulse/s',32:'位置 X m',33:'位置 Y m',34:'累计路程 m',35:'左前轮实测副本 pulse/s',36:'右前轮实测副本 pulse/s',39:'目标位置在车体系的横向偏差 m',41:'目标角速度副本 rad/s',42:'航向误差副本 °'};
var config = {radius:.04,ppr:440,wheelbase:.186,track:.22,dt:.05,omegaMode:'wheels',channels:{...DEFAULT_CHANNELS},metadata:{firmware:'',test:'',content:'',environment:'',battery:'',notes:''}};
var S = {rows:[],times:[],samples:[],points:[],cur:0,tab:'transport',medianDt:.05,gaps:0,name:'',
  alignment:{x:2.25,y:.15,angle:180},visible:{center:true,fl:true,fr:true,rl:true,rr:true},
  map:{x:1.2,y:1.2,spanX:2.6,spanY:2.6,zoom:1,scale:1,w:0,h:0},
  wave:{start:0,end:1,gain:1,mode:'body',w:0,h:0},
  pid:[],segments:[],selectedSegment:null,quality:null,tolerances:{lateral:.03,yaw:3,omega:10},
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
function sample(row,cfg=config) {
  const a=actual(row,cfg);return {...a,speed:Math.hypot(a.vx,a.vy),omegaDeg:a.omega*DEG,
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
const bt10={device:null,characteristic:null,writeCharacteristic:null,decoder:null,raw:'',ended:false};
const serialEcho={port:null,reader:null,writer:null,running:false};
const localSim={running:false,timer:null};
const BT10_SERVICE='0000ffe0-0000-1000-8000-00805f9b34fb';
const BT10_NOTIFY='0000ffe1-0000-1000-8000-00805f9b34fb';
const BT10_WRITE='0000ffe2-0000-1000-8000-00805f9b34fb';
function btStatus(text,kind=''){ $('bt-connect').textContent=kind==='connected'?'断开 BT10':'直接连接 BT10'; $('ws-status').textContent=text; }
function downloadBt10Log(){ if(!bt10.raw)return; const blob=new Blob([bt10.raw],{type:'text/csv;charset=utf-8'}); const a=document.createElement('a'); a.href=URL.createObjectURL(blob); a.download='bt10_'+new Date().toISOString().replace(/[:.]/g,'-')+'.csv'; a.click(); setTimeout(()=>URL.revokeObjectURL(a.href),1000); }
function bt10Notification(event){ const text=bt10.decoder.decode(event.target.value,{stream:true}); bt10.raw+=text; receiveRealtimeText(text); if(/(?:^|\r?\n)#BTLOG,END(?:,|\r?$)/m.test(bt10.raw)){ if(!bt10.ended){bt10.ended=true;downloadBt10Log();btStatus('BT10 日志已结束并已下载','connected');} } }
function bt10TestRows(){ const rows=['#BTLOG,BEGIN']; for(let i=0;i<20;i++){const t=i*100,x=i*.01,y=Math.sin(i*.2)*.1; const a=[0,1,1,0,0,t,0,0,1,0,.5,.2,.5,.2,0,0,i*.5,i*.5,0,.5385,.5385,x,y,i*.5,0,0,0,100,100,100,100,x,y,x,100,100,100,20,0,24,12,0]; rows.push(a.map(v=>typeof v==='number'&&v%1?v.toFixed(4):String(v)).join(','));} rows.push('#BTLOG,END'); return rows; }
async function sendBt10SelfTest(){ if(!bt10.writeCharacteristic){btStatus('请先连接 BT10','error');return;} const button=$('bt-selftest'); button.disabled=true; try{for(const line of bt10TestRows()){await bt10.writeCharacteristic.writeValueWithResponse(new TextEncoder().encode(line+'\r\n')); await new Promise(resolve=>setTimeout(resolve,60));} btStatus('自测已发送，等待回传','connected'); notify('BT10 自测已发送；需要 UART 回传或回环才能在网页收到测试曲线。');}catch(e){btStatus('自测发送失败：'+e.message,'error');}finally{button.disabled=false;} }
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
    bt10.device=device; bt10.characteristic=ch; bt10.writeCharacteristic=await service.getCharacteristic(BT10_WRITE); bt10.decoder=new TextDecoder(); bt10.raw=''; bt10.ended=false; resetRealtimeSession();
    ch.addEventListener('characteristicvaluechanged',bt10Notification); await ch.startNotifications();
    device.addEventListener('gattserverdisconnected',()=>{if(bt10.device===device){bt10.device=null;bt10.characteristic=null;bt10.writeCharacteristic=null;$('bt-selftest').disabled=true;btStatus('BT10 已断开','error');}});
    $('bt-selftest').disabled=false; btStatus('BT10 已连接，等待日志','connected'); notify('BT10 已连接，日志将实时显示并在 END 后自动下载。');
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
  for(const line of lines){const raw=line.trim();if(!raw)continue;if(/^#BTLOG,END\b/i.test(raw)){realtime.ended=true;realtimeStatus('已收到 END','connected');continue;}if(raw.startsWith('#')||raw.startsWith('//'))continue;
    try{const parsed=parse(raw+'\n');if(parsed.rows[0])queueRealtimeRow(parsed.rows[0]);}catch(_){/* Ignore partial/status lines from the collector. */}
  }
}
function disconnectRealtime() { if(realtime.socket){realtime.socket.onclose=null;realtime.socket.close();realtime.socket=null;}realtime.buffer='';realtime.pending=[];realtimeStatus('未连接');$('ws-connect').textContent='连接实时日志'; }
function connectRealtime() {
  if(realtime.socket){disconnectRealtime();return;}
  if(typeof WebSocket==='undefined'){realtimeStatus('当前浏览器不支持 WebSocket','error');return;}
  const url=$('ws-url').value.trim();if(!/^wss?:\/\//i.test(url)){realtimeStatus('地址必须以 ws:// 或 wss:// 开头','error');return;}
  try{const socket=new WebSocket(url);realtime.socket=socket;realtime.buffer='';realtime.ended=false;$('ws-connect').textContent='断开实时日志';realtimeStatus('连接中…');
    socket.onopen=()=>{resetRealtimeSession();realtimeStatus('已连接，等待 BT10 日志','connected');notify('已连接实时日志；等待 #BTLOG,BEGIN 和 ULog43 数据。');};
    socket.onmessage=e=>{if(typeof e.data==='string')receiveRealtimeText(e.data);else if(e.data?.text)e.data.text().then(receiveRealtimeText);};
    socket.onerror=()=>realtimeStatus('连接失败，请检查 Windows 采集器','error');
    socket.onclose=()=>{realtime.socket=null;$('ws-connect').textContent='连接实时日志';if(!realtime.ended)realtimeStatus('连接已断开','error');};
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
  return [{name:'纵向 vx',unit:'m/s',color:'#67baff',get:a=>a.vx},
    {name:'横向 vy',unit:'m/s',color:'#50dfae',get:a=>a.vy},
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
    return `<div class="motor-item"><h3 style="color:${COLORS[w]}">${NAMES[w]}</h3><div class="wheel-speed" style="color:${COLORS[w]}">${signed(v)}</div><small>轮缘线速度 m/s</small><div class="wheel-bar"><i style="left:${pct<0?50+pct:50}%;width:${Math.abs(pct)}%;background:${COLORS[w]}"></i></div><table>${tableRows([['实测',read(w+'a','pulse/s')],['目标',read(w+'t','pulse/s')],['目标 − 实测',finite(target-act)?fmt(target-act,1)+' pulse/s':'未记录'],['控制输出',read(w+'o','原始单位')],['编码器计数',read(w+'e','pulse')]])}</table></div>`;
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
  showConfig();rebuild();
  document.querySelectorAll('.tab').forEach(b=>b.onclick=()=>activateTab(b.dataset.tab));
  $('ws-connect').onclick=connectRealtime;
  $('bt-connect').onclick=connectBt10;
  $('bt-selftest').onclick=sendBt10SelfTest;
  $('serial-echo').onclick=toggleSerialEcho;
  $('local-sim').onclick=toggleLocalSim;
  $('file').onchange=async e=>{const file=e.target.files[0];if(file){try{load(await file.text(),file.name);}catch(err){notify('文件读取失败：'+err.message,true);}e.target.value='';}};
  document.addEventListener('dragover',e=>e.preventDefault());document.addEventListener('drop',async e=>{e.preventDefault();const file=e.dataTransfer?.files[0];if(file){try{load(await file.text(),file.name);}catch(err){notify('文件读取失败：'+err.message,true);}}});
  $('slider').oninput=()=>go(Number($('slider').value));$('play-toggle').onclick=()=>S.playback.running?pausePlayback():startPlayback();$('play-restart').onclick=()=>startPlayback(true);
  $('play-rate').onchange=()=>{if(S.playback.running){const p=S.playback;const t=p.anchorTime+(performance.now()-p.anchorMs)/1000*p.rate;p.anchorTime=Math.min(t,duration());p.anchorMs=performance.now();}S.playback.rate=Number($('play-rate').value);};
  document.addEventListener('visibilitychange',()=>{if(document.hidden)pausePlayback();});
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
  $('wave-mode').onchange=()=>{S.wave.mode=$('wave-mode').value;S.wave.gain=1;updateWindow();drawStrip();$('wave-note').textContent=S.wave.mode==='body'?'vx / vy 没有独立目标通道；|v| 目标只表示平移速度大小，不能当作 vx 目标。':S.wave.mode==='wheels'?'四轮实测使用同一时间轴；当前只有左前轮目标已记录。各轨道纵轴自动定标，请对照数值比较。':'误差为目标减实测；横向偏差是目标位置投影，不等同于到规划轨迹的最短距离。虚线为自设参考值。';};
  $('wave-full').onclick=()=>{$('wave-window').value='all';setWindow(0,duration()||config.dt);};
  $('wave-window').onchange=()=>{const raw=$('wave-window').value;if(raw==='all')setWindow(0,duration()||config.dt);else{const span=Number(raw),t=S.times[S.cur]||0;setWindow(t-span*.2,t+span*.8);}};
  $('wave-x-in').onclick=()=>zoomWave(2);$('wave-x-out').onclick=()=>zoomWave(.5);
  const gain=factor=>{S.wave.gain=clamp(S.wave.gain*factor,.125,32);updateWindow();drawStrip();};
  $('wave-y-in').onclick=()=>gain(1.5);$('wave-y-out').onclick=()=>gain(1/1.5);$('wave-auto').onclick=()=>{S.wave.gain=1;updateWindow();drawStrip();};
  $('wave-apply').onclick=()=>{const a=Number($('wave-start').value),b=Number($('wave-end').value);if(!finite(a)||!finite(b)||b<=a){notify('时间窗口的终点必须大于起点。',true);return;}setWindow(a,b);};
  $('apply-tolerance').onclick=()=>{const next={lateral:Number($('tol-lateral').value),yaw:Number($('tol-yaw').value),omega:Number($('tol-omega').value)};if(!Object.values(next).every(v=>finite(v)&&v>0)){notify('参考值必须大于 0。',true);return;}S.tolerances=next;drawStability();drawStrip();};
  $('pid-axis').onchange=()=>{calculatePid();drawPid();};$('pid-apply').onclick=()=>{calculatePid();drawPid();};
  bindGestures('path','map');bindGestures('strip','wave');
  window.addEventListener('resize',()=>{drawPath();drawStrip();drawDirection(S.samples[S.cur]);});
  if(typeof ResizeObserver!=='undefined'){const observer=new ResizeObserver(()=>{drawPath();drawStrip();});observer.observe($('wave-wrap'));observer.observe($('path').parentElement);}
  fitMap();updateWindow();render();setWavePreview(false);
}
init();
