// Strict DOM fixture + deterministic animation clock. Optional native canvas
// snapshots validate the drawing only, not browser CSS layout.
const fs=require('node:fs'),vm=require('node:vm'),assert=require('node:assert/strict'),cp=require('node:child_process');
const html=fs.readFileSync('kart_log_view_mecanum_v4.html','utf8');
const script=html.split('<script>')[1].split('</script>')[0];
const elements=new Map(),draws=[],nativeCanvases=new Map();let active='transport';
const canvasModule=process.env.VIEWER_CANVAS_MODULE?require(process.env.VIEWER_CANVAS_MODULE):null;
const numeric=args=>assert(args.every(Number.isFinite),'Canvas non-finite coordinates: '+args);
function contextFor(id){const c={};for(const k of ['clearRect','setTransform','moveTo','lineTo','arc','fillRect','strokeRect','rect'])c[k]=(...args)=>{numeric(args);draws.push({id,k});};for(const k of ['beginPath','stroke','fill','save','restore','clip','setLineDash'])c[k]=()=>{};c.fillText=(text,...args)=>{numeric(args);draws.push({id,text});};return c;}
function classes(initial=''){const set=new Set(initial.split(/\s+/));return {add:x=>set.add(x),remove:x=>set.delete(x),contains:x=>set.has(x),toggle(x,v){if(v===undefined)v=!set.has(x);if(v)set.add(x);else set.delete(x);return v;}};}
class Element {
 constructor(id,tag=''){
  this.id=id;this.value=tag.match(/\bvalue="([^"]*)"/)?.[1]||'';this.checked=/\bchecked\b/.test(tag);this.style={};this.textContent='';this._html='';this.listeners={};this.classList=classes(tag.match(/\bclass="([^"]*)"/)?.[1]||'');this.dataset={};for(const m of tag.matchAll(/data-([a-z]+)="([^"]+)"/g))this.dataset[m[1]]=m[2];this.nextElementSibling={textContent:''};this.parentElement={};this.context=contextFor(id);
 }
 set innerHTML(text){this._html=text;register(text);}get innerHTML(){return this._html;}
 querySelectorAll(s){return s==='button'?[...this._html.matchAll(/<button[^>]*>/g)].map(m=>new Element('',m[0])):[];}
 getBoundingClientRect(){const mini=this.id==='strip'&&document.body.classList.contains('transport-wave-mini');return {width:this.id==='path'?(active==='transport'?900:0):this.id==='direction'?90:mini?536:1100,height:this.id==='path'?490:this.id==='direction'?90:mini?270:428,left:0,top:0};}
 addEventListener(k,fn){this.listeners[k]=fn;}setPointerCapture(){}releasePointerCapture(){}
 getContext(){if(!canvasModule)return this.context;const cv=canvasModule.createCanvas(this.width,this.height);nativeCanvases.set(this.id,cv);const ctx=cv.getContext('2d'),spy=this.context;return new Proxy(ctx,{get(t,k){const v=t[k];return typeof v==='function'?(...args)=>{spy[k]?.(...args);return v.apply(t,args);}:v;},set(t,k,v){t[k]=v;return true;}});}
}
function register(text){for(const m of text.matchAll(/<[^>]+\bid="([^"]+)"[^>]*>/g))if(!elements.has(m[1]))elements.set(m[1],new Element(m[1],m[0]));}
register(html);assert.equal([...html.matchAll(/\bid="([^"]+)"/g)].length,elements.size,'No duplicate IDs');
for(const [id,v] of Object.entries({'play-rate':'1','wave-mode':'body','wave-window':'all','pid-axis':'omega','omega-mode':'wheels'}))elements.get(id).value=v;
const tabs=[...html.matchAll(/<button[^>]+data-tab="[^"]+"[^>]*>/g)].map(m=>new Element('',m[0]));
const solos=[...html.matchAll(/<button[^>]+data-solo="[^"]+"[^>]*>/g)].map(m=>new Element('',m[0]));
const document={hidden:false,listeners:{},body:{classList:classes()},getElementById(id){assert(elements.has(id),'Missing element '+id);return elements.get(id);},addEventListener(k,fn){this.listeners[k]=fn;},querySelectorAll(s){return s==='.tab'?tabs:s==='.pane'?['transport','motor','pid','channels','settings','quality'].map(id=>elements.get(id)):s==='[data-solo]'?solos:[];}};
let now=0,rafId=0;const callbacks=new Map();
const env={document,window:{devicePixelRatio:2,addEventListener(){}},innerWidth:1600,innerHeight:900,console,performance:{now:()=>now},requestAnimationFrame(cb){callbacks.set(++rafId,cb);return rafId;},cancelAnimationFrame(id){callbacks.delete(id);}};
vm.createContext(env);vm.runInContext(script,env);const el=id=>elements.get(id);
function advance(ms){now+=ms;const work=[...callbacks.values()];callbacks.clear();work.forEach(cb=>cb(now));}
function near(a,b,eps=1e-8){assert(Math.abs(a-b)<eps,`${a} != ${b}`);}
function activate(id){active=id;env.activateTab(id);}
function makeRow(ms,vx=0,vy=0,omega=0){const r=new Array(43).fill(0),L=.203,p2m=2*Math.PI*.04/440;r[5]=ms;r[14]=omega;r[15]=omega;r[19]=Math.hypot(vx,vy)/.073632;[vx-vy-L*omega,vx+vy+L*omega,vx+vy-L*omega,vx-vy+L*omega].forEach((v,i)=>r[28+i]=v/p2m);r[27]=r[28];return r;}
assert(draws.some(d=>d.text==='启停 1'),'Field visible before import');assert(el('play-toggle').disabled);
assert(document.body.classList.contains('transport-wave-mini'),'Preview is visible before any layout interaction');
for(const [vx,vy,w] of [[.4,0,0],[-.4,0,0],[0,.3,0],[0,-.3,0],[0,0,1],[0,0,-1],[.2,-.3,.7]]){const a=env.actual(makeRow(0,vx,vy,w));near(a.vx,vx);near(a.vy,vy);near(a.omega,w);near(env.sample(makeRow(0,vx,vy,w)).targetSpeed,Math.hypot(vx,vy));}
const rotating=makeRow(0,0,0,1);rotating[15]=0;near(env.actual(rotating).omega,1);near(env.sample(rotating).speed,0);
const missing=makeRow(0,.4);missing[29]=NaN;assert(Number.isNaN(env.actual(missing).vx));
const mapped=makeRow(0,.4,.1,.2);mapped[35]=101;mapped[36]=102;mapped[37]=103;
assert.equal(env.value(mapped,'flt'),mapped[27]);assert.equal(env.value(mapped,'frt'),101);assert.equal(env.value(mapped,'rlt'),102);assert.equal(env.value(mapped,'rrt'),103);
const csv=fs.readFileSync(fs.existsSync('log0_kart.csv')?'log0_kart.csv':'field_assets/logs/log0_kart.csv','utf8');env.load(csv,'log0_kart.csv');assert.equal(env.S.rows.length,2216);near(env.duration(),111.182);near(env.S.medianDt,.05);
assert(el('notice').textContent.includes('角速度通道全为 0'));assert(env.S.samples.some(s=>Math.abs(s.vy)>.05));assert(env.S.samples.some(s=>Math.abs(s.omegaDeg)>10));
const saved=env.S.rows;env.load('invalid','bad.csv');assert.equal(env.S.rows,saved);
env.load([makeRow(100),makeRow(100)].map(r=>r.join(',')).join('\n'),'reset.csv');assert.equal(env.S.rows,saved);
near(env.timeline([makeRow(100),makeRow(150),makeRow(1200)]).times[2],1.1);
near(env.timeline([makeRow(4294967280),makeRow(34)]).times[1],.05);
near(env.timeline([makeRow(0),makeRow(0)],{...env.config,channels:{...env.config.channels,time:null}}).times[1],.05);
const pose=makeRow(0);pose[32]=1;pose[33]=2;pose[16]=90;const wheel=env.wheelPos(pose,'fl');near(wheel.x,.89);near(wheel.y,2.093);
env.solo('fr');assert(env.S.visible.fr);assert.equal(Object.values(env.S.visible).filter(Boolean).length,1);el('show-all').onclick();assert(Object.values(env.S.visible).every(Boolean));
const screen=env.project(.7,1.6),world=env.unproject(...screen);near(world[0],.7);near(world[1],1.6);env.zoomMap(2,...screen);const anchored=env.project(.7,1.6);near(anchored[0],screen[0]);near(anchored[1],screen[1]);env.fitMap();
const pointer=(x,y)=>({button:0,pointerId:1,clientX:x,clientY:y,preventDefault(){}});
const oldMapX=env.S.map.x;el('path').listeners.pointerdown(pointer(300,200));el('path').listeners.pointermove(pointer(350,200));el('path').listeners.pointerup(pointer(350,200));assert(env.S.map.x<oldMapX);env.fitMap();
el('map-expand').onclick();assert(el('map-card').classList.contains('expanded'));el('map-expand').onclick();assert(!el('map-card').classList.contains('expanded'));
el('wave-mini-toggle').onclick();assert(document.body.classList.contains('transport-wave-expanded'));assert.equal(el('wave-mini-toggle').textContent,'收起波形');el('wave-mini-toggle').onclick();assert(document.body.classList.contains('transport-wave-mini'));assert.equal(el('wave-mini-toggle').textContent,'展开波形');
el('wave-minimize').onclick();assert(document.body.classList.contains('transport-wave-icon'));assert(!document.body.classList.contains('transport-wave-mini'));assert.equal(el('wave-minimize').title,'恢复波形小窗');el('wave-minimize').onclick();assert(document.body.classList.contains('transport-wave-mini'));assert(!document.body.classList.contains('transport-wave-icon'));assert.equal(el('wave-minimize').title,'缩小为波形图标');
el('wave-mini-toggle').onclick();activate('motor');assert(!document.body.classList.contains('transport-wave-expanded'));assert(!document.body.classList.contains('transport-wave-mini'));activate('transport');assert(document.body.classList.contains('transport-wave-mini'));assert.equal(el('wave-mini-toggle').textContent,'展开波形');
env.setWindow(10,20);near(env.S.wave.start,10);near(env.S.wave.end,20);assert(el('window-label').textContent.includes('199 帧'));env.zoomWave(2,15);near(env.S.wave.start,12.5);near(env.S.wave.end,17.5);
el('wave-y-in').onclick();near(env.S.wave.gain,1.5);el('wave-auto').onclick();near(env.S.wave.gain,1);const stats=env.statistics([1,-1,NaN]);near(stats.rms,1);near(stats.mean,0);assert.equal(stats.n,2);
for(const mode of ['body','wheels','errors']){el('wave-mode').value=mode;el('wave-mode').onchange();}
for(const tab of ['motor','pid','channels','settings','transport'])activate(tab);
activate('motor');assert(el('motor-cards').innerHTML.includes('未记录'));assert(!el('motor-cards').innerHTML.includes('NaN'));activate('transport');
// The real log has 50–51 ms jitter. Check one second against actual timestamps.
env.go(0);env.startPlayback();advance(1000);assert.equal(env.S.cur,19);env.pausePlayback();
const uniform=Array.from({length:2216},(_,i)=>makeRow(i*50,.2,.1,.3).join(',')).join('\n');env.load(uniform,'uniform.csv');env.setWindow(0,env.duration());env.go(0);
env.startPlayback();advance(1000);assert.equal(env.S.cur,20);assert(el('play-time').textContent.startsWith('1.00 /'));env.pausePlayback();assert.equal(callbacks.size,0);advance(1000);assert.equal(env.S.cur,20);
env.startPlayback();advance(500);assert.equal(env.S.cur,30);el('play-rate').value='2';el('play-rate').onchange();advance(1000);assert.equal(env.S.cur,70);env.go(500);advance(500);assert.equal(env.S.cur,520);
for(const rate of [.25,.5,1,2,4,8]){el('play-rate').value=String(rate);el('play-rate').onchange();env.go(0);advance(1000);assert.equal(env.S.cur,20*rate);}
env.go(2214);advance(1000);assert.equal(env.S.cur,2215);assert.equal(callbacks.size,0);assert(!env.S.playback.running);el('play-toggle').onclick();assert.equal(env.S.cur,0);assert(env.S.playback.running);document.hidden=true;document.listeners.visibilitychange();assert(!env.S.playback.running);document.hidden=false;
env.load([makeRow(0),makeRow(50),makeRow(100),makeRow(1000)].map(r=>r.join(',')).join('\n'),'gap.csv');el('play-rate').value='1';el('play-rate').onchange();env.startPlayback();advance(500);assert.equal(env.S.cur,2);advance(500);assert.equal(env.S.cur,3);assert(!env.S.playback.running);
const nan=makeRow(0);nan[16]=NaN;nan[32]=NaN;nan[33]=NaN;nan[29]=NaN;env.load(nan.join(','),'missing.csv');assert.equal(env.S.rows.length,1);assert(el('play-toggle').disabled);assert.equal(el('vx').textContent,'—');
env.load(csv,'log0_kart.csv');el('map-vx').value='24';el('map-flo').value='27';el('apply-config').onclick();near(env.actual(env.S.rows[200]).vx,env.S.rows[200][24]);assert.equal(env.config.channels.flo,27);el('reset-config').onclick();assert.equal(env.config.channels.flo,null);
env.go(2215);el('wave-mode').value='body';el('wave-mode').onchange();env.setWindow(20,35);
if(canvasModule){fs.mkdirSync('viewer/verification',{recursive:true});for(const id of ['path','strip','direction'])fs.writeFileSync('viewer/verification/'+id+'.png',nativeCanvases.get(id).toBuffer('image/png'));}
assert.equal(fs.readFileSync('kart_log_view_mecanum_v3.html','utf8'),html);
const named='time_ms,phase,x_pos,y_pos,yaw,fl_actual,fr_actual,rl_actual,rr_actual\n0,0,0,0,0,0,0,0,0\n50,1,0.1,0,0,100,100,100,100';env.load(named,'named.csv');assert.equal(env.S.quality.format,'带字段名日志');assert.equal(env.S.rows.length,2);assert.equal(env.config.channels.fla,5);assert.equal(env.config.channels.xt,null);
const burst='timestamp_ms,fl_target,fl_actual,fl_output,fr_target,fr_actual,fr_output,rl_target,rl_actual,rl_output,rr_target,rr_actual,rr_output\n0,100,0,0,100,0,0,100,0,0,100,0,0\n100,100,90,900,100,92,920,100,88,880,100,91,910\n200,100,100,1000,100,100,1000,100,100,1000,100,100,1000\n300,100,100,500,100,100,500,100,100,500,100,100,500';const burstData=env.parsePidBurstCsv(burst),burstAnalysis=env.analyzePidBurst(burstData,{rate:100,satLimit:1000,tolerance:.02});assert.equal(burstData.rows.length,4);assert.equal(burstData.fields.fr_target,4);assert(burstAnalysis.wheels.fl.iae>0);assert.equal(burstAnalysis.wheels.fl.saturationRatio,.25);assert.equal(burstAnalysis.wheels.fl.overshoot,0);const burstCandidates=env.buildPidCandidates(burstAnalysis,{kp:2.5,ki:1.6,kd:0}).candidates;assert.equal(burstCandidates.length,0);const aiAdvice=env.buildAiPidAdvice(burstAnalysis,{kp:2.5,ki:1.6,kd:0});assert.equal(aiAdvice.ranked.length,0);assert.equal(aiAdvice.confidence,0);assert(aiAdvice.warnings.length>0);

env.loadPidBurst(burst,'burst.csv');assert(env.S.pidBurst);assert(el('pid-burst-status').textContent.includes('burst.csv'));assert(el('pid-burst-metrics').innerHTML.includes('FL 左前轮'));
const rawBurst='#BTLOG,PID_BEGIN,timestamp_ms,fl_target,fl_actual\n#BTLOG,PID,0,20,0\n#BTLOG,PID,10,20,20\n#BTLOG,PID_END';const rawBurstData=env.parsePidBurstCsv(rawBurst);assert.equal(rawBurstData.rows.length,2);assert.equal(rawBurstData.fields.fl_actual,2);
console.log('PASS: firmware mapping; 7 signed kinematic cases; real 2216-frame log; zero angular placeholder; timestamp gaps/wrap/reset; missing values; wheel geometry/solo; map zoom; wave scaling/window statistics; all tabs; playback rates/pause/seek/end; remapping; v3/v4 parity.');

// Continuous live reception and the rolling cache have independent limits.
env.pidWireSelfTest();
assert.equal(vm.runInContext('pidStream.live.length',env),4000);
assert.equal(vm.runInContext('pidStream.stored.length',env),2000);
assert.equal(vm.runInContext('pidStream.complete',env),true);
assert.equal(env.S.pidBurst.data.rows.length,2000);
near(env.S.pidBurst.data.rows[0][0],10010);
near(env.S.pidBurst.data.rows[1999][0],30000);
assert.equal(vm.runInContext('pidStream.liveCount',env),6000);
const wireLines=[],wireDecoder=env.createPidWireDecoder(text=>wireLines.push(text));
if(fs.existsSync('tmp/pid_wire_test.bin')){
  const wire=fs.readFileSync('tmp/pid_wire_test.bin');
  for(let i=0;i<wire.length;i+=7)wireDecoder.feed(wire.subarray(i,i+7));
  const data=env.parsePidBurstCsv(wireLines.join(''));
  assert.equal(data.rows.length,2000);
  near(data.rows[0][1],123.4);near(data.rows[0][2],-23.4);
  near(data.rows[1999][0]-data.rows[0][0],19990);
  // A corrupt frame must not contaminate the following ASCII or binary frame.
  const start=wire.indexOf(Buffer.from([165,90,1,2]));
  const good=wire.subarray(start,start+36),bad=Buffer.from(good);bad[15]^=1;
  const out=[],d=env.createPidWireDecoder(t=>out.push(t));
  d.feed(Buffer.concat([bad,Buffer.from('#BTLOG,STATE,READY\n'),good]));
  assert(out.some(t=>t.includes('STATE,READY')));
  assert.equal(out.filter(t=>t.startsWith('#BTLOG,PID,')).length,1);
}
console.log('PASS: 30-second live / last-20-second cache protocol self-test, four-wheel plots, fragmented firmware binary/CSV parity, CRC recovery.');

if(fs.existsSync('tmp/pid_continuous_wire.bin')){
  env.resetPidStream();const lines=[],d=env.createPidWireDecoder(t=>{lines.push(t);t.trim().split(/\r?\n/).forEach(env.receivePidLine);});
  const wire=fs.readFileSync('tmp/pid_continuous_wire.bin');for(let i=0;i<wire.length;i+=181)d.feed(wire.subarray(i,i+181));
  assert.equal(vm.runInContext('pidStream.liveCount',env),72006);assert.equal(vm.runInContext('pidStream.gaps',env),0);
  assert.equal(vm.runInContext('pidStream.live.length',env),4000);
  const data=env.parsePidBurstCsv(lines.join(''));assert.equal(data.rows.length,2000);near(data.rows[0][0],342040);near(data.rows.at(-1)[0],362030);
  const live=env.parsePidBurstCsv(env.pidLiveParts(lines).join(''));assert.equal(live.rows.length,72006);near(live.rows[0][0],2005);near(live.rows.at(-1)[0],362030);
  console.log('PASS: 360-second firmware stream, 16-bit sequence wrap, all 72006 live samples exported, rolling last 20-second cache.');
}

// Execute the actual Motor.c against HAL counter/PWM stubs. No copy of its
// controller implementation is used as the parity oracle.
const gcc=process.env.PID_GCC||(fs.existsSync('D:/minGW/mingw64/bin/gcc.exe')?'D:/minGW/mingw64/bin/gcc.exe':'gcc');
fs.writeFileSync('tmp/stm32f4xx_hal.h',`
#ifndef PID_PARITY_HAL
#define PID_PARITY_HAL
#include <stdint.h>
typedef struct {int32_t cnt;uint32_t ccr[4];} TIM_HandleTypeDef;
#define TIM_CHANNEL_1 0
#define TIM_CHANNEL_2 1
#define __HAL_TIM_GET_COUNTER(t) ((t)->cnt)
#define __HAL_TIM_SET_COMPARE(t,c,v) ((t)->ccr[c]=(v))
#define HAL_TIM_PWM_Start(t,c) ((void)(t),(void)(c))
#define HAL_TIM_Encoder_Start(t,c) ((void)(t),(void)(c))
#endif
`);
fs.writeFileSync('tmp/pid_controller_parity.c',`
#include "motor.h"
#include <stdio.h>
int main(void){
 TIM_HandleTypeDef pwm={0},enc={0};Motor_t m;unsigned i;
 Motor_Init(&m,&pwm,0,8399,&pwm,1,8399,&enc,2.5f,1.6f,.004f,1,1);
 m.integral_max=5;
 for(i=0;i<8000;i++){
  unsigned phase=i/500;float target=phase%6==0?0:phase%6==1?26.3f:phase%6==2?78.8f:phase%6==3?-52.5f:phase%6==4?900.0f:-900.0f;
  Motor_SetTargetSpeed(&m,target);
  if(i%11==0)enc.cnt+=(phase%3==0?-1:1);
  {float previous=m.err_last;const MotorControlInfo *info=Motor_GetControlInfo();
  Motor_Update(&m,.001f);
  if(i%5==4)printf("%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g\\n",m.target_speed,m.actual_speed,m.integral,m.err_last,m.kick,m.output,
     m.target_speed>info->ff_min_speed?m.target_speed*info->ff_gain+info->deadzone:m.target_speed< -info->ff_min_speed?m.target_speed*info->ff_gain-info->deadzone:0,
     m.kp*m.err,m.kd*(m.err-previous)/.005f);}
 }
 return 0;
}
`);
const compile=cp.spawnSync(gcc,['-std=c99','-Wall','-Wextra','-Werror','-Itmp','-IControl','tmp/pid_controller_parity.c','Control/Motor.c','-lm','-o','tmp/pid_controller_parity.exe'],{encoding:'utf8'});
assert.equal(compile.status,0,compile.stderr);
const parityRun=cp.spawnSync('tmp/pid_controller_parity.exe',[],{encoding:'utf8'});assert.equal(parityRun.status,0,parityRun.stderr);
const parityControl={kp:2.5,ki:1.6,kd:.004,periodMs:5,integralMax:5,outputMax:1000,ffGain:.3,deadzone:150,ffMinSpeed:5,kickStep:3,kickMax:120,stuckRatio:.1,slew:30};
const parityState={integral:0,previousError:0,kick:0,output:0};let parityN=0;
for(const line of parityRun.stdout.trim().split(/\r?\n/)){
 const [r,y,integral,error,kick,output,ff,p,d]=line.split(',').map(Number);
 const v=env.pidControllerStep(parityState,r,y,parityControl,parityControl);
 for(const [a,b] of [[v.integral,integral],[parityState.previousError,error],[v.kick,kick],[v.output,output],[v.ff,ff],[v.p,p],[v.d,d]])near(a,b,.04);
 parityN++;
}
assert.equal(parityN,1600);console.log('PASS: actual Motor.c vs JS, 1600 steps: forward/reverse/zero, target ramp, P/I/D, kick, conditional integral, sign clamp, saturation and slew.');
// Independent, known plant. Generate closed-loop data without calling viewer's
// simulator; the expected plant parameters are specified separately below.
function knownPlantFixture({noise=0,delay=2,corruptTest=false}={}){
  const control={axis:'FL',revision:0,kp:.25,ki:.1,kd:0,periodMs:5,outputMax:1000,integralMax:800,ffGain:0,deadzone:0,ffMinSpeed:0,ramp:4000,slew:1000,lpf:1,kickStep:0,kickMax:0,stuckRatio:.1};
  let y=0,u=0,integral=0;const rows=[],history=Array(delay+1).fill(0),alpha=Math.exp(-.005/.12),targets=[0,80,30,-60,110,-30,55,-80];
  for(let k=0;k<1600;k++){const t=targets[Math.floor(k/200)],actual=y+noise*Math.sin(k*1.37);
    const error=t-actual;integral=Math.max(-800,Math.min(800,integral+error*.005));let next=.25*error+.1*integral;if(t&&next*t<0)next=0;u=Math.max(-1000,Math.min(1000,next));
    rows.push([k*5,t,actual,u]);history.push(u);const applied=history.shift();y=alpha*y+2*(1-alpha)*applied;if(corruptTest&&k>1280)y+=1.5*Math.sin(k*.11);
  }
  const text='timestamp_ms,fl_target,fl_actual,fl_output\n'+rows.map(r=>r.join(',')).join('\n'),a=env.analyzePidBurst(env.parsePidBurstCsv(text),{rate:200,satLimit:1000});return {control,a,text};
}
const golden=knownPlantFixture(),goldenPlant=env.identifyPidPlant(golden.a,'fl');
assert(goldenPlant.valid,JSON.stringify(goldenPlant.reasons));near(goldenPlant.model.gain,2,.12);near(goldenPlant.model.tau,.12,.04);assert(goldenPlant.test.freeRunNrmse<.05);
const goldenAdvice=env.buildAiPidAdvice(golden.a,golden.control,{control:golden.control});
assert(goldenAdvice.candidates.length>0,goldenAdvice.warnings.join(';'));assert(goldenAdvice.candidates.some(c=>c.eligible));assert(goldenAdvice.candidates.every(c=>env.pidProposalInBounds(c,golden.control,golden.control)));
assert(new Set(goldenAdvice.candidates.map(c=>c.prediction?.cost.toFixed(7))).size>1,'candidate rankings must depend on their own predicted response');
assert(goldenAdvice.candidates.filter(c=>c.eligible).every(c=>c.prediction.improvement>=2&&c.prediction.worstImprovement>=0));
const noisy=knownPlantFixture({noise:.5}),noisyPlant=env.identifyPidPlant(noisy.a,'fl');assert(noisyPlant.valid);near(noisyPlant.model.gain,2,.3);
const mismatch=knownPlantFixture({corruptTest:true});assert.equal(env.identifyPidPlant(mismatch.a,'fl').valid,false,'held-out dynamics change must not be claimed as a valid model');
const gap=knownPlantFixture();gap.a.quality.gaps=1;assert.equal(env.identifyPidPlant(gap.a,'fl').valid,false);
const missingOutput=env.analyzePidBurst(env.parsePidBurstCsv(golden.text.split('\n').map(l=>l.split(',').slice(0,3).join(',')).join('\n')),{rate:200});assert.equal(env.identifyPidPlant(missingOutput,'fl').valid,false);
const constant=env.analyzePidBurst(env.parsePidBurstCsv('timestamp_ms,fl_target,fl_actual,fl_output\n'+Array.from({length:400},(_,i)=>[i*5,100,100,30].join(',')).join('\n')),{rate:200});assert.equal(env.identifyPidPlant(constant,'fl').valid,false);
const decel=Array.from({length:200},(_,i)=>({t:i*.005,target:i<100?100:0,actual:i<100?100:100*Math.exp(-(i-100)/15),output:0}));assert(env.pidResponseMetrics(decel).overshootPct===0,'deceleration lag is not positive-step overshoot');
const transient=Array.from({length:200},(_,i)=>({t:i*.005,target:100,actual:i<10?0:i>140&&i<160?120:100,output:20}));assert(env.pidResponseMetrics(transient).settlingTime>=.8,'settling must remain within band until segment end');
const ramp=Array.from({length:100},(_,i)=>({t:i*.005,target:i*5,actual:i*5,output:i}));assert(Number.isNaN(env.pidResponseMetrics(ramp).settlingTime));
const slowRamp=ramp.map((s,i)=>({...s,target:i*.2,actual:i*.2}));assert(Number.isNaN(env.pidResponseMetrics(slowRamp).settlingTime),'cumulative target drift cannot masquerade as a flat plateau');
const downsampled=env.analyzePidBurst(env.parsePidBurstCsv(golden.text.split('\n').filter((_,i)=>i===0||i%2===1).join('\n')),{rate:100,satLimit:1000});
assert.equal(env.identifyPidPlant(downsampled,'fl').valid,false,'100Hz must never be interpolated into controller evidence');
const timestampJitter=env.analyzePidBurst(env.parsePidBurstCsv(golden.text.split('\n').map((l,i)=>i?l.replace(/^\d+/,String(Math.round((i-1)*5.06))):l).join('\n')),{rate:200,satLimit:1000});
const aligned=env.identifyPidPlant(timestampJitter,'fl');assert(aligned.valid);assert.equal(aligned.samples.length,golden.a.rows,'no invented/deleted control samples');
for(let i=0;i<golden.a.rows;i++){assert.equal(aligned.samples[i].actual,golden.a.wheels.fl.samples[i].actual);assert.equal(aligned.samples[i].observedTime,timestampJitter.wheels.fl.samples[i].t);}
const holdout=knownPlantFixture({noise:.15});assert(env.validatePidExperiment(goldenPlant,holdout.a,'fl').valid);
assert.equal(env.validatePidExperiment(goldenPlant,constant,'fl').valid,false,'constant validation cannot prove generalization');
assert.equal(env.validatePidExperiment(goldenPlant,mismatch.a,'fl').valid,false,'independent mismatch rejected without refitting');
const externalAdvice=env.buildAiPidAdvice(golden.a,golden.control,{control:golden.control,validation:mismatch.a});assert.equal(externalAdvice.candidates.length,0);assert.equal(externalAdvice.plant.valid,false);assert(env.identifyPidPlant(golden.a,'fl').valid,'external failure must not mutate cached training model');
const audit=env.auditPidController(golden.a.wheels.fl,golden.control);assert(audit.aligned);assert(audit.normalizedError<.001,'recorded u[k] follows measured y[k]');
const exactSimulation=env.simulatePidCandidate(goldenPlant,golden.control,golden.control);assert(exactSimulation);assert(exactSimulation.samples.length===golden.a.rows);
console.log('PASS: 200Hz sample preservation, 100Hz diagnosis-only, measured-before-output audit, independent holdout pass/fail and cache isolation.');
const actualSnapshot=env.encodePidUsed({...golden.control,editable:false});el('pid-param-axis').value='FL';env.loadPidBurst(actualSnapshot+golden.text,'known-plant.csv');
assert.throws(()=>env.loadPidValidation(actualSnapshot+golden.text,'same.csv'),/相同/);
env.loadPidValidation(holdout.text,'holdout.csv');assert(el('pid-validation-status').textContent.includes('跨文件预测通过'));
env.loadPidValidation(mismatch.text,'mismatch.csv');assert(el('pid-validation-status').textContent.includes('未通过'));assert.equal(vm.runInContext('pidOnline.candidates.length',env),0);
el('pid-validation-clear').onclick();assert.equal(vm.runInContext('pidValidation',env),null);assert(vm.runInContext('pidOnline.candidates.length',env)>0);
vm.runInContext('pidOnline.candidates[0].eligible=false',env);
const writeBefore=el('pid-param-kp').value;vm.runInContext('pidOnline.candidates[0].eligible=false',env);env.usePidCandidate(0);assert.equal(el('pid-param-kp').value,writeBefore,'rejected candidate cannot fill');
const evidence=env.pidEvidencePayload(env.pidAnalysisContext(),env.buildAiPidAdvice(golden.a,golden.control,{control:golden.control}));assert(evidence.series.length<=160);assert(evidence.series.every(s=>Number.isFinite(s.minActual)&&Number.isFinite(s.maxActual)));assert(evidence.controllerAudit);assert(evidence.model.test);assert.equal(evidence.axis,'FL');
fs.writeFileSync('tmp/pid_advisor_request.json',JSON.stringify(evidence));
if(process.env.PID_DIAG_CSV){
 const text=fs.readFileSync(process.env.PID_DIAG_CSV,'utf8'),a=env.analyzePidBurst(env.parsePidBurstCsv(text),{rate:200,satLimit:1000});
 assert.equal(a.quality.nonMonotonic,1);assert.equal(a.quality.issues[0].kind,'rollback_or_duplicate');
 for(const w of ['fl','fr','rl','rr'])assert.equal(env.identifyPidPlant(a,w).valid,false,'mixed reset sessions cannot produce candidates');
 assert(a.wheels.rr.samples.every(s=>s.actual===0));
 const report={rows:a.rows,quality:a.quality,wheels:Object.fromEntries(['fl','fr','rl','rr'].map(w=>[w,env.identifyPidPlant(a,w)]))};
 fs.writeFileSync('tmp/pid_review_field_diagnosis.json',JSON.stringify(report,null,2));console.log('PASS: real 24772-frame mixed-session file rejected; RR zero feedback correctly diagnosed');
}
// A real field file is optional on other computers; never store it in Git.
if(process.env.PID_FIELD_CSV){const text=fs.readFileSync(process.env.PID_FIELD_CSV,'utf8');env.loadPidBurst(text,'real-field.csv');const a=env.S.pidBurst.analysis;assert.equal(a.rows,4133);assert.equal(a.quality.gaps,0);assert.equal(a.quality.jitter,0);near(a.wheels.fr.saturationRatio,555/4133,1e-9);near(a.wheels.rl.saturationRatio,625/4133,1e-9);assert(el('pid-ai-advice').innerHTML.includes('控制输出'));}
console.log('PASS: independent delayed/noisy plant recovery; held-out mismatch rejection; per-candidate predictions/robustness; missing/gapped/constant data; deceleration/settling/ramp metrics; real parameter evidence and candidate gates.');

// PID metadata is generated by the same C parser/runtime access tested above.
const paramWire=fs.readFileSync('tmp/pid_params_wire.bin'),paramLines=[],paramDecoder=env.createPidWireDecoder(t=>paramLines.push(t.trim()));
for(let i=0;i<paramWire.length;i+=7)paramDecoder.feed(paramWire.subarray(i,i+7));
const actualParams=paramLines.filter(x=>x.startsWith('#BTLOG,PID_PARAM,')).map(env.decodePidParam);
assert(actualParams.every(Boolean));near(actualParams.find(x=>x.axis==='FL').kp,2.5);near(actualParams.find(x=>x.axis==='FL').ffGain,.3,1e-6);
const usedLines=paramLines.filter(x=>x.startsWith('#BTLOG,PID_USED,'));assert.equal(usedLines.length,7);
for(const line of usedLines)env.receivePidParamLine(line);
const metadata=env.pidRecordedParams(usedLines);assert.equal(Object.keys(metadata.axes).length,7);near(metadata.axes.FL.kp,3);near(metadata.axes.TH.kp,3.5);assert.equal(metadata.mixed.length,0);
const cached=env.parsePidBurstCsv(paramLines.join('\n'));assert.equal(cached.rows.length,30);assert.equal(env.pidRecordedParams(cached.comments).axes.FL.kp,3);
el('pid-param-axis').value='FL';env.loadPidBurst(paramLines.join('\n'),'actual-runtime.csv');
assert.equal(env.pidAnalysisContext().base.kp,3);assert.equal(env.pidAnalysisContext().known,true);assert.equal(env.pidAnalysisContext().analysis.wheels.fr.n,0);
assert(el('pid-burst-candidates').innerHTML.includes('版本 2'));
const formBefore=el('pid-param-kp').value;env.usePidCandidate(0);assert.equal(el('pid-param-axis').value,'FL');assert.equal(el('pid-param-kp').value,formBefore,'insufficient short trace must not fill an invented candidate');
assert(env.pidLiveParts([usedLines.join('\n')+'\n']).join('').includes('PID_USED'));
const second=env.encodePidUsed({...metadata.axes.FL,kp:4,revision:3});assert(env.pidRecordedParams([...usedLines,second.trim()]).mixed.includes('FL'));
console.log('PASS: actual C -> JS seven-axis readback, feedforward context, recorded baseline, selected-wheel candidates, mixed-parameter detection.');

// Small transactional IndexedDB fixture: asynchronous request/commit, atomic
// rollback, legacy records and compound chunk keys exercise the real archiver.
const asyncGuard=setTimeout(()=>{console.error('FAIL: asynchronous test stalled',vm.runInContext('pidOnline.pending&&[pidOnline.pending.id,pidOnline.pending.axis,pidOnline.pending.operation]',env));process.exitCode=1;},15000);
(async()=>{
  env.fetch=async()=>({ok:true,json:async()=>({ok:true,key_configured:true})});assert.equal(await env.connectDeepSeek(),false);assert(el('pid-deepseek').disabled);
  env.fetch=async()=>({ok:true,json:async()=>({ok:true,key_configured:true,advisor_version:2})});assert.equal(await env.connectDeepSeek(),false);
  env.fetch=async()=>({ok:true,json:async()=>({ok:true,key_configured:true,advisor_version:3})});assert.equal(await env.connectDeepSeek(),true);assert.equal(el('pid-deepseek').disabled,false);
  env.loadPidBurst(actualSnapshot+golden.text,'ai-evidence.csv');
  const approved=env.buildAiPidAdvice(env.pidAnalysisContext().analysis,golden.control,{control:golden.control}).candidates.find(c=>c.eligible);
  assert(approved);
  env.fetch=async (_url,options)=>{const sent=JSON.parse(options.body);assert(sent.model.test);assert(sent.controllerAudit);assert(sent.series.length<=160);assert.equal(sent.recordedParameters.kp,.25);return {ok:true,json:async()=>({diagnosis:'<script>not markup</script>',ranked:[],warnings:[],proposed:[approved],next_experiment:'manual low speed'})};};
  await env.requestDeepSeekAdvice();assert(el('pid-deepseek-advice').innerHTML.includes('&lt;script&gt;'));assert(vm.runInContext('pidOnline.candidates.some(c=>c.name.startsWith("AI · ")&&c.eligible)',env));
  env.fetch=async()=>({ok:true,json:async()=>({diagnosis:'bounded test',proposed:[{...approved,kp:100}],warnings:[]})});
  env.renderPidBurst();const candidateCount=vm.runInContext('pidOnline.candidates.length',env);await env.requestDeepSeekAdvice();assert.equal(vm.runInContext('pidOnline.candidates.length',env),candidateCount);
  let finishAdvice;env.fetch=()=>new Promise(resolve=>{finishAdvice=resolve;});const late=env.requestDeepSeekAdvice();el('pid-param-axis').value='FR';finishAdvice({ok:true,json:async()=>({proposed:[approved],warnings:[]})});await late;assert(el('pid-deepseek-status').textContent.includes('忽略旧'));el('pid-param-axis').value='FL';
  let finishValidationAdvice;env.fetch=()=>new Promise(resolve=>{finishValidationAdvice=resolve;});const changedValidation=env.requestDeepSeekAdvice();env.loadPidValidation(holdout.text,'new-validation.csv');finishValidationAdvice({ok:true,json:async()=>({proposed:[approved],warnings:[]})});await changedValidation;assert(el('pid-deepseek-status').textContent.includes('忽略旧'));el('pid-validation-clear').onclick();
  console.log('PASS: AI sends measured evidence; proposals checked locally; out-of-bound/stale results rejected; provider text escaped; no automatic writes.');
  const clone=x=>x===undefined?undefined:structuredClone(x),stores={logs:new Map([[1,{id:1,name:'legacy',text:'old\n',frames:1}]] )};let nextId=2,failNext=false,openedVersion=0;
  const key=(name,v)=>JSON.stringify(name==='logs'?v.id:[v.logId,v.part]);
  // Seed keys in the same representation used by the store.
  stores.logs=new Map([[JSON.stringify(1),{id:1,name:'legacy',text:'old\n',frames:1}]]);
  const db={objectStoreNames:{contains:n=>!!stores[n]},createObjectStore(n){stores[n]=new Map();return {createIndex(){}};},close(){},transaction(names,mode){
    names=Array.isArray(names)?names:[names];const local=Object.fromEntries(names.map(n=>[n,new Map([...stores[n]].map(([k,v])=>[k,clone(v)]))]));let aborted=false,scheduled=false;
    const tx={error:null,abort(){aborted=true;queueMicrotask(()=>tx.onabort?.());},objectStore(n){const m=local[n];assert(m,'Store in transaction');
      const request=fn=>{const r={};queueMicrotask(()=>{if(aborted)return;try{r.result=fn();r.onsuccess?.();}catch(e){tx.error=e;tx.abort();}});return r;};
      const select=id=>[...m.values()].filter(v=>v.logId===id);
      return {add(v){const rec=clone(v);rec.id=nextId++;return request(()=>{m.set(key(n,rec),rec);return rec.id;});},put(v){const rec=clone(v);return request(()=>m.set(key(n,rec),rec));},get(id){return request(()=>clone(m.get(JSON.stringify(id))));},getAll(){return request(()=>clone([...m.values()]));},delete(id){return request(()=>m.delete(JSON.stringify(id)));},clear(){return request(()=>m.clear());},index(){return {getAll(id){return request(()=>clone(select(id)));},openCursor(id){const values=select(id);let i=0;const r={};const step=()=>queueMicrotask(()=>{const v=values[i++];r.result=v?{delete(){m.delete(key(n,v));},continue:step}:null;r.onsuccess?.();});step();return r;}};}};
    }};
    if(!scheduled){scheduled=true;setImmediate(()=>{if(aborted)return;if(mode==='readwrite'&&failNext){failNext=false;tx.error=new Error('quota fixture');tx.abort();return;}if(mode==='readwrite')for(const n of names)stores[n]=local[n];tx.oncomplete?.();});}
    return tx;
  }};
  env.indexedDB={open(name,version){openedVersion=version;const r={result:db};queueMicrotask(()=>{r.onupgradeneeded?.();r.onsuccess?.();});return r;}};
  env.setInterval=()=>1;env.clearInterval=()=>{};env.setTimeout=()=>{};
  const downloads=[];env.downloadParts=(parts,name)=>downloads.push({text:parts.join(''),name});
  await env.archiveOpen();assert.equal(openedVersion,2);assert(stores.chunks);assert.equal((await env.archiveGet(1)).text,'old\n');
  const lines=Array.from({length:6000},(_,i)=>'#BTLOG,PID_LIVE,'+(i+1)+','+[(i+1)*5,...new Array(12).fill(i/10)].join(',')+'\n');
  await Promise.all(lines.map(env.archiveLine));await env.archiveFlush();
  const id=vm.runInContext('archive.session.id',env),rec=stores.logs.get(JSON.stringify(id));
  assert.equal(rec.liveFrames,6000);assert(!('text' in rec));assert(rec.parts>1);assert.equal((await env.archiveGet(id)).text,lines.join(''));
  await env.downloadPidLive();const exported=env.parsePidBurstCsv(downloads.at(-1).text);assert.equal(exported.rows.length,6000);near(exported.rows.at(-1)[0],30000);
  const extra='#BTLOG,PID_LIVE,6001,30005,'+new Array(12).fill(1).join(',')+'\n';await env.archiveLine(extra);
  failNext=true;await env.archiveFlush();assert.equal(vm.runInContext('archive.pending',env),extra);assert(vm.runInContext('archive.error',env).includes('quota'));
  await env.archiveFlush();assert.equal(vm.runInContext('archive.pending',env),'');assert.equal((await env.archiveGet(id)).text,lines.join('')+extra);
  await env.archiveFinish(false);
  // END download must select the new session after its queued opening, not the previous id.
  const opening=env.archiveLine('#BTLOG,BEGIN\n'),ending=env.archiveFinish(true),download=env.downloadBt10Log();await Promise.all([opening,ending,download]);
  assert.equal(downloads.at(-1).text,'#BTLOG,BEGIN\n');assert.notEqual(vm.runInContext('archive.lastId',env),id);
  const fragmented=env.pidLiveParts([extra.slice(0,18),extra.slice(18).trimEnd()]);assert.equal(env.parsePidBurstCsv(fragmented.join('')).rows.length,1);
  await env.archiveDelete(id);assert(!stores.logs.has(JSON.stringify(id)));assert(![...stores.chunks.values()].some(v=>v.logId===id));assert.equal((await env.archiveGet(1)).text,'old\n');
  const originalOpen=env.indexedDB.open;vm.runInContext('archive.db=null',env);env.indexedDB.open=()=>{throw new Error('open fixture');};
  await env.archiveLine(extra);assert.equal(vm.runInContext('archive.pending',env),extra);
  env.indexedDB.open=originalOpen;await env.archiveFlush();assert.equal(vm.runInContext('archive.session.rec.liveFrames',env),1);
  assert.equal((await env.archiveGet(vm.runInContext('archive.session.id',env))).text,extra);await env.archiveFinish(false);
  console.log('PASS: IndexedDB v1 migration; chunked 6000-frame live archive/export; atomic failure retry without duplication; queued END session selection; delete chunks; legacy import.');
  // Simulate a BLE transport using actual firmware response structure. Commands
  // must be serialized across 20-byte GATT writes, matched by id, then read back.
  env.TextEncoder=TextEncoder;const timers=new Map();let timerId=0;
  env.setTimeout=fn=>{timers.set(++timerId,fn);return timerId;};env.clearTimeout=id=>timers.delete(id);
  const local=new Map();env.localStorage={getItem:k=>local.get(k)||null,setItem:(k,v)=>local.set(k,v)};
  const model=Object.fromEntries(Object.entries(metadata.axes).map(([axis,p])=>[axis,{...p,editable:true}]));let revision=2,input='',setCount=0,parked=true,corruptReadback=false,suppress=false,blockedAxis=null,rejectSetAxis=null,dropSetAxis=null,changeFinal=false;
  const wireDecoder=env.createPidWireDecoder(env.bt10DecodedText),commands=[];
  env.pidMockWrite=async bytes=>{
    assert(bytes.length<=20);input+=Buffer.from(bytes).toString();if(!input.endsWith('\n'))return;
    const t=input.trim().split(' ');commands.push(input);input='';const op=t[1],id=Number(t[2]),axis=t[op==='SET'?4:3];
    if(suppress)return;
    if(op==='SET'){
      if(axis===rejectSetAxis){wireDecoder.feed(Buffer.from('#BTLOG,PID_ERROR,'+id+',STALE_REVISION\n'));return;}
      assert.equal(Number(t[3]),revision);assert(parked);setCount++;revision++;
      ['kp','ki','kd'].forEach((k,i)=>model[axis][k]=Number(t[i+5]));
      if(axis===dropSetAxis)return; // The MCU applied SET but its reply was lost.
    }
    if(changeFinal&&op==='GET'&&axis==='FL'&&commands.at(-2)?.includes(' GET ')&&commands.at(-2)?.trim().endsWith('RR')){revision++;changeFinal=false;}
    let value={...model[axis],editable:parked&&axis!==blockedAxis,revision};if(corruptReadback&&op==='GET'&&setCount>1)value.kp+=.5;
    const response=env.encodePidUsed(value).replace('PID_USED,0,','PID_PARAM,'+id+',').replace(',READ,',','+op.replace('GET','READ')+',');
    const data=Buffer.from(response);for(let i=0;i<data.length;i+=7)wireDecoder.feed(data.subarray(i,i+7));
  };
  vm.runInContext('bt10.device={gatt:{connected:true}};bt10.writeCharacteristic={writeValueWithResponse:pidMockWrite};pidOnline.current={};',env);
  await env.readVehiclePid();assert.equal(vm.runInContext('Object.keys(pidOnline.current).length',env),7);assert.equal(el('pid-param-kp').value,'3');
  el('pid-param-kp').value='2.7';el('pid-param-ki').value='1.5';el('pid-param-kd').value='0';
  const applied=await env.writeVehiclePid();assert.equal(applied.kp,2.7);assert.equal(setCount,1);assert(el('pid-param-status').textContent.includes('读回确认'));
  assert(local.get('mecanum-pid-presets-v1').includes('2.7'));env.loadPidPreset(true);assert.equal(el('pid-param-kp').value,'3');
  // Running and invalid values cannot send a SET; mismatched readback isn't success.
  parked=false;await env.writeVehiclePid();assert.equal(setCount,1);parked=true;
  el('pid-param-kp').value='NaN';await env.writeVehiclePid();assert.equal(setCount,1);
  el('pid-param-kp').value='2.8';corruptReadback=true;assert.equal(await env.writeVehiclePid(),null);assert.equal(setCount,2);assert(el('pid-param-status').textContent.includes('不匹配'));corruptReadback=false;
  // Four-wheel editing uses the original single-axis protocol, never an implied
  // atomic update. Validate everything first; report partial/unknown outcomes.
  const motors=['FL','FR','RL','RR'],gains=['kp','ki','kd'];
  const fillMotor=(axis,p)=>gains.forEach(k=>el('pid-motor-'+axis+'-'+k).value=String(p[k]));
  const checkMotor=(axis,p)=>gains.forEach(k=>assert.equal(Number(el('pid-motor-'+axis+'-'+k).value),p[k]));
  await env.readVehiclePid();env.loadMotorPidForms('current');motors.forEach(axis=>checkMotor(axis,model[axis]));
  const beforeFill=commands.length;
  el('pid-motors-kp').value='2.6';el('pid-motors-ki').value='1.4';el('pid-motors-kd').value='0';el('pid-motors-fill').onclick();
  motors.forEach(axis=>checkMotor(axis,{kp:2.6,ki:1.4,kd:0}));assert.equal(commands.length,beforeFill,'unified fill never sends data');
  el('pid-motors-kp').value='11';env.fillAllMotorPid();motors.forEach(axis=>checkMotor(axis,{kp:2.6,ki:1.4,kd:0}));
  env.loadMotorPidForms('saved');motors.forEach(axis=>checkMotor(axis,{kp:2.6,ki:1.4,kd:0})); // Incomplete preset: no partial fill.
  const requested=Object.fromEntries(motors.map((axis,i)=>[axis,{kp:2.2+i*.1,ki:1+i*.1,kd:i*.001}]));
  motors.forEach(axis=>fillMotor(axis,requested[axis]));el('pid-motor-RR-ki').value='';
  assert.equal(await env.writeMotorPids(),null);assert.equal(commands.length,beforeFill,'bad last row cannot write the first three motors');
  fillMotor('RR',requested.RR);blockedAxis='RR';const beforePreflight=setCount;
  assert.equal(await env.writeMotorPids(),null);assert.equal(setCount,beforePreflight);assert(el('pid-param-status').textContent.includes('未发送任何写入'));blockedAxis=null;
  const savedPositions=JSON.stringify([model.X,model.Y,model.TH]),beforeBatch=Object.fromEntries(motors.map(axis=>[axis,{...model[axis]}]));
  const batchStart=commands.length,writing=el('pid-motors-write').onclick();assert(el('pid-motors-write').disabled);assert(el('pid-motor-FL-kp').disabled);assert(el('pid-param-write').disabled);
  assert.equal(await env.writeMotorPids(),undefined,'no concurrent batch');assert(await writing);assert(!el('pid-motors-write').disabled);
  assert.equal(commands.length-batchStart,16);assert(commands.slice(batchStart,batchStart+4).every(c=>c.includes(' GET ')));
  assert(commands.slice(-4).every(c=>c.includes(' GET ')),'a complete final revision sweep is required');
  motors.forEach(axis=>{gains.forEach(k=>near(model[axis][k],requested[axis][k]));assert(el('pid-motor-'+axis+'-status').textContent.includes('整组读回确认'));});
  assert.equal(JSON.stringify([model.X,model.Y,model.TH]),savedPositions,'four-wheel writes do not touch position controllers');
  const motorPreset=JSON.parse(local.get('mecanum-pid-presets-v1'));motors.forEach(axis=>gains.forEach(k=>near(motorPreset[axis][k],requested[axis][k])));
  env.loadMotorPidForms('previous');motors.forEach(axis=>checkMotor(axis,beforeBatch[axis]));env.loadMotorPidForms('saved');motors.forEach(axis=>gains.forEach(k=>near(Number(el('pid-motor-'+axis+'-'+k).value),requested[axis][k])));
  // A rejected third SET stops the fourth and requires fresh reads; no rollback.
  const partialStart=commands.length,partialSets=setCount;rejectSetAxis='RL';
  assert.equal(await env.writeMotorPids(),null);assert.equal(setCount,partialSets+2);assert(!commands.slice(partialStart).some(c=>c.includes(' SET ')&&c.split(' ')[4]==='RR'));
  assert(el('pid-motor-FL-status').textContent.includes('此前已逐轮读回'));assert(el('pid-motor-RL-status').textContent.includes('未确认'));assert(el('pid-motor-RR-status').textContent.includes('本轮未写入'));assert(el('pid-param-status').textContent.includes('可能部分生效'));
  assert.equal(vm.runInContext('Object.keys(pidOnline.current).length',env),0);rejectSetAxis=null;
  // Lost SET ACK may mean the parameter is already applied: never retry it.
  const lostStart=commands.length,lostSets=setCount;dropSetAxis='FR';const lost=env.writeMotorPids();
  for(let i=0;i<100&&!vm.runInContext('pidOnline.pending?.operation==="SET"&&pidOnline.pending?.axis==="FR"',env);i++)await new Promise(setImmediate);
  assert(vm.runInContext('pidOnline.pending?.operation==="SET"&&pidOnline.pending?.axis==="FR"',env));[...timers.values()].forEach(fn=>fn());assert.equal(await lost,null);
  assert.equal(setCount,lostSets+2);assert.equal(commands.slice(lostStart).filter(c=>c.includes(' SET ')&&c.split(' ')[4]==='FR').length,1);assert(el('pid-motor-FR-status').textContent.includes('未确认'));assert(el('pid-motor-RL-status').textContent.includes('本轮未写入'));dropSetAxis=null;
  // Even after all four individual confirmations, an intervening revision at
  // the final sweep prevents a false whole-group success.
  changeFinal=true;assert.equal(await env.writeMotorPids(),null);assert(el('pid-param-status').textContent.includes('整组复核'));motors.forEach(axis=>assert(el('pid-motor-'+axis+'-status').textContent.includes('最终结果未确认')));
  await env.readVehiclePid();env.loadMotorPidForms('current');motors.forEach(axis=>checkMotor(axis,model[axis]));
  console.log('PASS: four-wheel unified/individual edits; complete validation and parked preflight; serialized writes and final revision sweep; presets; partial rejection/lost ACK/external change stop without retries or rollback.');
  // No automatic retransmission after an uncertain response.
  suppress=true;const timed=env.pidExchange('GET','FL');await new Promise(setImmediate);const pendingId=vm.runInContext('pidOnline.pending.id',env);const count=commands.length;
  [...timers.values()].forEach(fn=>fn());await assert.rejects(timed,/超时/);await new Promise(setImmediate);assert.equal(commands.length,count);
  env.receivePidParamLine(env.encodePidUsed({...model.FL,revision}).replace('PID_USED,0,','PID_PARAM,'+pendingId+','));assert.equal(vm.runInContext('Object.keys(pidOnline.current).length',env),0);
  suppress=false;await env.readVehiclePid();env.pidParamDisconnected();assert.equal(vm.runInContext('Object.keys(pidOnline.current).length',env),0);
  console.log('PASS: BLE 20-byte request serialization; seven reads; SET then GET verification; RAM preset/rollback; running/NaN rejection; mismatch/timeout/late-ACK/disconnect handling.');

})().catch(e=>{console.error(e);process.exitCode=1;}).finally(()=>clearTimeout(asyncGuard));
