// Strict DOM fixture + deterministic animation clock. Optional native canvas
// snapshots validate the drawing only, not browser CSS layout.
const fs=require('node:fs'),vm=require('node:vm'),assert=require('node:assert/strict');
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
const burst='timestamp_ms,fl_target,fl_actual,fl_output,fr_target,fr_actual,fr_output,rl_target,rl_actual,rl_output,rr_target,rr_actual,rr_output\n0,100,0,0,100,0,0,100,0,0,100,0,0\n100,100,90,900,100,92,920,100,88,880,100,91,910\n200,100,100,1000,100,100,1000,100,100,1000,100,100,1000\n300,100,100,500,100,100,500,100,100,500,100,100,500';const burstData=env.parsePidBurstCsv(burst),burstAnalysis=env.analyzePidBurst(burstData,{rate:100,satLimit:1000,tolerance:.02});assert.equal(burstData.rows.length,4);assert.equal(burstData.fields.fr_target,4);assert(burstAnalysis.wheels.fl.iae>0);assert.equal(burstAnalysis.wheels.fl.saturationRatio,.25);assert.equal(burstAnalysis.wheels.fl.overshoot,0);const burstCandidates=env.buildPidCandidates(burstAnalysis,{kp:2.5,ki:1.6,kd:0}).candidates;assert.equal(burstCandidates.length,3);assert(burstCandidates.every(c=>c.kp>=.05&&c.kp<=10&&c.ki>=0&&c.ki<=8&&c.kd>=0&&c.kd<=3));const aiAdvice=env.buildAiPidAdvice(burstAnalysis,{kp:2.5,ki:1.6,kd:0},{lowSpeed:true,feedforward:true});assert.equal(aiAdvice.ranked.length,3);assert(aiAdvice.confidence>=10&&aiAdvice.confidence<=95);assert(aiAdvice.ranked.every(c=>c.score>=0&&c.score<=100));
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

// Small transactional IndexedDB fixture: asynchronous request/commit, atomic
// rollback, legacy records and compound chunk keys exercise the real archiver.
(async()=>{
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
})().catch(e=>{console.error(e);process.exitCode=1;});
