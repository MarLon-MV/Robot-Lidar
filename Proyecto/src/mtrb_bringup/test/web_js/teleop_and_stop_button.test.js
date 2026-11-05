// Behaviour test of web/app.js (no browser, no npm, no network, no real timers).
//
// Covers: teleop (disabled by default, 10 Hz, proportional joystick, zero on release, seq
//    restart, blocked while nav is active, 30 s idle off, rejected/disabled messages) and PARAR
//    (keys Space/Escape, POST with WS down, retry, 3 s timeout, 503 text with physical-stop
//    warning).
//
// app.js is run in a Node `vm` context with fake DOM, fake fetch, fake WebSocket and fake
// timers/clock. Only Node built-in modules are used (vm, fs, path, assert).
//
// Run by hand:   node teleop_and_stop_button.test.js [path/to/web]
// (the web/ directory defaults to ../../web relative to this file). Exit code 0 and the
// line "t17 OK" on stdout mean success; it is also run by test_web_frontend_behavior.py.
const vm = require('vm'), fs = require('fs'), assert = require('assert'), path = require('path');
const WEB = process.argv[2] || path.join(__dirname, '..', '..', 'web');
const src=fs.readFileSync(WEB+'/app.js','utf8');
const els={};
function mk(id){const l={};return {id,hidden:false,disabled:false,checked:false,value:'0',textContent:'',className:'',attrs:{},
 style:{},listeners:l,addEventListener(t,f){(l[t]=l[t]||[]).push(f)},setAttribute(k,v){this.attrs[k]=v},
 focus(){},children:[],appendChild(c){this.children.push(c)},removeChild(){},setPointerCapture(){},getBoundingClientRect(){return{left:0,top:0,width:150,height:150}},
 getContext(){return new Proxy({},{get:(t,k)=>k==='createImageData'?()=>({data:[]}):()=>{},set:()=>true})},
 width:800,height:600,fire(t,ev){(l[t]||[]).forEach(f=>f(ev))}}}
const doc={getElementById:id=>els[id]||(els[id]=mk(id)),createElement:()=>mk('x'),activeElement:null,hidden:false,vl:{},addEventListener(t,f){(this.vl[t]=this.vl[t]||[]).push(f)}};
let now=1000;let timers=[];let tid=0;
const wl={};
const calls=[];let fetchMode='ok';
const ctx={document:doc,navigator:{},console,Math,JSON,Promise,Array,Map,Uint8Array,DataView,Response:function(){},Blob:function(){},
 performance:{now:()=>now},
 setTimeout:(f,ms)=>{const t={id:++tid,f,at:now+ms,ms,live:true};timers.push(t);return t.id},
 clearTimeout(i){timers.forEach(t=>{if(t.id===i)t.live=false})},
 setInterval:(f,ms)=>{const t={id:++tid,f,at:now+ms,ms,iv:true,live:true};timers.push(t);return t.id},
 clearInterval(i){timers.forEach(t=>{if(t.id===i)t.live=false})},
 AbortController:function(){this.signal={};this.abort=()=>{}},requestAnimationFrame(){},
 location:{protocol:'http:',host:'x'},encodeURIComponent,isFinite,parseFloat,String,Object,Error,
 localStorage:{store:{mtrb_token:'tok'},getItem(k){return this.store[k]||null},setItem(k,v){this.store[k]=v},removeItem(k){delete this.store[k]}},
 fetch(path,init){calls.push({path,init});if(fetchMode==='fail')return Promise.reject(new Error('net'));
   if(fetchMode==='503')return Promise.resolve({ok:false,status:503,json:()=>Promise.resolve({error:'nav_unavailable'})});
   if(fetchMode==='hang')return new Promise(()=>{});
   return Promise.resolve({ok:true,status:200,json:()=>Promise.resolve({ok:true})})}};
ctx.addEventListener=(t,f)=>{(wl[t]=wl[t]||[]).push(f)};ctx.window=ctx;ctx.devicePixelRatio=1;
const sockets=[];
ctx.WebSocket=function(){this.readyState=1;this.sent=[];this.send=s=>{if(this.readyState!==1)throw new Error('closed');this.sent.push(JSON.parse(s))};this.close=()=>{};sockets.push(this)};
vm.createContext(ctx);vm.runInContext(src,ctx);
const E=id=>doc.getElementById(id);
const advance=ms=>{const end=now+ms;for(;;){const c=timers.filter(t=>t.live&&t.at<=end).sort((a,b)=>a.at-b.at)[0];if(!c)break;now=Math.max(now,c.at);if(c.iv)c.at+=c.ms;else c.live=false;c.f();}now=end};
const tick=()=>new Promise(s=>setImmediate(s));
const fireW=(t,ev)=>(wl[t]||[]).forEach(f=>f(ev||{}));
const key=(t,k,x)=>fireW(t,Object.assign({key:k,target:{tagName:'BODY'},preventDefault(){}},x));
const msg=(s,o)=>s.onmessage({data:JSON.stringify(o)});
const tele=s=>s.sent.filter(m=>m.type==='teleop');
const joy=(t,id,x,y)=>E('joystick').fire(t,{pointerId:id,clientX:x,clientY:y,preventDefault(){}});
(async()=>{
 // connect
 E('token-input').value='tok';E('login-form').fire('submit',{preventDefault(){}});
 let ws=sockets[sockets.length-1];msg(ws,{type:'nav',state:'idle'});
 // default disabled
 assert.equal(E('teleop-toggle').checked,false);assert.equal(E('st-teleop').textContent,'Inactiva');
 assert(E('joystick').hidden);assert(/0,2 m\/s/.test(E('st-limits').textContent)&&/0,8 rad\/s/.test(E('st-limits').textContent),E('st-limits').textContent);
 // disabled: keys/joystick send nothing
 key('keydown','w');key('keyup','w');joy('pointerdown',1,75,0);joy('pointerup',1,75,0);assert.equal(tele(ws).length,0,'disabled sends nothing');
 // enable
 E('teleop-toggle').checked=true;E('teleop-toggle').fire('change');
 assert(!E('joystick').hidden);assert.equal(E('st-teleop').textContent,'Activa');
 // joystick: full forward-left (up-left corner) saturates
 joy('pointerdown',1,75,-500);
 let t=tele(ws);assert.equal(t.length,1,'immediate first send');
 assert.equal(t[0].vx,0.2);assert.equal(t[0].wz,0);assert.equal(t[0].seq,1);
 // 10 Hz over 1 s with fake clock
 advance(1000);t=tele(ws);assert(t.length>=10&&t.length<=12,'rate '+t.length);
 for(let i=1;i<t.length;i++)assert(t[i].seq>t[i-1].seq,'seq increasing');
 // proportional: half forward, no turn  (center 75,75 ; travel = 75-28=47)
 joy('pointermove',1,75,75-23.5);advance(100);let l=tele(ws).pop();
 assert(Math.abs(l.vx-0.1)<0.005&&Math.abs(l.wz)<1e-9,JSON.stringify(l));
 // other finger ignored
 joy('pointerdown',2,0,0);joy('pointermove',2,150,150);advance(100);l=tele(ws).pop();assert(Math.abs(l.vx-0.1)<0.005);
 // release: zero immediately (+ second zero) and loop stops
 const n0=tele(ws).length;joy('pointerup',1,0,0);
 let z=tele(ws).slice(n0);assert.equal(z.length,1);assert(z[0].vx===0&&z[0].wz===0);
 advance(60);z=tele(ws).slice(n0);assert.equal(z.length,2,'second zero');assert(z[1].seq>z[0].seq);
 advance(1000);assert.equal(tele(ws).length,n0+2,'loop stopped');
 // lost joyId sanity: pointerup of pointer 2 after not owning does nothing
 joy('pointerup',2,0,0);
 // keys
 let n=tele(ws).length;key('keydown','ArrowUp');key('keydown','a');advance(100);l=tele(ws).pop();
 assert.equal(l.vx,0.2);assert.equal(l.wz,0.8);
 key('keyup','ArrowUp');advance(100);l=tele(ws).pop();assert.equal(l.vx,0);assert.equal(l.wz,0.8);
 key('keyup','a');l=tele(ws).pop();assert(l.vx===0&&l.wz===0,'zero on last key up');advance(500);
 // blur / visibility / touchcancel / pointercancel
 for(const [name,fn] of [['blur',()=>fireW('blur')],['visibility',()=>{doc.hidden=true;(doc.vl.visibilitychange||[]).forEach(f=>f())}],['touchcancel',()=>fireW('touchcancel')],['pointercancel',()=>joy('pointercancel',1,0,0)]]){
   if(name==='pointercancel'||name==='touchcancel')joy('pointerdown',1,10,10);else key('keydown','w');
   advance(100);const b=tele(ws).length;fn();doc.hidden=false;
   const lz=tele(ws).slice(b);assert(lz.length>=1&&lz[0].vx===0&&lz[0].wz===0,name+' zero');
   advance(500);assert.equal(tele(ws).length,b+2,name+' loop stopped');
   key('keyup','w');
 }
 // no sends while held after disconnect: WS closed -> no throw, nothing sent, switch off
 key('keydown','w');advance(100);const sb=tele(ws).length;ws.readyState=3;
 ws.onclose({code:1006});advance(500);assert.equal(tele(ws).length,sb,'nothing sent after WS down');
 assert.equal(E('teleop-toggle').checked,false,'switch off on disconnect');
 assert(/conexión/.test(E('teleop-msg').textContent));
 // reconnect: seq restarts
 await tick();await tick();advance(6000);ws=sockets[sockets.length-1];assert(ws.sent.length===0);msg(ws,{type:'nav',state:'idle'});
 E('teleop-toggle').checked=true;E('teleop-toggle').fire('change');
 key('keydown','w');l=tele(ws)[0];assert.equal(l.seq,1,'seq reset on reconnect');key('keyup','w');
 // nav active blocks, and an ongoing loop stops
 key('keydown','w');advance(100);const nb=tele(ws).length;msg(ws,{type:'nav',state:'active'});advance(500);
 assert.equal(tele(ws).length,nb,'no send after nav active');
 assert.equal(E('teleop-hint').textContent,'Teleoperación bloqueada: hay una navegación activa');
 key('keyup','w');key('keydown','w');assert.equal(tele(ws).length,nb,'no send while nav active');key('keyup','w');
 msg(ws,{type:'nav',state:'failed'});key('keydown','w');assert.equal(tele(ws).length,nb,'no send on failed');key('keyup','w');
 msg(ws,{type:'nav',state:'idle'});assert.equal(E('teleop-hint').textContent,'');
 // teleop_rejected stops the loop, one message
 key('keydown','w');advance(100);const rb=tele(ws).length;
 msg(ws,{type:'error',code:'teleop_rejected'});msg(ws,{type:'error',code:'teleop_rejected'});
 assert(/otra sesión controla el robot/.test(E('teleop-msg').textContent));
 advance(500);assert.equal(tele(ws).length,rb,'loop stopped after reject');key('keyup','w');
 // teleop_disabled switches off
 msg(ws,{type:'error',code:'teleop_disabled'});assert.equal(E('teleop-toggle').checked,false);
 assert(/desactivada en el robot/.test(E('teleop-msg').textContent));
 // auto disable after 30 s idle
 advance(10000);E('teleop-toggle').checked=true;E('teleop-toggle').fire('change');
 advance(29000);assert.equal(E('teleop-toggle').checked,true);advance(1500);
 assert.equal(E('teleop-toggle').checked,false,'auto off after 30 s');assert(/inactividad/.test(E('teleop-msg').textContent));
 // PARAR: with teleop running
 E('teleop-toggle').checked=true;E('teleop-toggle').fire('change');
 key('keydown','w');advance(100);const pb=ws.sent.length,cb=calls.length;
 E('btn-stop-all').fire('click');
 assert.equal(ws.sent.length,pb+1);assert.deepEqual(ws.sent[pb],{type:'stop'});
 assert.equal(calls.length,cb+1);assert.equal(calls[cb].path,'/api/stop');assert.equal(calls[cb].init.method,'POST');
 assert.equal(calls[cb].init.headers.Authorization,'Bearer tok');assert.equal(calls[cb].init.body,undefined);
 assert.equal(E('stop-status').textContent,'Parada enviada');
 advance(500);assert.equal(ws.sent.length,pb+1,'loop cancelled by PARAR');
 key('keydown','w',{repeat:true});advance(300);assert.equal(ws.sent.length,pb+1,'key repeat does not restart');
 key('keyup','w');
 // Space / Escape
 const c2=calls.length;key('keydown',' ');key('keydown','Escape');assert.equal(calls.length,c2+2);
 // Space while typing in a text input is not PARAR
 key('keydown',' ',{target:{tagName:'INPUT',type:'password'}});assert.equal(calls.length,c2+2);
 // Space with focus on the teleop checkbox / a button: native behaviour only, no PARAR
 const c2b=calls.length,sb3=ws.sent.length;
 key('keydown',' ',{target:{tagName:'INPUT',type:'checkbox'}});key('keydown',' ',{target:{tagName:'BUTTON'}});
 assert.equal(calls.length,c2b,'space on control is not PARAR');assert.equal(ws.sent.length,sb3);
 let pd=false;key('keydown',' ',{target:{tagName:'INPUT',type:'checkbox'},preventDefault(){pd=true}});assert(!pd,'native toggle not prevented');
 key('keydown','Escape',{target:{tagName:'INPUT',type:'checkbox'}});assert.equal(calls.length,c2b+1,'Escape always PARAR');
 // WS down: PARAR still POSTs, no WS send, no throw
 ws.readyState=3;ws.onclose({code:1006});const sb2=ws.sent.length,c3=calls.length;
 E('btn-stop-all').fire('click');assert.equal(ws.sent.length,sb2);assert.equal(calls.length,c3+1);
 // POST fail -> visible error, button active, retry works
 fetchMode='fail';E('btn-stop-all').fire('click');await tick();await tick();
 assert(/pulsa de nuevo/i.test(E('stop-status').textContent),E('stop-status').textContent);assert(!E('stop-status').hidden);
 assert(/PARADA FÍSICA/.test(E('stop-status').textContent),'no WS + POST fail -> physical stop');
 assert.equal(E('btn-stop-all').disabled,false);
 // timeout 3 s
 fetchMode='hang';const c4=calls.length;E('btn-stop-all').fire('click');
 assert(timers.some(x=>x.live&&x.ms===3000),'3 s timeout');advance(3000);await tick();
 assert(/pulsa de nuevo/i.test(E('stop-status').textContent));
 // 503 (saturated) with WS down: dedicated text, alert, does not disappear, instant retry
 const SAT='Servidor saturado: la parada por red puede no haber llegado. USA LA PARADA FÍSICA del robot.';
 fetchMode='503';E('btn-stop-all').fire('click');await tick();await tick();
 assert.equal(E('stop-status').textContent,SAT);assert.equal(E('stop-status').attrs.role,'alert');assert(!E('stop-status').hidden);
 advance(60000);assert(!E('stop-status').hidden,'503 text must not vanish by itself');assert.equal(E('btn-stop-all').disabled,false);
 const c5=calls.length;E('btn-stop-all').fire('click');assert.equal(calls.length,c5+1,'instant retry');await tick();await tick();assert.equal(E('stop-status').textContent,SAT);
 // 503 with WS open: stop still sent over WS, same text
 advance(40000);const ws2=sockets[sockets.length-1];assert(ws2!==ws,'reconnected');ws2.readyState=1;msg(ws2,{type:'nav',state:'idle'});const sb5=ws2.sent.length;
 if(ws2.readyState===1){E('btn-stop-all').fire('click');await tick();await tick();
  assert.equal(ws2.sent.length,sb5+1);assert.deepEqual(ws2.sent[sb5],{type:'stop'});assert.equal(E('stop-status').textContent,SAT);}
 // network failure with WS open: no physical-stop suffix required (WS stop went out)
 fetchMode='fail';E('btn-stop-all').fire('click');await tick();await tick();
 assert(/pulsa de nuevo/i.test(E('stop-status').textContent));assert(!/PARADA FÍSICA/.test(E('stop-status').textContent));
 fetchMode='ok';E('btn-stop-all').fire('click');assert.equal(calls.length,c4+2+ (calls.length-c4-2>0?calls.length-c4-2:0));
 console.log('t17 OK');
})().catch(e=>{console.error(e);process.exit(1)});
