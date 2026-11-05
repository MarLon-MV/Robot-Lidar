// Behaviour test of web/app.js (no browser, no npm, no network, no real timers).
//
// Covers: client timeouts (cancel freed after 3 s, other commands 8 s) and the "Nav2 may keep
//    moving" warning when a goal fails or times out (409/400 races, accepted dock clears it).
//
// app.js is run in a Node `vm` context with fake DOM, fake fetch, fake WebSocket and fake
// timers/clock. Only Node built-in modules are used (vm, fs, path, assert).
//
// Run by hand:   node cancel_timeout_and_nav_warning.test.js [path/to/web]
// (the web/ directory defaults to ../../web relative to this file). Exit code 0 and the
// line "TIMEOUT+FAILED OK" on stdout mean success; it is also run by test_web_frontend_behavior.py.
const vm = require('vm'), fs = require('fs'), assert = require('assert'), path = require('path');
const WEB = process.argv[2] || path.join(__dirname, '..', '..', 'web');
const src=fs.readFileSync(WEB+'/app.js','utf8');
const els={};
function mk(id){const l={};return {id,hidden:false,disabled:false,checked:true,value:'0',textContent:'',className:'',attrs:{},
 style:{},listeners:l,addEventListener(t,f){(l[t]=l[t]||[]).push(f)},setAttribute(k,v){this.attrs[k]=v},
 focus(){},children:[],appendChild(c){c.parentNode=this;this.children.push(c)},removeChild(c){this.children=this.children.filter(x=>x!==c);c.parentNode=null},setPointerCapture(){},getBoundingClientRect(){return{left:0,top:0,width:800,height:600}},
 getContext(){return new Proxy({},{get:(t,k)=>k==='createImageData'?()=>({data:[]}):()=>{},set:()=>true})},
 width:800,height:600,fire(t,ev){(l[t]||[]).forEach(f=>f(ev))}}}
const doc={getElementById:id=>els[id]||(els[id]=mk(id)),createElement:()=>mk('x'),activeElement:null};
const timers=[];let aborted=0;const calls=[];let responder=()=>({status:200,body:{ok:true}});let resolvers=[];
const ctx={document:doc,navigator:{},console,Math,JSON,Promise,Array,Map,Uint8Array,DataView,Response:function(){},Blob:function(){},
 performance:{now:()=>1000},setTimeout:(f,ms)=>{timers.push({f,ms,live:true});return timers.length-1},clearTimeout(i){if(timers[i])timers[i].live=false},AbortController:function(){this.signal={};this.abort=()=>{aborted++}},setInterval:()=>0,requestAnimationFrame(){},
 location:{protocol:'http:',host:'x'},WebSocket:function(){this.close=()=>{}},encodeURIComponent,isFinite,parseFloat,String,Object,Error,
 localStorage:{store:{mtrb_token:'tok'},getItem(k){return this.store[k]||null},setItem(k,v){this.store[k]=v},removeItem(k){delete this.store[k]}},
 fetch(path,init){calls.push({path,init});return new Promise(res=>{resolvers.push(()=>{const r=responder(path);res({ok:r.status<400,status:r.status,json:()=>Promise.resolve(r.body)})})})}};
ctx.addEventListener=()=>{};ctx.window=ctx;ctx.window.devicePixelRatio=1;
vm.createContext(ctx);vm.runInContext(src,ctx);
const flush=async()=>{const r=resolvers;resolvers=[];r.forEach(f=>f());await new Promise(s=>setImmediate(s))};
const E=id=>doc.getElementById(id);
const ws=()=>{};
// feed status via the websocket onmessage: grab the WS instance
let wsInst;ctx.WebSocket=function(){wsInst=this;this.close=()=>{}};ctx.fetch0=ctx.fetch;
// reconnect by submitting token (login form)
E('token-input').value='tok';E('login-form').fire('submit',{preventDefault(){}});
const msg=o=>wsInst.onmessage({data:JSON.stringify(o)});
const ptr=(t,id,x,y)=>E('view').fire(t,{pointerId:id,clientX:x,clientY:y,type:t});
const tap=(x,y)=>{ptr('pointerdown',1,x,y);ptr('pointerup',1,x,y)};

const hasWarn=()=>E('st-nav').children.length>0&&/Nav2 puede seguir moviendo/.test(E('st-nav').children[0].textContent);
const fire=ms=>timers.filter(t=>t.live&&t.ms===ms).forEach(t=>{t.live=false;t.f()});
(async()=>{
 msg({type:'status',mode:'map',dock:{x:1,y:2,yaw:0}});
 // cancel never answers -> after 3 s button freed, message shown, retry immediately
 E('btn-cancel-nav').fire('click');const c0=calls.length;
 assert(E('btn-cancel-nav').disabled,'locked in flight');
 assert(timers.some(t=>t.live&&t.ms===3000),'3 s timeout for cancel');
 fire(3000);await new Promise(s=>setImmediate(s));
 assert(!E('btn-cancel-nav').disabled,'cancel freed after timeout');
 assert.equal(aborted,1,'request aborted');
 assert(/Sin respuesta del robot; pulsa de nuevo/.test(E('msg').textContent),E('msg').textContent);
 E('btn-cancel-nav').fire('click');assert.equal(calls.length,c0+1,'immediate retry sends');
 await flush(); // late/first responses arrive: must not break anything
 assert(!E('btn-cancel-nav').disabled);
 // other commands use 8 s
 msg({type:'status',mode:'map',dock:{x:1,y:2,yaw:0}});
 tap(400,300);E('btn-confirm').fire('click');
 assert(timers.some(t=>t.live&&t.ms===8000),'8 s timeout for goal');
 assert(E('btn-confirm').disabled);
 fire(8000);await new Promise(s=>setImmediate(s));
 assert(!E('btn-confirm').disabled,'confirm freed');assert(/Sin respuesta/.test(E('msg').textContent));
 // failed warning: goal timed out (not a clear rejection) -> shown
 msg({type:'nav',state:'failed'});assert(hasWarn(),'warn on failed');
 msg({type:'nav',state:'active'});assert(!hasWarn(),'warn gone when not failed');
 // 400 rejection with nothing running -> no warning
 msg({type:'nav',state:'idle'});
 responder=()=>({status:400,body:{error:'bad'}});
 tap(400,300);E('btn-confirm').fire('click');await flush();
 msg({type:'nav',state:'failed'});assert(!hasWarn(),'no warn after 400 with nothing running');
 // nav active clears flag
 msg({type:'nav',state:'active'});msg({type:'nav',state:'failed'});assert(hasWarn(),'active clears flag');
 // 409 while G1 active, then G1 fails by watchdog -> warning MUST appear
 responder=()=>({status:200,body:{ok:true}});
 tap(400,300);E('btn-confirm').fire('click');await flush();
 msg({type:'nav',state:'active'});
 responder=()=>({status:409,body:{error:'goal_active'}});
 tap(400,300);E('btn-confirm').fire('click');await flush();
 msg({type:'nav',state:'failed'});assert(hasWarn(),'warn after 409 + watchdog failed');
 // 400 while nav already failed (orphan) must not hide the warning
 responder=()=>({status:400,body:{error:'bad'}});
 tap(400,300);E('btn-confirm').fire('click');await flush();
 assert(hasWarn(),'400 during failed keeps warn');
 // rejected dock + later goal failed -> warning appears
 msg({type:'nav',state:'idle'});
 responder=()=>({status:409,body:{error:'x'}});
 E('btn-dock').fire('click');E('btn-dock-yes').fire('click');await flush();
 responder=()=>({status:200,body:{ok:true}});
 tap(400,300);E('btn-confirm').fire('click');await flush();
 msg({type:'nav',state:'failed'});assert(hasWarn(),'warn after dock rejected + goal failed');
 // accepted dock clears a stale flag
 msg({type:'nav',state:'idle'});
 responder=()=>({status:503,body:{error:'x'}});
 tap(400,300);E('btn-confirm').fire('click');await flush();
 responder=()=>({status:200,body:{ok:true}});
 E('btn-dock').fire('click');E('btn-dock-yes').fire('click');await flush();
 msg({type:'nav',state:'failed'});assert(hasWarn(),'accepted dock clears flag');
 // 400 hides warn; WS reconnects; nav failed from another tab -> warn appears
 msg({type:'nav',state:'idle'});
 responder=()=>({status:400,body:{error:'bad'}});
 tap(400,300);E('btn-confirm').fire('click');await flush();
 msg({type:'nav',state:'failed'});assert(!hasWarn(),'400 hides warn');
 E('token-input').value='tok';E('login-form').fire('submit',{preventDefault(){}});
 msg({type:'status',mode:'map',dock:{x:1,y:2,yaw:0}});
 msg({type:'nav',state:'failed'});assert(hasWarn(),'warn after reconnect');
 console.log('TIMEOUT+FAILED OK');
})().catch(e=>{console.error(e);process.exit(1)});
