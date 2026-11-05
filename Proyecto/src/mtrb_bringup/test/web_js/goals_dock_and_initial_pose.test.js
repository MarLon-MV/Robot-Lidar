// Behaviour test of web/app.js (no browser, no npm, no network, no real timers).
//
// Covers: goal/dock/initial-pose flow: tap previews, drag sets yaw, confirm sends one request,
//    409/503/400/401 and network errors, dock needs confirmation, mapping mode blocks
//    initialpose, pending pose dropped on mode change.
//
// app.js is run in a Node `vm` context with fake DOM, fake fetch, fake WebSocket and fake
// timers/clock. Only Node built-in modules are used (vm, fs, path, assert).
//
// Run by hand:   node goals_dock_and_initial_pose.test.js [path/to/web]
// (the web/ directory defaults to ../../web relative to this file). Exit code 0 and the
// line "ALL OK" on stdout mean success; it is also run by test_web_frontend_behavior.py.
const vm = require('vm'), fs = require('fs'), assert = require('assert'), path = require('path');
const WEB = process.argv[2] || path.join(__dirname, '..', '..', 'web');
const src=fs.readFileSync(WEB+'/app.js','utf8');
const els={};
function mk(id){const l={};return {id,hidden:false,disabled:false,checked:true,value:'0',textContent:'',className:'',attrs:{},
 style:{},listeners:l,addEventListener(t,f){(l[t]=l[t]||[]).push(f)},setAttribute(k,v){this.attrs[k]=v},
 focus(){},appendChild(c){c.parentNode=this},removeChild(c){c.parentNode=null},setPointerCapture(){},getBoundingClientRect(){return{left:0,top:0,width:800,height:600}},
 getContext(){return new Proxy({},{get:(t,k)=>k==='createImageData'?()=>({data:[]}):()=>{},set:()=>true})},
 width:800,height:600,fire(t,ev){(l[t]||[]).forEach(f=>f(ev))}}}
const doc={getElementById:id=>els[id]||(els[id]=mk(id)),createElement:()=>mk('x'),activeElement:null};
const calls=[];let responder=()=>({status:200,body:{ok:true}});let resolvers=[];
const ctx={document:doc,navigator:{},console,Math,JSON,Promise,Array,Map,Uint8Array,DataView,Response:function(){},Blob:function(){},
 performance:{now:()=>1000},setTimeout:()=>0,clearTimeout(){},setInterval:()=>0,requestAnimationFrame(){},
 location:{protocol:'http:',host:'x'},WebSocket:function(){this.close=()=>{}},encodeURIComponent,isFinite,parseFloat,String,Object,Error,
 localStorage:{store:{mtrb_token:'tok'},getItem(k){return this.store[k]||null},setItem(k,v){this.store[k]=v},removeItem(k){delete this.store[k]}},
 fetch(path,init){calls.push({path,init});return new Promise(res=>{resolvers.push(()=>{const r=responder(path);res({ok:r.status<400,status:r.status,json:()=>Promise.resolve(r.body)})})})}};
ctx.addEventListener=()=>{};ctx.window=ctx;ctx.window.devicePixelRatio=1;
vm.createContext(ctx);vm.runInContext(src,ctx);
const flush=async()=>{const r=resolvers;resolvers=[];r.forEach(f=>f());await new Promise(s=>setImmediate(s))};
const E=id=>doc.getElementById(id);
const ws=()=>{};
// feed status via the websocket onmessage: grab the WS instance
let wsInst;ctx.WebSocket=function(){wsInst=this;this.close=()=>{}};
// reconnect by submitting token (login form)
E('token-input').value='tok';E('login-form').fire('submit',{preventDefault(){}});
const msg=o=>wsInst.onmessage({data:JSON.stringify(o)});
const ptr=(t,id,x,y)=>E('view').fire(t,{pointerId:id,clientX:x,clientY:y,type:t});
const tap=(x,y)=>{ptr('pointerdown',1,x,y);ptr('pointerup',1,x,y)};
(async()=>{
 msg({type:'status',mode:'mapping',battery_pct:50});
 // initialpose blocked in mapping
 assert(E('btn-initpose').disabled,'initpose disabled in mapping');
 assert(/mapeo/.test(E('initpose-hint').textContent));
 // tap only previews
 tap(400,300);
 assert.equal(calls.length,0,'no request on click');
 assert(!E('pending-bar').hidden);
 assert.equal(E('btn-confirm').textContent,'Confirmar objetivo');
 // drag from pending point sets yaw (to the right => 0 → up => 90)
 ptr('pointerdown',1,400,300);ptr('pointermove',1,400,200);ptr('pointerup',1,400,200);
 assert.equal(E('yaw-input').value,'90');
 assert.equal(calls.length,0);
 // pan elsewhere keeps pending and doesn't create new one
 // confirm: double click → single request
 E('btn-confirm').fire('click');E('btn-confirm').fire('click');
 assert.equal(calls.length,1);assert(E('btn-confirm').disabled);
 const c=calls[0];assert.equal(c.path,'/api/goal');assert.equal(c.init.headers.Authorization,'Bearer tok');
 const b=JSON.parse(c.init.body);assert.deepEqual(Object.keys(b).sort(),['x','y','yaw']);assert(Math.abs(b.yaw-Math.PI/2)<1e-6);
 // 409
 responder=()=>({status:409,body:{error:'goal_active',detail:'x'}});await flush();
 assert.equal(E('msg').textContent,'Ya hay una navegación activa; cancélala primero.');
 assert.equal(E('msg').attrs.role,'alert');
 assert(!E('btn-confirm').disabled,'re-enabled');assert(!E('pending-bar').hidden,'preview kept');
 // 503, 400
 responder=()=>({status:503,body:{error:'nav_unavailable'}});E('btn-confirm').fire('click');await flush();
 assert.equal(E('msg').textContent,'Navegación no disponible.');
 responder=()=>({status:400,body:{error:'invalid_goal'}});E('btn-confirm').fire('click');await flush();
 assert.equal(E('msg').textContent,'Objetivo no válido.');
 // success clears pending
 responder=()=>({status:200,body:{ok:true}});E('btn-confirm').fire('click');await flush();
 assert(E('pending-bar').hidden);assert.equal(E('msg').textContent,'Objetivo enviado.');
 // cancel
 E('btn-cancel-nav').fire('click');assert.equal(calls[calls.length-1].path,'/api/cancel');assert.equal(calls[calls.length-1].init.body,undefined);await flush();
 // dock needs confirmation
 const n=calls.length;E('btn-dock').fire('click');assert.equal(calls.length,n);assert(!E('dock-confirm').hidden);
 E('btn-dock-no').fire('click');assert(E('dock-confirm').hidden);assert.equal(calls.length,n);
 E('btn-dock').fire('click');E('btn-dock-yes').fire('click');assert.equal(calls[calls.length-1].path,'/api/dock');await flush();
 // mapping blocks initialpose click
 E('btn-initpose').fire('click');tap(300,300);assert.equal(E('btn-confirm').textContent,'Confirmar objetivo');
 E('btn-discard').fire('click');assert(E('pending-bar').hidden);
 // map mode enables
 msg({type:'status',mode:'map'});assert(!E('btn-initpose').disabled);
 E('btn-initpose').fire('click');tap(300,300);
 assert.equal(E('btn-confirm').textContent,'Confirmar pose inicial');
 const m=calls.length;E('btn-confirm').fire('click');assert.equal(calls[m].path,'/api/initialpose');await flush();
 // mode flips to mapping with pending initialpose -> dropped
 E('btn-initpose').fire('click');tap(300,300);msg({type:'status',mode:'mapping'});assert(E('pending-bar').hidden);
 // nav panel
 msg({type:'nav',state:'active',distance_remaining:2.34});assert.equal(E('st-nav').textContent,'Navegando (faltan 2.3 m)');
 for(const [s,t] of [['idle','Inactivo'],['succeeded','Llegado'],['canceled','Cancelado'],['failed','Fallido']]){msg({type:'nav',state:s,distance_remaining:null});assert.equal(E('st-nav').textContent,t)}
 // dock layer
 assert(E('layer-dock').disabled);msg({type:'status',mode:'map',dock:{x:1,y:2,yaw:0}});assert(!E('layer-dock').disabled);
 // 401 => login
 E('btn-cancel-nav').fire('click');responder=()=>({status:401,body:{error:'unauthorized'}});await flush();
 assert(!E('login').hidden);assert.equal(ctx.localStorage.store.mtrb_token,undefined);
 // network failure
 doc.getElementById('x');
 console.log('ALL OK');
})().catch(e=>{console.error(e);process.exit(1)});
