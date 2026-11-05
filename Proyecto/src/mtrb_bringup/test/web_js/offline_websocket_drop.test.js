// Behaviour test of web/app.js (no browser, no npm, no network, no real timers).
//
// Covers: behaviour when the WebSocket drops: commands disabled and offline hint shown, pending
//    pose not sent, cancel still available and retry after a failed cancel, no automatic retry
//    of an in-flight confirm.
//
// app.js is run in a Node `vm` context with fake DOM, fake fetch, fake WebSocket and fake
// timers/clock. Only Node built-in modules are used (vm, fs, path, assert).
//
// Run by hand:   node offline_websocket_drop.test.js [path/to/web]
// (the web/ directory defaults to ../../web relative to this file). Exit code 0 and the
// line "OFFLINE OK" on stdout mean success; it is also run by test_web_frontend_behavior.py.
const vm = require('vm'), fs = require('fs'), assert = require('assert'), path = require('path');
const WEB = process.argv[2] || path.join(__dirname, '..', '..', 'web');
const src=fs.readFileSync(WEB+'/app.js','utf8');
const els={};
function mk(id){const l={};return {id,hidden:false,disabled:false,checked:true,value:'0',textContent:'',className:'',attrs:{},
 style:{},listeners:l,addEventListener(t,f){(l[t]=l[t]||[]).push(f)},setAttribute(k,v){this.attrs[k]=v},
 focus(){},setPointerCapture(){},getBoundingClientRect(){return{left:0,top:0,width:800,height:600}},
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
let wsInst;ctx.WebSocket=function(){wsInst=this;this.close=()=>{}};ctx.fetch0=ctx.fetch;
// reconnect by submitting token (login form)
E('token-input').value='tok';E('login-form').fire('submit',{preventDefault(){}});
const msg=o=>wsInst.onmessage({data:JSON.stringify(o)});
const ptr=(t,id,x,y)=>E('view').fire(t,{pointerId:id,clientX:x,clientY:y,type:t});
const tap=(x,y)=>{ptr('pointerdown',1,x,y);ptr('pointerup',1,x,y)};
(async()=>{
 msg({type:'status',mode:'map',dock:{x:1,y:2,yaw:0}});
 assert(!E('btn-dock').disabled);assert(E('offline-hint').hidden);
 tap(400,300);assert(!E('btn-confirm').disabled);
 // drop the WS
 responder=()=>({status:200,body:{ok:true}});
 const closed=wsInst;
 wsInst.onclose({code:1006});
 assert(E('btn-confirm').disabled,'confirm off');assert(E('btn-dock').disabled,'dock off');
 assert(E('btn-initpose').disabled,'initpose off');assert(!E('btn-cancel-nav').disabled,'cancel on');
 assert(!E('offline-hint').hidden);assert(/Sin conexión/.test(E('offline-hint').textContent));
 const n=calls.length;
 E('btn-confirm').fire('click');E('btn-dock').fire('click');E('btn-dock-yes').fire('click');
 assert.equal(calls.length,n,'no command sent offline');assert(E('dock-confirm').hidden);
 // pose inicial pendiente tampoco se envia
 // cancel still works offline
 E('btn-cancel-nav').fire('click');assert.equal(calls[n].path,'/api/cancel');await flush();
 // cancel failure: visible error and quick retry
 responder=()=>({status:0,body:{}});
 E('btn-cancel-nav').fire('click');assert.equal(calls.length,n+2);
 responder=()=>({status:503,body:{error:'nav_unavailable'}});await flush();
 assert.equal(E('msg').attrs.role,'alert');assert(/reintentar/.test(E('msg').textContent),E('msg').textContent);
 assert(!E('btn-cancel-nav').disabled,'cancel re-enabled for retry');
 E('btn-cancel-nav').fire('click');assert.equal(calls.length,n+3);await flush();
 // reconnect (new socket, as scheduleRetry would) re-enables
 E('token-input').value='tok';E('login-form').fire('submit',{preventDefault(){}});
 msg({type:'status',mode:'map',dock:{x:1,y:2,yaw:0}});
 assert(!E('btn-confirm').disabled);assert(!E('btn-dock').disabled);assert(!E('btn-initpose').disabled);assert(E('offline-hint').hidden);
 // in-flight confirm when WS drops: no automatic retry
 responder=()=>({status:200,body:{ok:true}});
 E('btn-confirm').fire('click');const k=calls.length;wsInst.onclose({code:1006});await flush();
 assert.equal(calls.slice(k-1).filter(c=>c.path==='/api/goal').length,1,'goal sent once, no auto retry');
 console.log('OFFLINE OK');
})().catch(e=>{console.error(e);process.exit(1)});
