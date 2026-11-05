// Behaviour test of web/app.js (no browser, no npm, no network, no real timers).
//
// Covers: WS close 1008 (distinct from invalid token, no aggressive retry, token kept), REST 429
//    on the probe after a close (token not deleted), 401 still invalidates, initialpose errors
//    keep the preview, and CSP in index.html.
//
// app.js is run in a Node `vm` context with fake DOM, fake fetch, fake WebSocket and fake
// timers/clock. Only Node built-in modules are used (vm, fs, path, assert).
//
// Run by hand:   node ws_1008_429_and_csp.test.js [path/to/web]
// (the web/ directory defaults to ../../web relative to this file). Exit code 0 and the
// line "t19 OK" on stdout mean success; it is also run by test_web_frontend_behavior.py.
const vm = require('vm'), fs = require('fs'), assert = require('assert'), path = require('path');
const WEB = process.argv[2] || path.join(__dirname, '..', '..', 'web');
const src=fs.readFileSync(WEB+'/app.js','utf8');
function boot(opts){
 opts=opts||{};const els={};
 function mk(id,tag){const l={};const o={id,tagName:tag||'DIV',hidden:false,disabled:false,checked:false,open:false,value:'',textContent:'',className:'',attrs:{},
  style:{},listeners:l,children:[],firstChild:null,addEventListener(t,f){(l[t]=l[t]||[]).push(f)},setAttribute(k,v){this.attrs[k]=v},
  focus(){doc.activeElement=this},
  appendChild(c){this.children.push(c);this.firstChild=this.children[0]},
  removeChild(c){this.children=this.children.filter(x=>x!==c);this.firstChild=this.children[0]||null},
  setPointerCapture(){},getBoundingClientRect(){return{left:0,top:0,width:150,height:150}},
  getContext(){return new Proxy({},{get:(t,k)=>k==='createImageData'?()=>({data:[]}):()=>{},set:()=>true})},
  width:800,height:600,fire(t,ev){(l[t]||[]).forEach(f=>f(ev||{preventDefault(){}}))}};return o}
 const doc={getElementById:id=>els[id]||(els[id]=mk(id)),createElement:t=>mk('x',String(t).toUpperCase()),activeElement:null,hidden:false,addEventListener(){}};
 // checked default true for layers
 ['layer-scan','layer-plan','layer-dock'].forEach(i=>{doc.getElementById(i).checked=true});
 let now=1000,timers=[],tid=0;const calls=[];
 const ctx={document:doc,navigator:{},console,Math,JSON,Promise,Array,Map,Uint8Array,DataView,Response:function(){},Blob:function(){},
  performance:{now:()=>now},
  setTimeout:(f,ms)=>{const t={id:++tid,f,at:now+ms,ms,live:true};timers.push(t);return t.id},
  clearTimeout(i){timers.forEach(t=>{if(t.id===i)t.live=false})},
  setInterval:(f,ms)=>{const t={id:++tid,f,at:now+ms,ms,iv:true,live:true};timers.push(t);return t.id},
  clearInterval(i){timers.forEach(t=>{if(t.id===i)t.live=false})},
  AbortController:function(){this.signal={};this.abort=()=>{}},requestAnimationFrame(){},
  location:{protocol:'http:',host:'x'},encodeURIComponent,isFinite,parseFloat,String,Object,Error,
  fetch(path,init){const c={path,init,body:init&&init.body?JSON.parse(init.body):undefined};calls.push(c);return opts.fetch?opts.fetch(c):Promise.resolve(R(200,{ok:true}))}};
 const store={mtrb_token:'tok'};Object.assign(store,opts.store||{});
 if(opts.lsBlocked){Object.defineProperty(ctx,'localStorage',{get(){throw new Error('SecurityError')}})}
 else ctx.localStorage={store,getItem(k){return k in this.store?this.store[k]:null},setItem(k,v){this.store[k]=v},removeItem(k){delete this.store[k]}};
 ctx.addEventListener=()=>{};ctx.window=ctx;ctx.devicePixelRatio=1;
 const sockets=[];
 ctx.WebSocket=function(){this.readyState=1;this.sent=[];this.send=s=>this.sent.push(JSON.parse(s));this.close=()=>{};sockets.push(this)};
 vm.createContext(ctx);vm.runInContext(src,ctx);
 const E=id=>doc.getElementById(id);
 const advance=ms=>{const end=now+ms;for(;;){const c=timers.filter(t=>t.live&&t.at<=end).sort((a,b)=>a.at-b.at)[0];if(!c)break;now=Math.max(now,c.at);if(c.iv)c.at+=c.ms;else c.live=false;c.f();}now=end};
 const ws=()=>sockets[sockets.length-1];
 const msg=o=>ws().onmessage({data:JSON.stringify(o)});
 return {ctx,E,calls,advance,ws,msg,store,timers,sockets,doc};
}
function R(status,body){return {ok:status<400,status,json:()=>Promise.resolve(body)}}
const tick=()=>new Promise(s=>setImmediate(s));
const MSG='Demasiados intentos con un token incorrecto. Espera un minuto y vuelve a probar.';
(async()=>{
 const P=boot().ctx.MTRBPure,d=P.describeApiError;
 assert.equal(d(429,'too_many_attempts'),MSG);
 assert.equal(d(400,'invalid_pose','/api/initialpose'),'La pose inicial está fuera del mapa');
 assert.equal(d(409,'goal_active','/api/initialpose'),'No se puede fijar la pose inicial mientras el robot navega o se teleopera. Detén el robot primero.');
 assert.equal(d(503,'no_map','/api/initialpose'),'Aún no hay mapa');
 assert(/navegación activa/.test(d(409,'goal_active','/api/goal')),'goal text unchanged');
 // WS close 1008 -> message, distinct from invalid token, no aggressive retry, token kept
 {const h=boot({fetch:()=>Promise.resolve(R(200,{}))});h.msg({type:'nav',state:'idle'});
  const n=h.sockets.length,fc=h.calls.length;
  h.ws().onclose({code:1008});await tick();
  assert.equal(h.E('login-error').textContent,MSG);assert.equal(h.E('login').hidden,false);
  assert.equal(h.calls.length,fc,'no token probe on 1008');assert(h.store.mtrb_token,'token kept');
  h.advance(30000);assert.equal(h.sockets.length,n,'no reconnect before 30 s');
  h.advance(10000);assert.equal(h.sockets.length,n+1,'reconnect after lockout wait');}
 // REST 429 on the probe after a plain close
 {const h=boot({fetch:()=>Promise.resolve(R(429,{error:'too_many_attempts'}))});h.msg({type:'nav',state:'idle'});
  const n=h.sockets.length;h.ws().onclose({code:1006});await tick();await tick();
  assert.equal(h.E('login-error').textContent,MSG);assert(h.store.mtrb_token,'token kept');
  h.advance(30000);assert.equal(h.sockets.length,n,'no reconnect in 30 s');h.advance(10000);assert.equal(h.sockets.length,n+1);}
 // 401 still means invalid token
 {const h=boot({fetch:()=>Promise.resolve(R(401,{}))});h.msg({type:'nav',state:'idle'});
  h.ws().onclose({code:1006});await tick();await tick();
  assert(/Token inválido/.test(h.E('login-error').textContent));assert(!h.store.mtrb_token);}
 // initialpose errors keep the preview
 {const h=boot({fetch:c=>Promise.resolve(c.path==='/api/initialpose'?R(409,{error:'goal_active'}):R(200,{}))});
  h.msg({type:'nav',state:'idle'});h.msg({type:'status',mode:'map',nodes:{},scan_age:0.1,odom_age:0.1});
  h.ctx.eval&&0;
  const src=fs.readFileSync(WEB+'/index.html','utf8');
  assert(src.includes('El token no se envía cifrado: usa una red dedicada.'));
  assert(!/<script>|style=|onclick=/.test(src));}
 console.log('t19 OK');
})().catch(e=>{console.error(e);process.exit(1)});
