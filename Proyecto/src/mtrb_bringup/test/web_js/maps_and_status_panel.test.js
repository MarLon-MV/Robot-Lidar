// Behaviour test of web/app.js (no browser, no npm, no network, no real timers).
//
// Covers: map saving (validation, double click, overwrite:true only after explicit confirmation,
//    error codes), maps list rendered as text (hostile names), status panel, layer toggles and
//    storage failures.
//
// app.js is run in a Node `vm` context with fake DOM, fake fetch, fake WebSocket and fake
// timers/clock. Only Node built-in modules are used (vm, fs, path, assert).
//
// Run by hand:   node maps_and_status_panel.test.js [path/to/web]
// (the web/ directory defaults to ../../web relative to this file). Exit code 0 and the
// line "t18 OK" on stdout mean success; it is also run by test_web_frontend_behavior.py.
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
const saves=h=>h.calls.filter(c=>c.path==='/api/map/save');
const getMaps=h=>h.calls.filter(c=>c.path==='/api/maps');
const status=(mode,extra)=>Object.assign({type:'status',mode,battery_pct:71.4,nodes:{slam_toolbox:'active'},scan_age:0.2,odom_age:0.1,dock:null},extra||{});
function connect(h,mode){h.E('token-input').value='tok';h.E('login-form').fire('submit');h.msg({type:'nav',state:'idle'});if(mode)h.msg(status(mode))}
function typeName(h,v){h.E('map-name').value=v;h.E('map-name').fire('input')}

(async()=>{
 // ---- pure helpers ----
 {const h=boot();const P=h.ctx.MTRBPure;
  for(const ok of ['a','A_b-9','x'.repeat(40)])assert(P.isValidMapName(ok),ok);
  for(const bad of ['','x'.repeat(41),'-a','-','map','MAP','Map','a b','a/b','../x','ñu','a.b','<script>',"a'b",'a\n',null,5])assert(!P.isValidMapName(bad),String(bad));
  const e=P.describeSaveError;
  assert(/modo mapeo/.test(e(409,'wrong_mode')));assert(/guardado en curso/.test(e(409,'save_in_progress')));
  assert(/SLAM no disponible/.test(e(503,'slam_unavailable')));
  assert(/si se guardó/.test(e(504,'save_timeout')));assert(/si se guardó/.test(e(500,'save_timeout')));
  assert(/Nombre no válido/.test(e(400,'invalid_name'))&&/Nombre reservado/.test(e(400,'reserved_name'))&&/Revisa o elimina los archivos \.bak/.test(e(409,'backup_pending'))&&/demasiados mapas/.test(e(409,'map_limit'))&&/espacio libre/.test(e(507,'insufficient_storage'))&&/Espera unos segundos/.test(e(429,'save_rate_limited'))&&/empezar por guion/.test(P.mapNameProblem('-x'))&&/reservado/.test(P.mapNameProblem('MAP'))&&P.mapNameProblem('ok')==='');assert(/token/.test(e(401,'unauthorized')));
  assert(/No se pudo contactar/.test(e(0,undefined)));assert(/Sin respuesta del robot; comprueba en «Mapas guardados» si se guardó/.test(e(0,undefined,true)));
  for(const c of [[409,'wrong_mode'],[409,'save_in_progress'],[503,'slam_unavailable'],[504,'save_timeout'],[400,'invalid_name'],[401,'x'],[0,'']])assert(!/error \d|undefined|null/.test(e(c[0],c[1])));
 }
 // ---- button state: map mode / no WS / mapping ----
 {const h=boot();const B=()=>h.E('btn-save-map');
  assert(B().disabled||true);
  connect(h);                         // connected, no status yet
  assert(B().disabled,'disabled without status');
  h.msg(status('map'));assert(B().disabled,'disabled in map');
  assert(/modo mapeo/.test(h.E('map-save-hint').textContent),h.E('map-save-hint').textContent);
  typeName(h,'okname');B().fire('click');assert.equal(saves(h).length,0,'no request from disabled state via direct call');
  h.E('map-form').fire('submit');assert.equal(saves(h).length,0,'Enter in map mode sends nothing');
  h.msg(status('mapping'));assert(!B().disabled,'enabled in mapping');assert.equal(h.E('map-save-hint').textContent,'');
  // WS lost
  h.ws().readyState=3;h.ws().onclose({code:1006});assert(B().disabled,'disabled without WS');
  assert(/Sin conexión/.test(h.E('map-save-hint').textContent));
  h.E('map-form').fire('submit');assert.equal(saves(h).length,0);
 }
 // ---- validation in client ----
 {const h=boot();connect(h,'mapping');
  typeName(h,'');h.E('btn-save-map').fire('click');assert.equal(saves(h).length,0);assert(!h.E('map-name-error').hidden);
  assert(/Escribe un nombre/.test(h.E('map-name-error').textContent));
  typeName(h,'mal nombre');h.E('btn-save-map').fire('click');assert.equal(saves(h).length,0);
  assert(/Nombre no válido/.test(h.E('map-name-error').textContent));assert.equal(h.E('map-name').attrs['aria-invalid'],'true');
  typeName(h,'bien');assert(h.E('map-name-error').hidden,'error cleared on input');
  typeName(h,'-abc');h.E('btn-save-map').fire('click');assert.equal(saves(h).length,0);assert(/empezar por guion/.test(h.E('map-name-error').textContent));
  for(const r of ['map','MAP','Map']){typeName(h,r);h.E('btn-save-map').fire('click');assert.equal(saves(h).length,0,r);assert(/reservado/.test(h.E('map-name-error').textContent));}
  typeName(h,'x'.repeat(41));h.E('btn-save-map').fire('click');assert.equal(saves(h).length,0);
 }
 // ---- double click = one request; Guardando…; success + list refresh ----
 {let resolve;const h=boot({fetch:c=>c.path==='/api/map/save'?new Promise(r=>{resolve=r}):Promise.resolve(R(200,{maps:['a']}))});
  connect(h,'mapping');typeName(h,'mapa_1');
  h.E('btn-save-map').fire('click');h.E('btn-save-map').fire('click');h.E('map-form').fire('submit');
  assert.equal(saves(h).length,1,'single request');
  assert.deepEqual(saves(h)[0].body,{name:'mapa_1'});assert.equal(saves(h)[0].init.method,'POST');
  assert.equal(saves(h)[0].init.headers.Authorization,'Bearer tok');
  assert.equal(h.E('btn-save-map').textContent,'Guardando…');assert(h.E('btn-save-map').disabled);
  assert(h.timers.some(t=>t.live&&t.ms===40000),'40 s timeout');
  resolve(R(200,{ok:true,name:'mapa_1',posegraph:true}));await tick();await tick();await tick();
  assert.equal(h.E('map-save-msg').textContent,'Guardado: mapa_1');assert(!h.E('btn-save-map').disabled);
  assert.equal(h.E('btn-save-map').textContent,'Guardar mapa');assert.equal(getMaps(h).length,1,'list refreshed after save');
 }
 // ---- posegraph:false ----
 {const h=boot({fetch:c=>Promise.resolve(c.path==='/api/map/save'?R(200,{ok:true,name:'p',posegraph:false}):R(200,{maps:[]}))});
  connect(h,'mapping');typeName(h,'p');h.E('btn-save-map').fire('click');await tick();await tick();await tick();
  const t=h.E('map-save-msg').textContent;assert(/Guardado: p/.test(t)&&/no se pudo guardar el posegraph \(no podrás ampliarlo\)\. Si estabas sobrescribiendo, el posegraph anterior se ha eliminado/.test(t),t);
 }
 // ---- 409 map_exists -> confirm -> No / Sí ----
 {const h=boot({fetch:c=>Promise.resolve(c.path==='/api/map/save'?(c.body.overwrite?R(200,{ok:true,name:c.body.name,posegraph:true}):R(409,{error:'map_exists'})):R(200,{maps:['lab']}))});
  connect(h,'mapping');typeName(h,'lab');h.E('btn-save-map').fire('click');await tick();await tick();
  assert.equal(saves(h).length,1);assert(!('overwrite' in saves(h)[0].body),'first request has no overwrite');
  assert(!h.E('overwrite-confirm').hidden);assert.equal(h.E('overwrite-text').textContent,'Ya existe un mapa llamado lab. ¿Sobrescribir?');
  assert(/>Sí, sobrescribir</.test(fs.readFileSync(WEB+'/index.html','utf8')));assert(h.E('btn-save-map').disabled,'save locked while asking');
  assert.equal(h.doc.activeElement,h.E('btn-overwrite-no'),'focus on the safe option');
  // No: nothing sent
  h.E('btn-overwrite-no').fire('click');await tick();assert.equal(saves(h).length,1);assert(h.E('overwrite-confirm').hidden);
  assert(!h.E('btn-save-map').disabled);
  // Yes button with no pending confirmation does nothing
  h.E('btn-overwrite-yes').fire('click');assert.equal(saves(h).length,1,'no overwrite without confirmation');
  // again: ask, then edit name -> confirmation void
  h.E('btn-save-map').fire('click');await tick();await tick();assert(!h.E('overwrite-confirm').hidden);
  typeName(h,'lab2');assert(h.E('overwrite-confirm').hidden);h.E('btn-overwrite-yes').fire('click');
  assert.equal(saves(h).filter(c=>c.body.overwrite).length,0,'never overwrite after name change');
  // ask and confirm
  typeName(h,'lab');h.E('btn-save-map').fire('click');await tick();await tick();
  const n=saves(h).length;h.E('btn-overwrite-yes').fire('click');h.E('btn-overwrite-yes').fire('click');
  assert.equal(saves(h).length,n+1,'one overwrite request');assert.deepEqual(saves(h)[n].body,{name:'lab',overwrite:true});
  await tick();await tick();await tick();assert.equal(h.E('map-save-msg').textContent,'Guardado: lab');assert(h.E('overwrite-confirm').hidden);
 }
 // ---- confirmation dropped if mode/connection changes ----
 {const h=boot({fetch:c=>Promise.resolve(c.path==='/api/map/save'?R(409,{error:'map_exists'}):R(200,{maps:[]}))});
  connect(h,'mapping');typeName(h,'x');h.E('btn-save-map').fire('click');await tick();await tick();assert(!h.E('overwrite-confirm').hidden);
  h.msg(status('map'));assert(h.E('overwrite-confirm').hidden);h.E('btn-overwrite-yes').fire('click');
  assert.equal(saves(h).filter(c=>c.body.overwrite).length,0);
 }
 // ---- each error code ----
 const cases=[[409,'wrong_mode',/modo mapeo/],[409,'save_in_progress',/guardado en curso/],[503,'slam_unavailable',/SLAM no disponible/],
  [504,'save_timeout',/si se guardó/],[500,'save_timeout',/si se guardó/],[400,'invalid_name',/Nombre no válido/],[400,'invalid_body',/Petición no válida/],[400,'reserved_name',/Nombre reservado/],[409,'backup_pending',/respaldo anterior sin resolver.*\.bak/],[409,'map_limit',/demasiados mapas/],[507,'insufficient_storage',/espacio libre/],[429,'save_rate_limited',/Espera unos segundos antes de guardar de nuevo/]];
 for(const [st,code,re] of cases){
  const h=boot({fetch:c=>Promise.resolve(c.path==='/api/map/save'?R(st,{error:code,detail:'internal secret'}):R(200,{maps:[]}))});
  connect(h,'mapping');typeName(h,'z');h.E('btn-save-map').fire('click');await tick();await tick();
  const m=h.E('map-save-msg');assert(re.test(m.textContent),st+code+' -> '+m.textContent);assert(!/secret|undefined/.test(m.textContent));
  assert.equal(m.attrs.role,'alert');assert(!h.E('btn-save-map').disabled,'unlocked after '+code);
 }
 { // 401 -> login
  const h=boot({fetch:c=>Promise.resolve(c.path==='/api/map/save'?R(401,{error:'unauthorized'}):R(200,{maps:[]}))});
  connect(h,'mapping');typeName(h,'z');h.E('btn-save-map').fire('click');await tick();await tick();
  assert(/token/.test(h.E('map-save-msg').textContent));assert(!h.E('login').hidden,'login shown');assert(!('mtrb_token' in h.store));
 }
 { // network error
  const h=boot({fetch:c=>c.path==='/api/map/save'?Promise.reject(new Error('net')):Promise.resolve(R(200,{maps:[]}))});
  connect(h,'mapping');typeName(h,'z');h.E('btn-save-map').fire('click');await tick();await tick();
  assert(/No se pudo contactar/.test(h.E('map-save-msg').textContent));assert(!h.E('btn-save-map').disabled);
 }
 { // 20 s without answer
  const h=boot({fetch:c=>c.path==='/api/map/save'?new Promise(()=>{}):Promise.resolve(R(200,{maps:[]}))});
  connect(h,'mapping');typeName(h,'z');h.E('btn-save-map').fire('click');
  h.advance(39000);assert.equal(h.E('btn-save-map').textContent,'Guardando…');
  h.advance(1500);await tick();await tick();
  assert.equal(h.E('map-save-msg').textContent,'Sin respuesta del robot; comprueba en «Mapas guardados» si se guardó.');assert(!h.E('btn-save-map').disabled);
  assert.equal(getMaps(h).length,1,'list refreshed after timeout');
 }
 // ---- maps list: hostile names as text ----
 {const hostile=['<script>alert(1)</script>','a"b\'c','<img src=x onerror=alert(1)>','ok_map'];
  const h=boot({fetch:c=>Promise.resolve(R(200,{maps:hostile.concat([5,null])}))});
  connect(h,'mapping');
  h.E('map-box').open=true;h.E('map-box').fire('toggle');
  assert.equal(h.E('map-list-state').textContent,'Cargando mapas guardados…');
  await tick();await tick();
  const g=getMaps(h);assert.equal(g.length,1);assert.equal(g[0].init.method,'GET');assert.equal(g[0].init.headers.Authorization,'Bearer tok');
  const items=h.E('map-list').children;assert.equal(items.length,4,'non-strings dropped');
  items.forEach((li,i)=>{assert.equal(li.textContent,hostile[i]);assert.equal(li.innerHTML,undefined,'innerHTML never set');assert.equal(li.children.length,0)});
  assert(h.E('map-list-state').hidden);
  // no buttons/actions inside list
  assert(items.every(li=>!li.children.length));
  // refresh replaces the list (no duplicates)
  h.E('btn-refresh-maps').fire('click');await tick();await tick();assert.equal(h.E('map-list').children.length,4);
  // empty and error
  const h2=boot({fetch:()=>Promise.resolve(R(200,{maps:[]}))});connect(h2,'mapping');h2.E('map-box').open=true;h2.E('map-box').fire('toggle');await tick();await tick();
  assert.equal(h2.E('map-list-state').textContent,'No hay mapas guardados.');
  const h3=boot({fetch:()=>Promise.resolve(R(500,{}))});connect(h3,'mapping');h3.E('map-box').open=true;h3.E('map-box').fire('toggle');await tick();await tick();
  assert.equal(h3.E('map-list-state').textContent,'No se pudo cargar la lista de mapas.');
  const h4=boot({fetch:()=>Promise.reject(new Error('n'))});connect(h4,'mapping');h4.E('map-box').open=true;h4.E('map-box').fire('toggle');await tick();await tick();
  assert.equal(h4.E('map-list-state').textContent,'No se pudo cargar la lista de mapas.');
  // closing the panel does not fetch
  const h5=boot();connect(h5,'mapping');h5.E('map-box').open=false;h5.E('map-box').fire('toggle');assert.equal(getMaps(h5).length,0);
 }
 // ---- status panel ----
 {const h=boot();connect(h);
  h.msg(status('mapping',{nodes:{slam_toolbox:'active',amcl:'unavailable','<b>x</b>':'weird',bt_navigator:'unconfigured',lc:'finalized',m:'inactive',u:'unknown'},scan_age:3.4,odom_age:0.5,battery_pct:55.2}));
  assert.equal(h.E('st-mode').textContent,'Mapeo (mapping)');assert.equal(h.E('st-battery').textContent,'55 %');
  const li=h.E('st-nodes').children.map(x=>x.textContent);
  assert(li.includes('✔ slam_toolbox: Activo'),li.join('|'));assert(li.includes('⚠ amcl: No disponible'));
  assert(li.includes('⚠ <b>x</b>: Desconocido'),'hostile node name and unknown state as text');
  assert(li.includes('⚠ bt_navigator: Sin configurar')&&li.includes('⚠ lc: Finalizado')&&li.includes('⚠ m: Inactivo'));
  const cls=h.E('st-nodes').children.map(x=>x.className);assert(cls.includes('node-ok')&&cls.includes('node-bad')&&cls.includes('node-warn'));
  assert(/3,4 s/.test(h.E('st-scan-age').textContent)&&/⚠/.test(h.E('st-scan-age').textContent),h.E('st-scan-age').textContent);
  assert.equal(h.E('st-scan-age').className,'warn');assert.equal(h.E('st-odom-age').textContent,'0,5 s');assert.equal(h.E('st-odom-age').className,'');
  h.msg(status('mapping',{scan_age:2,odom_age:2.01}));assert.equal(h.E('st-scan-age').className,'','2.0 is not > 2');assert.equal(h.E('st-odom-age').className,'warn');
  h.msg(status('mapping',{scan_age:null,odom_age:null,battery_pct:null,nodes:{}}));
  assert(/sin datos/.test(h.E('st-scan-age').textContent));assert.equal(h.E('st-battery').textContent,'No disponible');
  assert.equal(h.E('st-nodes').children[0].textContent,'Sin datos de nodos');
  h.msg({type:'status',mode:'map'});assert.equal(h.E('st-nodes').children.length,1);assert(/sin datos/.test(h.E('st-odom-age').textContent));
  h.msg({type:'pose',x:1,y:2,yaw:0});assert(/x 1\.00 m, y 2\.00 m/.test(h.E('st-pose').textContent));
  // connection indicator text + stale note
  assert.equal(h.E('conn-text').textContent,'Conectado');assert.equal(h.E('conn').className,'badge badge-green');assert.equal(h.E('st-conn').textContent,'Conectado');
  h.advance(2500);assert.equal(h.E('conn').className,'badge badge-amber');assert.equal(h.E('conn-text').textContent,'Datos con retraso');
  h.advance(3000);assert.equal(h.E('conn').className,'badge badge-red');
  h.ws().readyState=3;h.ws().onclose({code:1006});assert.equal(h.E('conn-text').textContent,'Sin conexión');assert(!h.E('st-stale').hidden);
  assert(/último estado/.test(h.E('st-stale').textContent));
 }
 // ---- layers ----
 {const h=boot({lsBlocked:true}); // localStorage getter throws: must not break anything
  connect(h,'mapping');
  h.E('layer-scan').checked=false;h.E('layer-scan').fire('change');h.E('layer-trail').checked=true;h.E('layer-trail').fire('change');
  assert.equal(h.E('layer-scan').checked,false);
  h.msg({type:'pose',x:0,y:0,yaw:0});h.msg({type:'pose',x:1,y:0,yaw:0});h.msg({type:'pose',x:1.01,y:0,yaw:0});
  assert(h.ctx.MTRBPure,'app alive with blocked storage');
 }
 {const h=boot({store:{mtrb_layers:'{"scan":false,"plan":true,"dock":false,"trail":true}'}});
  assert.equal(h.E('layer-scan').checked,false);assert.equal(h.E('layer-plan').checked,true);assert.equal(h.E('layer-dock').checked,false);assert.equal(h.E('layer-trail').checked,true,'restored');
  h.E('layer-plan').checked=false;h.E('layer-plan').fire('change');
  assert.deepEqual(JSON.parse(h.store.mtrb_layers),{scan:false,plan:false,dock:false,trail:true},'persisted');
  const g=boot({store:{mtrb_layers:'{not json'}});assert.equal(g.E('layer-scan').checked,true,'corrupt value ignored');
  const g2=boot({store:{mtrb_layers:'{"scan":"yes","plan":1}'}});assert.equal(g2.E('layer-scan').checked,true,'non-boolean ignored');
  // setItem throws
  const k=boot();k.ctx.localStorage.setItem=()=>{throw new Error('quota')};k.E('layer-scan').checked=false;k.E('layer-scan').fire('change');
 }
 console.log('t18 OK');
})().catch(e=>{console.error(e);process.exit(1)});
