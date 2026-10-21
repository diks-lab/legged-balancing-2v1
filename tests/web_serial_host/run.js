// Actual HTML bootstrap + actual UI + bounded history in a browser-like DOM.
const assert=require('node:assert/strict'),fs=require('node:fs'),vm=require('node:vm');
const html=fs.readFileSync('data/index.html','utf8'),app=fs.readFileSync('data/app.js','utf8');
assert(!/<(?:link|script)[^>]+(?:href|src)="\//i.test(html));assert(html.includes('href="data:,"'));
for(const file of ['index.html','style.css','app.js']){const bytes=fs.readFileSync('data/'+file);assert(bytes.length+Array.from(bytes).filter(n=>n===10).length<=8192);}
const {DiagnosticHistory,SIGNALS,DEFAULTS}=require('../../data/app.js');
const h=new DiagnosticHistory();
for(let i=0;i<4000;i++)h.push({sample_ms:i*50,seq:i,values:{angle_error_deg:i},stop_reason:'"stop"'});
assert(h.csv().split('\n')[0].includes('angle_error_deg'));assert(!h.csv().split('\n')[0].includes('right_velocity_rad_s'));assert.equal(h.rows.length,1200);assert(h.clock-h.rows[0].time<=60000);assert(h.csv().includes('""stop""'));
h.push({sample_ms:300000,values:{}});assert.equal(h.rows.length,1);h.clear();assert.equal(h.rows.length,0);
h.push({sample_ms:0xfffffff0,values:{}});h.push({sample_ms:34,values:{}});assert.equal(h.rows[1].time-h.rows[0].time,50);
class Element{
 constructor(tag){this.tag=tag;this.children=[];this.textContent='';this.style={};}
 append(...nodes){this.children.push(...nodes);}appendChild(n){this.append(n);}replaceChildren(){this.children=[];}
 setAttribute(){}click(){}querySelectorAll(){return this.children.flatMap(n=>n.children).filter(n=>n.tag==='input'&&n.checked);}
 getContext(){return {clearRect(){},fillText(){},beginPath(){},lineTo(){},moveTo(){},stroke(){}};}
}
const bootstrap=html.match(/<script>([\s\S]*?)<\/script>/)[1];
const settle=async()=>{for(let i=0;i<30;i++)await Promise.resolve();};
async function run(fail='',bodyFailure=false,invalidJs=false){
 let ready,requests=[],timers=[],active=0,maxActive=0,sockets=[],downloads=[],reports=[];
 const elements=new Proxy({}, {get:(o,id)=>o[id]??(o[id]=new Element(id))});let context;
 const document={getElementById:id=>elements[id],createTextNode:text=>({textContent:text}),
  addEventListener:(event,fn,opts)=>{assert.equal(event,'DOMContentLoaded');assert(opts.once);ready=fn;},createElement:tag=>new Element(tag),
  head:{appendChild:node=>{assert.equal(active,0);if(node.tag==='script')vm.runInContext(node.textContent,context);}}};
 class WebSocket{
  constructor(url){assert.equal(active,0);assert.deepEqual(requests,['/style.css','/app.js']);assert(url.endsWith(':81/diagnostics'));this.readyState=0;this.sent=[];sockets.push(this);}
  send(text){this.sent.push(JSON.parse(text));}close(){this.readyState=3;this.onclose?.({code:1000});}
 }
 context=vm.createContext({console:{info:(...args)=>reports.push(args)},document,window:{},location:{hostname:'esp'},AbortSignal,Date,Error,WebSocket,
  Blob:class{constructor(parts){downloads.push(parts.join(''));}},URL:{createObjectURL:()=> 'blob:test',revokeObjectURL(){}},setTimeout:fn=>timers.push(fn),setInterval:fn=>timers.push(fn),fetch:async path=>{
   requests.push(path);++active;maxActive=Math.max(active,maxActive);
   if(path===fail&&!bodyFailure){--active;return {ok:false,status:503};}
   return {ok:true,text:async()=>{try{if(path===fail)throw new Error('truncated body');return path==='/app.js'?(invalidJs?'syntax error !':app):'body{}';}finally{--active;}}};
  }});
 vm.runInContext(bootstrap,context);assert.deepEqual(requests,[]);await ready();await settle();
 return {context,elements,requests,timers,sockets,maxActive,downloads,reports};
}
(async()=>{
 let r=await run();assert.deepEqual(r.requests,['/style.css','/app.js']);assert.equal(r.maxActive,1);assert.equal(r.sockets.length,1);
 let ws=r.sockets[0];ws.readyState=1;ws.onopen();assert.deepEqual(ws.sent[0].fields,DEFAULTS);assert.equal(ws.sent[0].period_ms,50);
 const ack={type:'subscribed',request_id:1,subscription_id:1,fields:DEFAULTS,streaming:true,period_ms:50};
 ws.onmessage({data:JSON.stringify(ack)});assert(r.elements.subscription.textContent.includes('世代 1'));
 const sample=(seq,extra={})=>({type:'sample',boot_id:'one',subscription_id:1,seq,sample_ms:seq*50,foc_capture_us:1,age_ms:2,dropped:0,stop_reason:'TILT_STOP',stop_ms:1,faults:1,values:{angle_error_deg:1},...extra});
 ws.onmessage({data:JSON.stringify(sample(1))});ws.onmessage({data:JSON.stringify(sample(4,{dropped:2}))});
 assert(r.elements.loss.textContent.includes('欠落推定: 2'));assert(r.elements.health.textContent.includes('TILT_STOP'));assert.equal(r.reports.length,1);assert.equal(r.reports[0][0],'WS_FIRST_SAMPLE');
 r.elements.pause.onclick();assert.equal(r.elements.pause.textContent,'表示再開');ws.onmessage({data:JSON.stringify(sample(5))});assert(r.elements.loss.textContent.includes('seq: 5'));r.elements.save.onclick();assert.equal(r.downloads[0].split('\n').length,4);assert(r.downloads[0].includes('faults'));
 r.elements.clear.onclick();r.elements.pause.onclick();r.elements.stop.onclick();assert.equal(ws.sent[1].type,'stop');
 const graphCount=r.elements.graphs.children.length;ws.onmessage({data:JSON.stringify({...ack,request_id:2,subscription_id:2,streaming:false,fields:[]})});assert.equal(r.elements.graphs.children.length,graphCount);
 r.elements.start.onclick();assert.equal(ws.sent[2].type,'subscribe');
 ws.onmessage({data:JSON.stringify({...ack,request_id:2,subscription_id:2})});assert(r.elements.subscription.textContent.includes('配信応答待ち'));
 ws.onmessage({data:JSON.stringify({...ack,request_id:3,subscription_id:3})});assert(r.elements.subscription.textContent.includes('世代 3'));
 ws.onmessage({data:JSON.stringify(sample(1,{boot_id:'two',subscription_id:3}))});assert(r.elements.loss.textContent.includes('欠落推定: 0'));
 if(process.env.WS_SAMPLE_FIXTURE){const native=JSON.parse(fs.readFileSync(process.env.WS_SAMPLE_FIXTURE))[0];ws.onmessage({data:JSON.stringify({...ack,request_id:3,subscription_id:native.subscription_id,fields:Object.keys(native.values)})});ws.onmessage({data:JSON.stringify(native)});assert(r.elements.loss.textContent.includes(`seq: ${native.seq}`));assert(r.reports.some(a=>a[0]==='WS_FIRST_SAMPLE'&&a[2]===native.seq));}
 ws.close();assert(r.elements.connection.textContent.includes('手動'));r.elements.reconnect.onclick();assert.equal(r.sockets.length,2);
 for(const target of ['/style.css','/app.js'])for(const body of [false,true]){r=await run(target,body);assert(r.elements.connection.textContent.includes(target)&&r.elements.connection.textContent.includes('失敗'));assert.equal(r.requests.length,target==='/style.css'?1:2);assert.equal(r.sockets.length,0);}
 r=await run('',false,true);assert.equal(r.sockets.length,0);assert(r.elements.connection.textContent.includes('/app.js'));
 assert.equal(SIGNALS.length,26);
 assert(html.includes('id="start">配信開始</button>'));assert(!html.includes('購読適用')&&!html.includes('購読開始'));
 console.log('PASS: real serial bootstrap -> WebSocket; selected JSON, generation/boot/loss/status, pause/clear/reconnect; 60s/1200 history/wrap/CSV; failure chain, LF/CRLF size bounds');
})().catch(e=>{console.error(e);process.exitCode=1;});
