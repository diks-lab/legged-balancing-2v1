// Execute the actual HTML bootstrap and app in a controlled browser-like DOM.
const assert=require('node:assert/strict'),fs=require('node:fs'),vm=require('node:vm');
const html=fs.readFileSync('data/index.html','utf8'),app=fs.readFileSync('data/app.js','utf8');
assert(!/<(?:link|script)[^>]+(?:href|src)="\//i.test(html));
assert(html.includes('href="data:,"')); // avoid an implicit favicon request
const bootstrap=html.match(/<script>([\s\S]*?)<\/script>/)[1];
const settle=async()=>{for(let i=0;i<30;i++)await Promise.resolve();};
async function run(fail='',bodyFailure=false,invalidJs=false){
 let ready,requests=[],timers=[],active=0,maxActive=0,initial=0;
 const elements={connection:{textContent:''},status:{textContent:''},result:{textContent:''}};
 let context;
 const document={getElementById:id=>elements[id],querySelectorAll:()=>[],
  addEventListener:(event,fn,opts)=>{assert.equal(event,'DOMContentLoaded');assert(opts.once);ready=fn;},
  createElement:tag=>({tag,textContent:''}),
  head:{appendChild:node=>{assert.equal(active,0);if(node.tag==='script')vm.runInContext(node.textContent,context);}}};
 context=vm.createContext({document,window:{},location:{host:'esp'},AbortSignal,Date,Error,
  setTimeout:fn=>timers.push(fn),fetch:async path=>{
   requests.push(path);++active;maxActive=Math.max(active,maxActive);
   if(path===fail&&!bodyFailure){--active;return {ok:false,status:503};}
   return {ok:true,text:async()=>{try{if(path===fail)throw new Error('truncated body');return path==='/app.js'?(invalidJs?'syntax error !':app):path==='/style.css'?'body{}':'status';}finally{--active;}}};
  }});
 vm.runInContext(bootstrap,context);
 assert.deepEqual(requests,[]); // no asset before HTML parse completion
 await ready();await settle();initial=requests.length;
 return {context,elements,requests,timers,maxActive,initial};
}
(async()=>{
 let r=await run();assert.deepEqual(r.requests,['/style.css','/app.js','/api/status']);assert.equal(r.maxActive,1);assert.equal(r.timers.length,1);
 for(const target of ['/style.css','/app.js'])for(const body of [false,true]){
  r=await run(target,body);assert(r.elements.connection.textContent.includes(target));assert(r.elements.connection.textContent.includes('失敗'));
  assert.equal(r.requests.length,target==='/style.css'?1:2);assert.equal(r.timers.length,0);
 }
 r=await run('',false,true);assert.equal(r.requests.length,2);assert.equal(r.timers.length,0);assert(r.elements.connection.textContent.includes('/app.js'));
 r=await run('/api/status');for(let i=0;i<2;i++){assert.equal(r.timers.length,1);await r.timers.shift()();await settle();}
 assert.equal(r.requests.filter(p=>p==='/api/status').length,3);assert.equal(r.timers.length,0);assert(r.elements.connection.textContent.includes('自動更新を停止'));
 // A successful refresh schedules only after full body consumption and resets failure streak.
 r=await run();await r.timers.shift()();await settle();assert.equal(r.maxActive,1);assert.equal(r.timers.length,1);
 console.log('PASS: actual HTML DOM-ready -> CSS body -> JS ready -> status; HTTP/truncated failure stops chain; status retries capped; no parallel fetch');
})().catch(e=>{console.error(e);process.exitCode=1;});
