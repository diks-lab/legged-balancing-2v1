'use strict';
const SIGNALS=[
['angle_deg','姿勢角','deg'],['reference_deg','基準角','deg'],['angle_error_deg','角度誤差','deg'],['gyro_deg_s','角速度','deg/s'],
['right_position_rad','右位置','rad'],['left_position_rad','左位置','rad'],['right_velocity_rad_s','右速度','rad/s'],['left_velocity_rad_s','左速度','rad/s'],['mean_velocity_rad_s','平均速度','rad/s'],
['angle_term_v','姿勢項','V'],['gyro_term_v','角速度項','V'],['position_term_v','位置項','V'],['speed_term_v','速度項（補正済み）','V'],['total_output_v','合計出力','V'],['right_command_v','右指令電圧','V'],['left_command_v','左指令電圧','V'],['right_foc_q_v','右FOC q電圧','V'],['left_foc_q_v','左FOC q電圧','V'],
['control_dt_ms','制御周期','ms'],['loop_us','ループ時間','us'],['right_foc_us','右FOC時間','us'],['left_foc_us','左FOC時間','us'],['imu_us','IMU時間','us'],['move_us','move時間','us'],['internal_free_b','内部空きRAM','B'],['internal_largest_b','内部最大連続領域','B']];
const DEFAULTS=['angle_error_deg','gyro_deg_s','right_velocity_rad_s','left_velocity_rad_s','right_command_v','left_command_v'];
class DiagnosticHistory{
constructor(){this.clear();}
clear(){this.rows=[];this.clock=0;this.raw=null;}
push(sample){
if(this.raw!==null){const delta=(sample.sample_ms-this.raw)>>>0;if(delta>0x7fffffff)return;this.clock+=delta;}else this.clock=sample.sample_ms;
this.raw=sample.sample_ms;this.rows.push({...sample,time:this.clock});
while(this.rows.length>1200||this.rows.length&&this.clock-this.rows[0].time>60000)this.rows.shift();
}
csv(){const keys=SIGNALS.map(s=>s[0]).filter(k=>this.rows.some(r=>k in r.values)),quote=v=>'"'+String(v??'').replaceAll('"','""')+'"';
const head=['boot_id','subscription_id','seq','source_seq','sample_ms','foc_capture_us','age_ms','dropped','stop_ms','stop_reason','faults','running','disabled','ps4','foc_ready',...keys];
return head.join(',')+'\n'+this.rows.map(s=>head.map(k=>quote(k in s?s[k]:s.values[k])).join(',')).join('\n');}
}
if(typeof module!=='undefined')module.exports={DiagnosticHistory,SIGNALS,DEFAULTS};
if(typeof window!=='undefined')window.startLogger=function(){
const el=id=>document.getElementById(id),history=new DiagnosticHistory();
let socket=null,request=0,pending=0,generation=null,boot=null,lastSeq=null,lastDrop=0,lost=0,paused=false,lastMessage=0;
let accepted=[],charts=[],lastDraw=0,first=null;
const selected=()=>Array.from(el('fields').querySelectorAll('input:checked'),n=>n.value);
for(const [key,label,unit] of SIGNALS){const labelNode=document.createElement('label'),input=document.createElement('input');input.type='checkbox';input.value=key;input.checked=DEFAULTS.includes(key);labelNode.append(input,document.createTextNode(`${label} [${unit}]`));el('fields').appendChild(labelNode);}
function graphs(){el('graphs').replaceChildren();charts=[];
for(const unit of new Set(SIGNALS.filter(s=>accepted.includes(s[0])).map(s=>s[2]))){const signals=SIGNALS.filter(s=>s[2]===unit&&accepted.includes(s[0])),box=document.createElement('section'),title=document.createElement('h2'),legend=document.createElement('p'),canvas=document.createElement('canvas');title.textContent=unit;canvas.width=1000;canvas.height=180;canvas.setAttribute('aria-label',`${unit} 時系列グラフ`);
signals.forEach((s,i)=>{const span=document.createElement('span');span.textContent=s[1]+' ';span.style.color=`hsl(${i*67%360} 70% 35%)`;legend.appendChild(span);});box.append(title,legend,canvas);el('graphs').appendChild(box);charts.push({canvas,signals,unit});}
}
function draw(){const end=history.clock,start=end-60000;
for(const {canvas,signals,unit} of charts){const c=canvas.getContext('2d');c.clearRect(0,0,1000,180);let lo=Infinity,hi=-Infinity;
for(const row of history.rows)for(const [key] of signals){const v=row.values[key];if(Number.isFinite(v)){lo=Math.min(lo,v);hi=Math.max(hi,v);}}
if(!Number.isFinite(lo)){lo=-1;hi=1;}if(lo===hi){lo-=1;hi+=1;}const pad=(hi-lo)*.08;lo-=pad;hi+=pad;
c.fillStyle='#334';c.font='12px sans-serif';c.fillText(`${hi.toPrecision(4)} ${unit}`,4,15);c.fillText(lo.toPrecision(4),4,150);c.fillText(`${(start/1000).toFixed(1)} s`,64,176);c.fillText(`${(end/1000).toFixed(1)} s`,900,176);
signals.forEach(([key],i)=>{c.strokeStyle=`hsl(${i*67%360} 70% 35%)`;c.beginPath();let pen=false,previous=null;
for(const row of history.rows){const v=row.values[key],x=65+(row.time-start)/60000*925,y=155-(v-lo)/(hi-lo)*130;if(!Number.isFinite(v)){pen=false;continue;}if(previous!==null&&row.time-previous>150)pen=false;if(pen)c.lineTo(x,y);else c.moveTo(x,y);pen=true;previous=row.time;}c.stroke();});}
}
function subscribe(stop=false){if(!socket||socket.readyState!==1){el('connection').textContent='未接続：再接続してください';return;}const fields=selected();if(!stop&&!fields.length){el('subscription').textContent='項目を1つ以上選択してください';return;}
pending=++request;socket.send(JSON.stringify(stop?{type:'stop',request_id:pending}:{type:'subscribe',request_id:pending,period_ms:50,fields}));el('subscription').textContent='配信応答待ち…';}
function connect(){if(socket&&socket.readyState<2)return;
el('connection').textContent='WebSocket接続中…';generation=first=null;
const ws=new WebSocket(`ws://${location.hostname}:81/diagnostics`);socket=ws;
ws.onopen=()=>{if(socket!==ws)return;lastMessage=Date.now();el('connection').textContent='接続：配信設定中';subscribe();};
ws.onmessage=event=>{if(socket!==ws)return;let s;try{s=JSON.parse(event.data);}catch{el('connection').textContent='不正なJSONを受信';ws.close();return;}lastMessage=Date.now();
if(s.type==='error'){el('subscription').textContent=`配信設定拒否: ${s.reason}`;return;}
if(s.type==='subscribed'){if(s.request_id!==pending)return;generation=s.subscription_id;if(s.streaming)accepted=s.fields;el('subscription').textContent=`世代 ${generation} / ${s.period_ms} ms / ${s.streaming?'配信中':'配信停止'} / ${accepted.join(', ')}`;if(s.streaming){graphs();draw();}return;}
if(s.type!=='sample'&&s.type!=='status')return;
if(boot!==s.boot_id){if(boot!==null)history.clear();boot=s.boot_id;lastSeq=null;lastDrop=0;lost=0;}
if(lastSeq!==null){const delta=(s.seq-lastSeq)>>>0;if(delta>0&&delta<0x80000000)lost+=Math.max(delta-1,(s.dropped-lastDrop)>>>0);}
if(lastSeq===null)lost=s.dropped;lastSeq=s.seq;lastDrop=s.dropped;
el('connection').textContent=`接続 / boot ${boot} / 取得 ${s.sample_ms} ms / age ${s.age_ms} ms`;
el('health').textContent=`最新停止: ${s.stop_reason} @ ${s.stop_ms} ms / 異常bit: ${s.faults} / ${s.running?'動作中':'停止中'} / 出力遮断:${s.disabled} / FOC:${s.foc_ready} / PS4:${s.ps4}`;
el('loss').textContent=`欠落推定: ${lost} / 機体の送信破棄累計: ${s.dropped} / seq: ${s.seq}`;
if(s.subscription_id!==generation||!Object.keys(s.values).length)return;
history.push(s);if(!paused&&Date.now()-lastDraw>=100){draw();lastDraw=Date.now();}if(first!==generation){first=generation;console.info('WS_FIRST_SAMPLE',generation,s.seq,s.sample_ms);}
};
ws.onerror=()=>{if(socket===ws)el('connection').textContent='WebSocket通信失敗（自動再接続なし）';};
ws.onclose=event=>{if(socket===ws)el('connection').textContent=`切断 code=${event.code}（再接続は手動）`;};
}
el('start').onclick=()=>subscribe();el('stop').onclick=()=>subscribe(true);el('reconnect').onclick=connect;
el('pause').onclick=()=>{paused=!paused;el('pause').textContent=paused?'表示再開':'表示一時停止';if(!paused)draw();};
el('clear').onclick=()=>{history.clear();draw();};
el('save').onclick=()=>{const url=URL.createObjectURL(new Blob([history.csv()],{type:'text/csv;charset=utf-8'})),a=document.createElement('a');a.href=url;a.download='esp32-browser.csv';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);};
setInterval(()=>{if(socket&&socket.readyState===1&&Date.now()-lastMessage>6000){el('connection').textContent='更新停止（6秒）。再接続してください';socket.close();}},1000);
connect();
};
