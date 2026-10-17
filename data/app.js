async function refresh(){
  try{
    const response=await fetch('/api/status',{cache:'no-store',signal:AbortSignal.timeout(3000)});
    if(!response.ok)throw new Error(await response.text());
    document.getElementById('status').textContent=await response.text();
    document.getElementById('connection').textContent=`接続: ${location.host} / 更新 ${new Date().toLocaleTimeString()}`;
  }catch(error){document.getElementById('connection').textContent=`状態取得失敗（表示は前回の値）: ${error.message}`;}
  setTimeout(refresh,1000);
}
for(const button of document.querySelectorAll('[data-command]')){
  button.addEventListener('click',async()=>{
    button.disabled=true;
    try{
      const response=await fetch(`/control/${button.dataset.command}`,{method:'POST',signal:AbortSignal.timeout(3000)});
      document.getElementById('result').textContent=await response.text();
    }catch(error){document.getElementById('result').textContent=`結果不明: ${error.message}。状態を確認してください。`;}
    finally{button.disabled=false;}
  });
}
refresh();
