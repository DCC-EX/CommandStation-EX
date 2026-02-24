R"???(
async function tick(){
try{
if(!paused && logEl.selectionStart===logEl.selectionEnd){
  const chunk=2048;
  let cmd=cmdWaiting ? cmdInput.value.trim() : '';
  cmdWaiting=false;
  if (cmd.length>0 && cmd[0]!='<') cmd='<' + cmd + '>';
  const cmdpart = cmd.length>0 ? ("&cmd="+encodeURIComponent(cmd)) : "";
  const uri='/log?from='+seq+'&chunk='+chunk+cmdpart;
  console.log("Fetching URI:", uri);
  const r=await fetch(uri,{cache:'no-store'});
  if(r.ok){
    const t=await r.text();
    const nxt=r.headers.get('X-Next-Seq');
    if(nxt) seq=parseInt(nxt,10)||seq;
    if(t && t.length){
      buf+=t;
      logEl.textContent=applyFilter(buf);
      if(followCb.checked&&!userScrolled) logEl.scrollTop=logEl.scrollHeight;
    }
    stat.textContent=(paused?'paused':'live')+' • seq '+seq+' • '+buf.length+' chars';
  } else {
    stat.textContent='http '+r.status;
  }
} else {
  stat.textContent='paused • seq '+seq+' • '+buf.length+' chars';
}
}catch(e){
stat.textContent='offline';
}
setTimeout(tick, 400);
}
tick();

)???"