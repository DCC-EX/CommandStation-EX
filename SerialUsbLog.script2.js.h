R"???(
function applyFilter(text){
const f=filterIn.value.trim();
if(!f) return text;
return text.split('\\n').filter(l=>l.includes(f)).join('\\n');
}
filterIn.addEventListener('input',()=>{logEl.textContent=applyFilter(buf); if(followCb.checked&&!userScrolled){logEl.scrollTop=logEl.scrollHeight;}});
async function tick(){
try{
if(!paused){
  const chunk=2048;
  const r=await fetch('/log?from='+seq+'&chunk='+chunk,{cache:'no-store'});
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