R"???(
<!doctype html><html><head>
      <meta name=viewport content='width=device-width,initial-scale=1'>
      <title>DCC-EX Server Log</title>
      <style>
      html,body{height:100%;margin:0;font-family:system-ui,-apple-system,Segoe UI,Roboto,Arial}
      body{background:#0f1115;color:#d7dae0;display:flex;flex-direction:column}
      header{display:flex;flex-wrap:wrap;gap:.5rem;align-items:center;padding:.6rem .75rem;border-bottom:1px solid #2a2d34}
      header h1{font-size:1rem;margin:0;flex:1;min-width:220px;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
      button,input,label,a.btn{background:#181a1f;color:#d7dae0;border:1px solid #2a2d34;border-radius:10px;padding:.35rem .6rem;text-decoration:none}
      button:hover,input:hover,label:hover,a.btn:hover{border-color:#3a3f4a}
      #wrapLbl{display:flex;align-items:center;gap:.35rem;opacity:.9}
      #followLbl{display:flex;align-items:center;gap:.35rem;opacity:.9}
      #log{flex:1;overflow:auto;padding:.75rem;font:12px/1.35 ui-monospace,SFMono-Regular,Menlo,Consolas,monospace;white-space:pre;tab-size:2}
      #stat{opacity:.75;font-size:.85rem;min-width:160px;text-align:right}
      .dim{opacity:.75}
      </style></head><body>
      <header>
        <h1>DCC-EX Server Log</h1>
        <button id=pause>Pause</button>
        <button id=clear>Clear</button>
        <button id=copy>Copy</button>
        <a class=btn href='/dump'>Download</a>
        <label id=followLbl><input id=follow type=checkbox checked>Follow</label>
        <label id=wrapLbl><input id=wrap type=checkbox>Wrap</label>
        <input id=filter placeholder='Filter…' size=10>
        <span id=stat class=dim></span>
      </header>
      <div id=log></div>
      <script>
      const logEl=document.getElementById('log');
      const pauseBtn=document.getElementById('pause');
      const clearBtn=document.getElementById('clear');
      const copyBtn=document.getElementById('copy');
      const wrapCb=document.getElementById('wrap');
      const followCb=document.getElementById('follow');
      const filterIn=document.getElementById('filter');
      const stat=document.getElementById('stat');
      let paused=false, seq=0, buf='';
      let userScrolled=false;
      function atBottom(){return (logEl.scrollHeight-logEl.scrollTop-logEl.clientHeight)<8;}
      logEl.addEventListener('scroll',()=>{userScrolled=!atBottom();});
      wrapCb.addEventListener('change',()=>{logEl.style.whiteSpace=wrapCb.checked?'pre-wrap':'pre';});
      pauseBtn.onclick=()=>{paused=!paused; pauseBtn.textContent=paused?'Resume':'Pause';};
      clearBtn.onclick=()=>{buf=''; logEl.textContent=''; seq=0;};
      copyBtn.onclick=async()=>{try{await navigator.clipboard.writeText(buf);}catch(e){}};
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
      </script></body></html>
)???"