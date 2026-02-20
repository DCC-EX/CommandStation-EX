R"???(
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
)???"