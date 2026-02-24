R"???(
function dog(tag) {return document.getElementById(tag);}
const logEl=dog('log');
const pauseBtn=dog('pause');
const clearBtn=dog('clear');
const wrapCb=dog('wrap');
const followCb=dog('follow');
const filterIn=dog('filter');
const stat=dog('stat');
const cmdInput = dog('cmd');
const cmdButton = dog('cmdButton');
let paused=false, seq=0, buf='',cmdWaiting=false;
let userScrolled=false;
function atBottom(){return (logEl.scrollHeight-logEl.scrollTop-logEl.clientHeight)<8;}
logEl.addEventListener('scroll',()=>{userScrolled=!atBottom();});
wrapCb.addEventListener('change',()=>{logEl.style.whiteSpace=wrapCb.checked?'pre-wrap':'pre';});
pauseBtn.onclick=()=>{paused=!paused; pauseBtn.textContent=paused?'Resume':'Pause';};
clearBtn.onclick=()=>{buf=''; logEl.value=''; seq=0;};
cmdButton.onclick=()=>{cmdWaiting=true;};
function applyFilter(text){
const f=filterIn.value.trim();
if(!f) return text;
return text.split('\n').filter(l=>l.includes(f)).join('\n');
}
filterIn.addEventListener('input',()=>{logEl.value=applyFilter(buf); if(followCb.checked&&!userScrolled){logEl.scrollTop=logEl.scrollHeight;}});
cmdInput?.addEventListener('keydown', e=>{ if(e.key==='Enter'){ e.preventDefault(); cmdButton.click(); }});
)???"