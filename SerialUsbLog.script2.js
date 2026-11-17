const oledScreens = new Map();

function syncOLEDWidths() {
  // Find the maximum natural width among all screens
  let maxWidth = 0;
  for (const screen of oledScreens.values()) {
    // Temporarily remove width constraint to measure natural content width
    const originalWidth = screen.frame.style.width;
    const originalMaxWidth = screen.frame.style.maxWidth;
    screen.frame.style.width = 'auto';
    screen.frame.style.maxWidth = 'none';
    
    // Measure the natural width needed
    const naturalWidth = screen.frame.offsetWidth;
    if (naturalWidth > maxWidth) {
      maxWidth = naturalWidth;
    }
    
    // Restore original constraints
    screen.frame.style.width = originalWidth;
    screen.frame.style.maxWidth = originalMaxWidth;
  }
  
  // Apply the maximum width to all screens
  if (maxWidth > 0) {
    for (const screen of oledScreens.values()) {
      screen.frame.style.width = maxWidth + 'px';
      screen.frame.style.maxWidth = 'none';
    }
  }
}

function OLED(screenNum, row, text) {
  // Create screen container if it doesn't exist
  if (!oledScreens.has(screenNum)) {
    const frame = document.createElement('div');
    frame.id = `oled-screen-${screenNum}`;
    frame.className = 'oled-screen-frame';
    
    // Create screen number label (positioned absolutely within frame)
    const screenLabel = document.createElement('div');
    screenLabel.className = 'oled-screen-label';
    screenLabel.textContent = screenNum;
    frame.appendChild(screenLabel);
    
    // Create a content container for the text (separate from label)
    const contentDiv = document.createElement('div');
    contentDiv.id = `oled-screen-${screenNum}-content`;
    contentDiv.className = 'oled-screen-content';
    frame.appendChild(contentDiv);
    
    // Create a wrapper to position both frame and label
    const wrapper = document.createElement('div');
    wrapper.className = 'oled-screen-wrapper';
    wrapper.id = `oled-screen-${screenNum}-wrapper`;
    wrapper.appendChild(frame);
    
    // Position relative to the log textarea
    const logEl = document.getElementById('log');
    if (logEl) {
      logEl.parentNode.style.position = 'relative';
      logEl.parentNode.appendChild(wrapper);
      
      // Position screens over the right side of textarea, stacked vertically
      const updatePosition = () => {
        const rect = logEl.getBoundingClientRect();
        const parentRect = logEl.parentNode.getBoundingClientRect();
        
        // Calculate vertical offset based on screen number and previously positioned screens
        let topOffset = rect.top - parentRect.top;
        let screenIndex = 0;
        for (const [num, screen] of oledScreens) {
          if (num < screenNum) {
            topOffset += screen.wrapper.offsetHeight + 10; // Add height of previous screen + gap
          }
        }
        
        wrapper.style.top = topOffset + 'px';
        wrapper.style.right = '30px';
        wrapper.style.width = 'auto';
        wrapper.style.height = 'auto';
        frame.style.maxHeight = (rect.height - topOffset + (rect.top - parentRect.top)) + 'px';
        frame.style.overflowY = 'auto';
      };
      updatePosition();
      window.addEventListener('resize', updatePosition);
      window.addEventListener('scroll', updatePosition);
    }
    
    oledScreens.set(screenNum, {
      frame: frame,
      contentDiv: contentDiv,
      wrapper: wrapper,
      rows: new Map()
    });
  }
  
  const screen = oledScreens.get(screenNum);
  const rows = screen.rows;
  
  // Store or update the row
  if (text && text.trim()) {
    rows.set(row, text);
  } else {
    rows.delete(row);
  }
  
  // Render only non-empty rows, sorted by row number
  const sortedRows = Array.from(rows.entries()).sort((a, b) => a[0] - b[0]);
  screen.contentDiv.textContent = sortedRows.map(([, text]) => text).join('\n');
  
  // Hide frame if no rows
  screen.frame.style.display = sortedRows.length > 0 ? 'block' : 'none';
  
  // Sync all screen widths to match the widest one
  syncOLEDWidths();
  screen.frame.style.display = sortedRows.length > 0 ? 'block' : 'none';
}

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
      
      // Scan for <@ lines and call OLED function
      const lines=t.split('\n');
      for(const line of lines){
        if(line.includes('<@')){
          const match=line.match(/\[\d+\]<@\s+(\d+)\s+(\d+)\s+"(.*)"/);
          if(match){
            OLED(parseInt(match[1]), parseInt(match[2]), match[3]);
          }
        }
      }
      
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
const hostName = window.location.hostname || window.location.host;
if (hostName && hostName!="DCC-EX") {
  showHostName.textContent = "(" + hostName + ")";
  document.title = "DCC-EX " + showHostName.textContent;
}
tick();
