String SerialUsbLog_style_css=R"???(
html,body{height:100%;margin:0;font-family:system-ui,-apple-system,Segoe UI,Roboto,Arial}
body{background:#0f1115;color:#d7dae0;display:flex;flex-direction:column}
header{display:flex;flex-wrap:wrap;gap:.5rem;align-items:center;padding:.6rem .75rem;border-bottom:1px solid #2a2d34}
header h1{font-size:1rem;margin:0;flex:1;min-width:220px;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
button,input,textarea,label,a.btn{background:#181a1f;color:#d7dae0;border:1px solid #2a2d34;border-radius:10px;padding:.35rem .6rem;text-decoration:none}
button:hover,input:hover,label:hover,a.btn:hover{border-color:#3a3f4a}
.menu-wrap{position:relative;display:inline-flex;align-items:center}
.menu{position:absolute;top:calc(100% + .35rem);left:0;z-index:1000;min-width:12rem;display:flex;flex-direction:column;gap:.25rem;padding:.35rem;background:#181a1f;border:1px solid #3a3f4a;border-radius:10px;box-shadow:0 10px 24px rgba(0,0,0,.45)}
.menu[hidden]{display:none!important}
.menu-item{width:100%;text-align:left;background:#181a1f;color:#d7dae0;border:1px solid transparent;border-radius:8px;padding:.35rem .55rem}
.menu-item:hover{border-color:#3a3f4a}
#wrapLbl{display:flex;align-items:center;gap:.35rem;opacity:.9}
#followLbl{display:flex;align-items:center;gap:.35rem;opacity:.9}
#log{flex:1;overflow:auto;padding:.75rem;font:12px/1.35 ui-monospace,SFMono-Regular,Menlo,Consolas,monospace;white-space:pre;tab-size:2}
#stat{opacity:.75;font-size:.85rem;min-width:160px;text-align:right}
.dim{opacity:.75}
.form-group {
  margin-bottom: 12px;
}

.form-group label {
  display: block;
  margin-bottom: 4px;
}

.config-panel-header {
  display: flex;
  justify-content: flex-start;
  gap: 8px;
  align-items: center;
  padding: 8px 10px;
  background: #0f0f0f;
  color: #fff;
  border-bottom: 1px solid #ccc;
}

#configPanelSave {
  margin-left: 0;
}

#configPanelRestartWarning {
  margin-left: auto;
  display: none;
  border: 1px solid #e44;
  color: #f88;
  padding: 3px 6px;
  font-size: .85rem;
  white-space: nowrap;
}

#configPanelBody {
  padding: 10px;
}

#configPanel {
  border: 2px solid #fff;
  background: #000;
  margin-top: 10px;
  box-sizing: border-box;
}
)???";