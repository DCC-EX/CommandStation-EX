String SerialUsbLog_html=R"???(
<!doctype html><html><head>
      <meta name=viewport content='width=device-width,initial-scale=1'>
      <title>DCC-EX Server Log</title>
      </head>
      <link rel=stylesheet href='/style.css'/>
      <body>
      <header>
        <h1>DCC-EX <span id=showHostName></span> Log</h1>

        <div class="menu-wrap">
          <button id=configsBtn type="button" aria-haspopup="true" aria-expanded="false">Configs</button>
          <div id=configsMenu class="menu" role="menu" hidden></div>
        </div>

        <button id=pause>Pause</button>
        <button id=clear>Clear</button>
        <label id=followLbl><input id=follow type=checkbox checked>Follow</label>
        <label id=wrapLbl><input id=wrap type=checkbox>Wrap</label>
        <input id=filter placeholder='Filter…' size=10>
        Command:
        <input id=cmd name="cmd" placeholder='Command input' size=40>
        <button id=cmdButton>Send</button>
        <span id=stat class=dim></span>
      </header>
      <textarea id=log readOnly></textarea>
      <div id="configPanel" hidden>
        <div class="config-panel-header">
          <div id="configPanelTitle">Config</div>
          <div id="configPanelRestartWarning" role="alert">Save Requires RESET</div>
          <button id="configPanelSave" type="button">Save</button>
          <button id="configPanelClose" type="button">Close</button>
        </div>
        <div id="configPanelBody"></div>
      </div>
      <script src="/script1.js" defer></script>
      <script src="/script2.js" defer></script>
      <script src="/script3.js" defer></script>
      <script src="/configs.js" defer></script>
</body></html>
)???";
