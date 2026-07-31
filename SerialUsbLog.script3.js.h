/* this is the first attempt at managing a popup menu to list available 
configuration dialogs. It was written mostly by AI. 
*/

String SerialUsbLog_script3_js=R"???(
(function() {
  const configsBtn = document.getElementById('configsBtn');
  const configsMenu = document.getElementById('configsMenu');

  if (!configsBtn || !configsMenu) return;

  function setMenuVisible(visible) {
    configsMenu.hidden = !visible;
    configsBtn.setAttribute('aria-expanded', visible ? 'true' : 'false');
  }

  configsBtn.addEventListener('click', function(e) {
    e.stopPropagation();
    setMenuVisible(configsMenu.hidden);
  });

  document.addEventListener('click', function() {
    setMenuVisible(false);
  });

  configsMenu.addEventListener('click', function(e) {
    e.stopPropagation();
  });

  document.addEventListener('keydown', function(e) {
    if (e.key === 'Escape') setMenuVisible(false);
  });

  window.addConfigMenuItem = function(label, handler, opts) {
    const item = document.createElement('button');
    item.type = 'button';
    item.className = 'menu-item';
    item.textContent = label;

    item.addEventListener('click', function(e) {
      e.stopPropagation();
      setMenuVisible(false);
      if (typeof handler === 'function') handler();
    });

    if (opts && opts.id) item.id = opts.id;
    configsMenu.appendChild(item);
    return item;
  };

  window.clearConfigMenu = function() {
    configsMenu.innerHTML = '';
  };

  window.setConfigMenuVisible = function(visible) {
    setMenuVisible(visible);
  };
})();

// this function called when SerialUsbLog.cpp enumerates the 
// list of config dialogs on path /configs.js
function addConfig(label,path) {
  addConfigMenuItem(label,function() { window.open(path); });
}

)???";
