/* this is the first attempt at managing a popup menu to list available 
configuration dialogs. It was written mostly by AI. 
*/

String SerialUsbLog_script3_js=R"???(
(function() {
  const configsBtn = document.getElementById('configsBtn');
  const configsMenu = document.getElementById('configsMenu');
  const configPanel = document.getElementById('configPanel');

  if (!configsBtn || !configsMenu) return;

  if (configPanel) {
    configPanel.hidden = true;
  }

  function setMenuVisible(visible) {
    configsMenu.hidden = !visible;
    configsBtn.setAttribute('aria-expanded', visible ? 'true' : 'false');
  }

  function activateScripts(container) {
    const scripts = container.querySelectorAll('script');
    scripts.forEach(oldScript => {
      const newScript = document.createElement('script');
      if (oldScript.src) {
        newScript.src = oldScript.src;
      } else {
        newScript.textContent = oldScript.textContent;
      }
      oldScript.parentNode.replaceChild(newScript, oldScript);
    });
  }

  window.setConfigPanel = function(title, html) {
    if (!configPanel) return;

    configPanel.hidden = false;
    configPanel.innerHTML = '';

    const header = document.createElement('div');
    header.style.cssText =
      'display:flex; justify-content:space-between; align-items:center; ' +
      'padding:8px 10px; background:#0f0f0f; border-bottom:1px solid #ccc;';

    const titleEl = document.createElement('div');
    titleEl.textContent = title || 'Config';
    titleEl.style.fontWeight = 'bold';

    const closeBtn = document.createElement('button');
    closeBtn.type = 'button';
    closeBtn.textContent = 'Close';
    closeBtn.style.padding = '4px 8px';
    closeBtn.addEventListener('click', function() {
      configPanel.hidden = true;
      configPanel.innerHTML = '';
    });

    header.appendChild(titleEl);
    header.appendChild(closeBtn);

    const body = document.createElement('div');
    body.style.padding = '10px';
    body.innerHTML = html;
    activateScripts(body);

    configPanel.appendChild(header);
    configPanel.appendChild(body);
  };

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
    if (e.key === 'Escape') {
      setMenuVisible(false);
      if (configPanel) {
        configPanel.hidden = true;
        configPanel.innerHTML = '';
      }
    }
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

window.addConfig = function(label, path) {
  addConfigMenuItem(label, function() {
    window.setConfigPanel(label, '<div style="color:#666;">Loading…</div>');

    fetch(path, {
      headers: { 'X-Requested-With': 'XMLHttpRequest' }
    })
      .then(function(response) {
        if (!response.ok) throw new Error('HTTP ' + response.status);
        return response.text();
      })
      .then(function(html) {
        window.setConfigPanel(label, html);
      })
      .catch(function(err) {
        window.setConfigPanel(
          label,
          '<div style="color:#a00;">Failed to load config: ' + err.message + '</div>'
        );
        console.error(err);
      });
  });
};

function SaveCvValues() {
  // create a string of cv values from CVTable up to the last one thats different to CVTableBefore
  let result = '';
  let lastDifferentIndex = -1;
  for (let i = 0; i < CVTable.length; i++) {
    if (CVTable[i] !== CVTableBefore[i]) {
      lastDifferentIndex = i;
    }
  }
  for (let i = 0; i <= lastDifferentIndex; i++) {
    result += (CVTable[i] ?? 0) + ',';
  }
  console.log('Saving CV values:', result.slice(0, -1)); // remove trailing comma
  // POST the result to /savecv
  fetch('/savecv', {
    method: 'POST',
    headers: {
      'Content-Type': 'application/txt'
    },
    body: '{' + result.slice(0, -1) + '}' // remove trailing comma
  })
    .then(response => {
      if (!response.ok) throw new Error('HTTP ' + response.status);
      return response.text();
    })
    .then(data => {
      console.log('CV values saved successfully:', data);
      // Update CVTableBefore to match CVTable after successful save
      CVTableBefore = CVTable.slice();
    })
    .catch(err => {
      console.error('Failed to save CV values:', err);
    });
}
)???";
