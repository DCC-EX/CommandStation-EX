/* this is the first attempt at managing a popup menu to list available 
configuration dialogs. It was written mostly by AI. 
*/

String SerialUsbLog_script3_js=R"???(
function convertCvInputs(container) {
  const cvNodes = container.querySelectorAll('cvinput');
  cvNodes.forEach(function(el) {
    const cv = parseInt(el.getAttribute('cv'), 10);
    const min = parseInt(el.getAttribute('min'), 10);
    const max = parseInt(el.getAttribute('max'), 10);

    const input = document.createElement('input');
    input.type = 'number';
    input.min = Number.isFinite(min) ? min : 0;
    input.max = Number.isFinite(max) ? max : 255;

    input.value = CVTable[cv] ?? 0;

    input.addEventListener('input', function() {
      const value = parseInt(this.value, 10);
      if (!Number.isNaN(value)) {
        CVTable[cv] = value;
      }
    });

    el.parentNode.replaceChild(input, el);
  });
};

(function() {
  const configsBtn = document.getElementById('configsBtn');
  const configsMenu = document.getElementById('configsMenu');
  const configPanel = document.getElementById('configPanel');
  const configPanelTitle = document.getElementById('configPanelTitle');
  const configPanelBody = document.getElementById('configPanelBody');
  const configPanelClose = document.getElementById('configPanelClose');
  const configPanelSave = document.getElementById('configPanelSave');

  if (!configsBtn || !configsMenu) return;

  function closeConfigPanel() {
    if (configPanel) {
      configPanel.hidden = true;
    }
    if (configPanelBody) {
      configPanelBody.innerHTML = '';
    }
  }
function saveConfigPanel() {
  const saveHandler =
    typeof window.dialogSave === 'function'
      ? window.dialogSave
      : (typeof window.saveCvValues === 'function' ? window.saveCvValues : null);

  if (saveHandler) {
    saveHandler();
  }
  }

  if (configPanel) {
    configPanel.hidden = true;
  }

  if (configPanelClose) {
    configPanelClose.addEventListener('click', closeConfigPanel);
  }
  if (configPanelSave) {
    configPanelSave.addEventListener('click', saveConfigPanel);
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
    if (!configPanel || !configPanelTitle || !configPanelBody) return;

    configPanelTitle.textContent = title || 'Config';
    configPanelBody.innerHTML = html;
    activateScripts(configPanelBody);
    convertCvInputs(configPanelBody);
    configPanel.hidden = false;
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
      closeConfigPanel();
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

function saveCvValues() {
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
