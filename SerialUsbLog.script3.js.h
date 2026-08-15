/* this is the first attempt at managing a popup menu to list available 
configuration dialogs. It was written mostly by AI. 
*/

String SerialUsbLog_script3_js=R"???(
NVSTable = [];
function convertNvsInputs(container) {
  const nvsNodes = container.querySelectorAll('nvsinput');
  nvsNodes.forEach(function(el) {
    const nvs = parseInt(el.getAttribute('nvs'), 10);
    const hasMin = el.hasAttribute('min');
    const hasMax = el.hasAttribute('max');
    const min = parseInt(el.getAttribute('min'), 10);
    const max = parseInt(el.getAttribute('max'), 10);
    const length = parseInt(el.getAttribute('length'), 10);
    const useTextInput = !hasMin && !hasMax && Number.isFinite(length) && length > 0;
    const useCheckbox = min === 0 && max === 1;

    const input = document.createElement('input');
    if (useCheckbox) {
      input.type = 'checkbox';
      input.checked = Number(NVSTable[nvs]) === 1;

      input.addEventListener('change', function() {
        NVSTable[nvs] = this.checked ? 1 : 0;
        if (typeof window.setConfigPanelDirty === 'function') {
          window.setConfigPanelDirty(true);
        }
      });
    } else if (useTextInput) {
      input.type = 'text';
      input.maxLength = length;
      input.value = NVSTable[nvs] ?? '';

      input.addEventListener('input', function() {
        NVSTable[nvs] = this.value;
        if (typeof window.setConfigPanelDirty === 'function') {
          window.setConfigPanelDirty(true);
        }
      });
    } else {
      input.type = 'number';
      input.min = Number.isFinite(min) ? min : 0;
      input.max = Number.isFinite(max) ? max : 255;
      input.value = NVSTable[nvs] ?? 0;

      input.addEventListener('input', function() {
        const value = parseInt(this.value, 10);
        if (!Number.isNaN(value)) {
          NVSTable[nvs] = value;
          if (typeof window.setConfigPanelDirty === 'function') {
            window.setConfigPanelDirty(true);
          }
        }
      });
    }

    el.parentNode.replaceChild(input, el);
  });
};

function normalizeNvsInputs(html) {
  return html.replace(/<nvsinput\b([^>]*)\/>/gi, '<nvsinput$1></nvsinput>');
}

async function refreshNvsValues() {
  const response = await fetch('/nvsValues.js', { cache: 'no-store' });
  if (!response.ok) throw new Error('HTTP ' + response.status);

  const source = await response.text();
  const values = new Function(source + '\nreturn NVSTable;')();
  NVSTable.length = values.length;
  values.forEach(function(value, index) {
    NVSTable[index] = value;
  });
  NVSTableBefore = values.slice();
}

(function() {
  const configsBtn = document.getElementById('configsBtn');
  const configsMenu = document.getElementById('configsMenu');
  const configPanel = document.getElementById('configPanel');
  const configPanelTitle = document.getElementById('configPanelTitle');
  const configPanelBody = document.getElementById('configPanelBody');
  const configPanelClose = document.getElementById('configPanelClose');
  const configPanelSave = document.getElementById('configPanelSave');
  let configPanelDirty = false;

  if (!configsBtn || !configsMenu) return;

  window.setConfigPanelDirty = function(dirty) {
    configPanelDirty = !!dirty;
    if (!configPanelSave) return;
    configPanelSave.style.visibility = configPanelDirty ? 'visible' : 'hidden';
    configPanelSave.disabled = !configPanelDirty;
  };

  function closeConfigPanel() {
    if (configPanelDirty && configPanelSave &&
        configPanelSave.style.visibility === 'visible' &&
        !window.confirm('Discard unsaved changes?')) {
      return;
    }
    if (configPanelDirty && Array.isArray(window.NVSTableBefore)) {
      NVSTable.length = NVSTableBefore.length;
      NVSTableBefore.forEach(function(value, index) {
        NVSTable[index] = value;
      });
    }
    if (configPanel) {
      configPanel.hidden = true;
    }
    if (configPanelBody) {
      configPanelBody.innerHTML = '';
    }
    window.setConfigPanelDirty(false);
  }
function saveConfigPanel() {
  const saveHandler =
    typeof window.dialogSave === 'function'
      ? window.dialogSave
      : (typeof window.saveNvsValues === 'function' ? window.saveNvsValues : null);

  if (saveHandler) {
    saveHandler();
  }
  }

  if (configPanel) {
    configPanel.hidden = true;
  }
  if (configPanelSave) {
    configPanelSave.style.visibility = 'hidden';
    configPanelSave.disabled = true;
  }
  configPanelDirty = false;

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

    const normalizedHtml = normalizeNvsInputs(html);
    configPanelBody.innerHTML = normalizedHtml;

    activateScripts(configPanelBody);
    convertNvsInputs(configPanelBody);

    configPanel.hidden = false;
    window.setConfigPanelDirty(false);
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
    configsMenu.insertBefore(item, configsMenu.firstChild);
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

    refreshNvsValues()
      .then(function() {
        return fetch(path, {
          headers: { 'X-Requested-With': 'XMLHttpRequest' }
        });
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

function saveNvsValues() {
  // create a string of nvs id=value from NVSTable up to the last one thats different to NVSTableBefore
  let result = '';
  for (let i = 0; i < NVSTable.length; i++) {
    if (NVSTable[i] !== NVSTableBefore[i]) {
    result += (i+'=')
     if (NVSTable[i] === null || NVSTable[i] === undefined) {
        result += '0,';
      } else if (typeof NVSTable[i] === 'string') {
        result += '"' + NVSTable[i].replace(/"/g, '\\"') + '",';
      } else {
        result += NVSTable[i] + ',';
      }
    }
  }
  
  console.log('Saving NVS values:', result.slice(0, -1)); // remove trailing comma
  // POST the result to /savenvs
  fetch('/savenvs', {
    method: 'POST',
    headers: {
      'Content-Type': 'application/txt'
    },
    body: result.slice(0, -1) + ';' 
  })
    .then(response => {
      if (!response.ok) throw new Error('HTTP ' + response.status);
      return response.text();
    })
    .then(data => {
      console.log('NVS values saved successfully:', data);
      // Update NVSTableBefore to match NVSTable after successful save
      NVSTableBefore = NVSTable.slice();
      if (typeof window.setConfigPanelDirty === 'function') {
        window.setConfigPanelDirty(false);
      }
    })
    .catch(err => {
      console.error('Failed to save NVS values:', err);
    });
}
)???";
