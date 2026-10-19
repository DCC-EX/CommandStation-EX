/*
 *  © 2026 Chris Harlow
 *
 *  This file is part of CommandStation-EX
 *
 *  This is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  It is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with CommandStation.  If not, see <https://www.gnu.org/licenses/>.
 */


// This file referes to roster values

const char Roster_html[]=R"???(

            <div class="selector-controls">
                Select a roster entry:
                <button class="btn-inline-danger" onclick="rosterNewLoco()">NEW</button>
                 or <select id="locoSelector" onchange="rosterSelectLoco(this.value)">
                    <option value="">-- No locomotives loaded yet --</option>
                </select>
                <button class="btn-inline-danger" onclick="rosterDeleteLoco()">DELETE</button>
                <button class="btn-inline-danger" onclick="rosterCopyLoco()">COPY</button>
            </div>
        </div>

        <div class="form-row">
           <label for="locoNumber">DCC Address:</label>
                <input type="number" id="locoNumber" placeholder="e.g. 3" value="3" 
                        min="0" max="10239" step="1">
                <label for="locoId">Name:</label>
                <input type="text" id="locoName" required placeholder="e.g. Thomas" value="Thomas">
                Check the box for momentary functions
            </div>
        </div>
    </div>

    <!-- Functions F0 to F32 -->
    <div class="section">
    
        <style>
            .functions-grid {
                display: grid;
                grid-template-columns: repeat(auto-fill, minmax(240px, 1fr));
                gap: 4px 16px;
            }
            .func-item {
                display: grid;
                grid-template-columns: 2.5em minmax(0, 1fr) auto;
                align-items: center;
                gap: 6px;
            }
            .func-label { text-align: right; font-weight: bold; }
            .func-item input[type="text"] { width: 100%; min-width: 0; box-sizing: border-box; }
            .func-moment { white-space: nowrap; }
        </style>
        <div class="functions-grid" id="functionsGrid"></div>
    </div>

    <script>
        var totalFunctions = 33;
        function initFunctionsGrid() {
        const grid = document.getElementById('functionsGrid');
        grid.innerHTML = '';
        for (let i = 0; i < totalFunctions; i++) {
            const item = document.createElement('div');
            item.className = 'func-item';
            
            let defaultText = '';
            let defaultChecked = '';

            if (i === 0) defaultText = 'Light';
            if (i === 1) defaultText = 'Bell';
            if (i === 2) { defaultText = 'Horn'; defaultChecked = 'checked'; }

            item.innerHTML = `
                <span class="func-label">F${i}</span>
                <input type="text" id="func_text_${i}" value="${defaultText}" placeholder="Function for F${i}" maxlength="20">
                <label class="func-moment" title="Momentary Function">
                    <input type="checkbox" id="func_star_${i}" ${defaultChecked}> 
                </label>
            `;
            grid.appendChild(item);
        }
    }
    function rosterSelectLoco(loco) {
        if (!window.roster || !roster[loco]) return;
        rosterCurrent = String(loco);
        let name=roster[loco]["name"];
        let functions = roster[loco]["functions"];
        const nummer = roster[loco]["number"];
        
        if (functions.endsWith('/')) {
            functions = functions.slice(0, -1);
        }

        document.getElementById('locoNumber').value = loco;
        document.getElementById('locoName').value = name;

        const parts = functions.split('/');

        for (let i = 0; i < totalFunctions; i++) {
            let textInput = document.getElementById(`func_text_${i}`);
            let starCheckbox = document.getElementById(`func_star_${i}`);

            if (i < parts.length) {
                let val = parts[i];
                
                if (val.startsWith('*')) {
                    starCheckbox.checked = true;
                    val = val.substring(1);
                } else {
                    starCheckbox.checked = false;
                }

                textInput.value = val;
            } else {
                textInput.value = '';
                starCheckbox.checked = false;
            }
        }
        window.setConfigPanelDirty(false);
    }
    async function rosterRefresh() {
        window.rosterDeleted = [];
        try {
            const response = await fetch('/rosterValues.js', { cache: 'no-store' });
            if (!response.ok) throw new Error('HTTP ' + response.status);
            const source = await response.text();
            window.roster = new Function(source + '\nreturn roster;')();
        } catch (err) {
            console.error('Roster load failed:', err);
            window.roster = [];
        }
        rosterFillSelector();
    }
    function rosterFillSelector() {
        const sel = document.getElementById('locoSelector');
        sel.innerHTML = '';
        const ids = Object.keys(window.roster || {});
        if (ids.length === 0) {
            sel.innerHTML = '<option value="">-- No locomotives loaded yet --</option>';
            return;
        }
        for (const id of ids) {
            const opt = document.createElement('option');
            opt.value = id;
            opt.textContent = id + ' - ' + roster[id]["name"];
            sel.appendChild(opt);
        }
        rosterSelectLoco(sel.value);
    }
    var rosterCurrent = '';
    function rosterFunctionMap() {
        const parts = [];
        for (let i = 0; i < totalFunctions; i++) {
            const text = document.getElementById(`func_text_${i}`).value.trim();
            const star = document.getElementById(`func_star_${i}`).checked;
            parts.push(text === '' ? '' : (star ? '*' : '') + text);
        }
        while (parts.length > 0 && parts[parts.length - 1] === '') parts.pop();
        return parts.join('/');
    }
    function rosterUpdateEntry() {
        const entry = window.roster && window.roster[rosterCurrent];
        if (!entry) return;
        entry.name = document.getElementById('locoName').value;
        entry.functions = rosterFunctionMap();
        entry.modified = true;

        // A changed address re-keys the entry, remembering the id it was loaded under.
        const newId = parseInt(document.getElementById('locoNumber').value, 10);
        if (!Number.isNaN(newId) && newId >= 0 && newId <= 10239 && String(newId) !== String(rosterCurrent) && !(newId in window.roster)) {
            if (entry.oldId === undefined) entry.oldId = parseInt(rosterCurrent, 10);
            window.roster[newId] = entry;
            delete window.roster[rosterCurrent];
            rosterCurrent = String(newId);
        }

        const sel = document.getElementById('locoSelector');
        const opt = sel.options[sel.selectedIndex];
        if (opt) {
            opt.value = rosterCurrent;
            opt.textContent = rosterCurrent + ' - ' + entry.name + ' *';
        }
    }
    function dialogSave() {
        const esc = s => s.replace(/\\/g, '\\\\').replace(/"/g, '\\"');
        const ids = Object.keys(window.roster || {});

        const blank = ids.find(id => window.roster[id].modified && window.roster[id].name.trim() === '');
        if (blank !== undefined) {
            rosterSelectLoco(blank);
            document.getElementById('locoSelector').value = blank;
            document.getElementById('locoName').focus();
            alert('Loco ' + blank + ' needs a name before the roster can be saved.');
            return '';
        }

        // Deletes go first so a replacement can reuse an id that was deleted or re-addressed.

        const deletes = new Set(window.rosterDeleted || []);
        ids.forEach(id => {
            const oldId = window.roster[id].oldId;
            if (window.roster[id].modified && oldId !== undefined) deletes.add(oldId);
        });

        let command = '';
        deletes.forEach(id => { command += `<JR ${id} DELETE>`; });
        ids.forEach(id => {
            const entry = window.roster[id];
            if (!entry.modified) return;
            command += `<JR ${id} "${esc(entry.name)}" "${esc(entry.functions)}">`;
        });
        if (command === '') return '';
        command += '<JR SAVE>';

        console.log('Roster save:', command);
        fetch('/parse', {
            method: 'POST',
            headers: { 'Content-Type': 'application/txt' },
            body: command
        })
            .then(response => {
                if (!response.ok) throw new Error('HTTP ' + response.status);
                return response.text();
            })
            .then(() => {
                if (typeof window.setConfigPanelDirty === 'function') window.setConfigPanelDirty(false);
                return rosterRefresh();
            })
            .catch(err => console.error('Failed to save roster:', err));
        return command;
    }
    function rosterNewLoco() {
        if (!window.roster) window.roster = [];
        let newId = 0;
        while (newId in window.roster) newId++;

        window.roster[newId] = { name: '', functions: '', modified: true };

        const sel = document.getElementById('locoSelector');
        // Drop the "no locomotives" placeholder if present.
        Array.from(sel.options).forEach(o => { if (o.value === '') o.remove(); });
        const opt = document.createElement('option');
        opt.value = String(newId);
        opt.textContent = newId + ' - ' + ' *';
        sel.appendChild(opt);
        sel.value = String(newId);
        rosterSelectLoco(String(newId));
        if (typeof window.setConfigPanelDirty === 'function') window.setConfigPanelDirty(true);
        document.getElementById('locoNumber').focus();
        document.getElementById('locoNumber').select();
    }
    function rosterCopyLoco() {
        const src = window.roster && window.roster[rosterCurrent];
        if (!src) return;
        // New id is the next unused address after the one being copied.
        let newId = parseInt(rosterCurrent, 10) + 1;
        while (newId in window.roster) newId++;

        window.roster[newId] = { name: src.name, functions: src.functions, modified: true };

        const sel = document.getElementById('locoSelector');
        const opt = document.createElement('option');
        opt.value = String(newId);
        opt.textContent = newId + ' - ' + src.name + ' *';
        sel.appendChild(opt);
        sel.value = String(newId);
        rosterSelectLoco(String(newId));
        if (typeof window.setConfigPanelDirty === 'function') window.setConfigPanelDirty(true);
        document.getElementById('locoNumber').focus();
        document.getElementById('locoNumber').select();
    }
    function rosterDeleteLoco() {
        const sel = document.getElementById('locoSelector');
        const id = sel.value;
        if (!id || !window.roster[id]) return;
        const entry = window.roster[id];
        // An entry that was re-addressed is deleted under the id the device knows.
        window.rosterDeleted.push(entry.oldId !== undefined ? entry.oldId : parseInt(id, 10));
        delete window.roster[id];
        sel.remove(sel.selectedIndex);
        if (sel.options.length > 0) {
            sel.selectedIndex = 0;
            rosterSelectLoco(sel.value);
        } else {
            rosterCurrent = '';
            sel.innerHTML = '<option value="">-- No locomotives loaded yet --</option>';
        }
        if (typeof window.setConfigPanelDirty === 'function') window.setConfigPanelDirty(true);
    }
    window.dialogSave = dialogSave;
    initFunctionsGrid();
    ['locoNumber', 'locoName', 'functionsGrid'].forEach(function(id) {
        document.getElementById(id).addEventListener('input', function() {
            rosterUpdateEntry();
            if (typeof window.setConfigPanelDirty === 'function') window.setConfigPanelDirty(true);
        });
    });
    rosterRefresh();
    
    </script>

)???";
