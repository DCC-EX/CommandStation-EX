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

<label for="locoSelector">Select a loaded locomotive to edit or delete:</label>
            <div class="selector-controls">
                <select id="locoSelector" onchange="rosterSelectLoco(this.value)">
                    <option value="">-- No locomotives loaded yet --</option>
                </select>
                <button class="btn-inline-danger" onclick="rosterDeleteLoco()">🗑 Delete Locomotive</button>
            </div>
        </div>

        <div class="form-row">
           <label for="locoNumber">DCC Address:</label>
                <input type="number" id="locoNumber" placeholder="e.g. 3" value="3" 
                        min="1" max="10239">
                <label for="locoId">ID:</label>
                <input type="text" id="locoId" placeholder="e.g. Thomas" value="Thomas">
            </div>
        </div>
    </div>

    <!-- Functions F0 to F32 -->
    <div class="section">
        <h2>Decoder Functions (F0 to F32)</h2>
        <p class="hint">Here you see all functions from F0 to F32. Check the asterisk (*) for momentary functions.</p>
        
        <div class="functions-grid" id="functionsGrid"></div>

        <div style="margin-top: 10px;">
            <button class="btn" onclick="rosterSave()">Generate Roster Line</button>
            <button class="btn btn-danger" onclick="rosterSelectLoco(locoSelector.value)">RESET</button>
        </div>
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
                    <input type="checkbox" id="func_star_${i}" ${defaultChecked}> *
                </label>
            `;
            grid.appendChild(item);
        }
    }
    function rosterSelectLoco(loco) {
        let name=roster[loco]["name"];
        let functions = roster[loco]["functions"];
        const nummer = roster[loco]["number"];
        
        if (functions.endsWith('/')) {
            functions = functions.slice(0, -1);
        }

        document.getElementById('locoNumber').value = loco;
        document.getElementById('locoId').value = name;

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
    function dialogSave() {
        const loco = parseInt(document.getElementById('locoNumber').value, 10);
        if (Number.isNaN(loco)) return '';
        const esc = s => s.replace(/\\/g, '\\\\').replace(/"/g, '\\"');
        const name = esc(document.getElementById('locoId').value);

        const parts = [];
        for (let i = 0; i < totalFunctions; i++) {
            const text = document.getElementById(`func_text_${i}`).value.trim();
            const star = document.getElementById(`func_star_${i}`).checked;
            parts.push(text === '' ? '' : (star ? '*' : '') + text);
        }
        while (parts.length > 0 && parts[parts.length - 1] === '') parts.pop();
        const functionMap = esc(parts.join('/'));

        const command = `<JR ${loco} "${name}" "${functionMap}">`;
        console.log('Roster save:', command);
        fetch('/saveRoster', {
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
    window.dialogSave = dialogSave;
    initFunctionsGrid();
    ['locoNumber', 'locoId', 'functionsGrid'].forEach(function(id) {
        document.getElementById(id).addEventListener('input', function() {
            if (typeof window.setConfigPanelDirty === 'function') window.setConfigPanelDirty(true);
        });
    });
    rosterRefresh();
    
    </script>

)???";
