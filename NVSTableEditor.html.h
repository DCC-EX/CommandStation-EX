String NVSTTableEditor_html=R"???(
<div class="nvs-editor">
  <style>
    .nvs-editor {
      font-family: inherit;
    }

    .nvs-editor .controls {
      display: flex;
      align-items: center;
      gap: 8px;
      margin-bottom: 10px;
    }

    .nvs-editor .nvs-grid {
      display: grid;
      grid-template-columns: repeat(16, minmax(80px, 1fr));
      gap: 0px;
      max-width: 1400px;
    }

    .nvs-editor .nvs-cell {
      display: flex;
      flex-direction: column;
      gap: 0px;
      padding: 0px;
      border: 1px solid #ccc;
      background: #0000;
    }

    .nvs-editor .nvs-cell label {
      font-size: 0.8rem;
      font-weight: bold;
    }

    .nvs-editor .nvs-cell input {
      width: 100%;
      box-sizing: border-box;
      font: inherit;
    }

    .nvs-editor .nvs-cell input[type="number"] {
      -moz-appearance: textfield;
      appearance: textfield;
    }

    .nvs-editor .nvs-cell input[type="number"]::-webkit-outer-spin-button,
    .nvs-editor .nvs-cell input[type="number"]::-webkit-inner-spin-button {
      -webkit-appearance: none;
      margin: 0;
    }

    .nvs-editor button {
      font: inherit;
    }
  </style>

  <div class="controls">
    <p>This is a generic editor for Command Station or Node configuration values.
     It allows you to view and edit the values of NVS 0–255 directly.
     The meaning of the NVS depends on the specific firmware and EXRAIL configuration of your Command Station or Node.
     <br/>
     This is not related in any way to DCC Locomotive or Accessory NVS. 
    <br/>
    Enter values for NVS 0–255 then Save
    </p>
  </div>

  <div id="nvsGrid" class="nvs-grid"></div>

  <script>
    (function() {
      const grid = document.getElementById('nvsGrid');
      if (!grid) return;

      function createNvsCell(nvs) {
        const cell = document.createElement('div');
        cell.className = 'nvs-cell';

        const label = document.createElement('label');
        label.textContent = 'NVS ' + nvs;

        const input = document.createElement('input');
        input.type = 'number';
        input.min = '-32768';
        input.max = '32767';
        //input.step = '1';
        input.value = (typeof NVSTable !== 'undefined' && NVSTable[nvs] != null) ? NVSTable[nvs] : 0;

        input.addEventListener('input', function() {
          const value = parseInt(this.value, 10);
          if (!Number.isNaN(value) && value >= -32768 && value <= 32767) {
            NVSTable[nvs] = value;
          } else {
            NVSTable[nvs] = 0;
          }
        });

        cell.appendChild(label);
        cell.appendChild(input);
        return cell;
      }

      for (let nvs = 0; nvs <= 255; nvs++) {
        grid.appendChild(createNvsCell(nvs));
      }

      window.saveAllNvsValues = function() {
        SaveNvsValues();
      };
    })();
  </script>
</div>
)???";
