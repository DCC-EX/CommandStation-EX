String CVTTableEditor_html=R"???(
<div class="cv-editor">
  <style>
    .cv-editor {
      font-family: inherit;
    }

    .cv-editor .controls {
      display: flex;
      align-items: center;
      gap: 8px;
      margin-bottom: 10px;
    }

    .cv-editor .cv-grid {
      display: grid;
      grid-template-columns: repeat(16, minmax(80px, 1fr));
      gap: 0px;
      max-width: 1400px;
    }

    .cv-editor .cv-cell {
      display: flex;
      flex-direction: column;
      gap: 0px;
      padding: 0px;
      border: 1px solid #ccc;
      background: #0000;
    }

    .cv-editor .cv-cell label {
      font-size: 0.8rem;
      font-weight: bold;
    }

    .cv-editor .cv-cell input {
      width: 100%;
      box-sizing: border-box;
      font: inherit;
    }

    .cv-editor .cv-cell input[type="number"] {
      -moz-appearance: textfield;
      appearance: textfield;
    }

    .cv-editor .cv-cell input[type="number"]::-webkit-outer-spin-button,
    .cv-editor .cv-cell input[type="number"]::-webkit-inner-spin-button {
      -webkit-appearance: none;
      margin: 0;
    }

    .cv-editor button {
      font: inherit;
    }
  </style>

  <div class="controls">
    <p>This is a generic editor for Command Station or Node configuration values.
     It allows you to view and edit the values of CVs 0–255 directly.
     The meaning of the CVs depends on the specific firmware and EXRAIL configuration of your Command Station or Node.
     <br/>
     This is not related in any way to DCC Locomotive or Accessory CVs. 
    <br/>
    Enter values for CVs 0–255 then -> 
    <button type="button" onclick="saveAllCvValues()">Save</button>
    </p>
  </div>

  <div id="cvGrid" class="cv-grid"></div>

  <script>
    (function() {
      const grid = document.getElementById('cvGrid');
      if (!grid) return;

      function createCvCell(cv) {
        const cell = document.createElement('div');
        cell.className = 'cv-cell';

        const label = document.createElement('label');
        label.textContent = 'CV ' + cv;

        const input = document.createElement('input');
        input.type = 'number';
        input.min = '-32768';
        input.max = '32767';
        //input.step = '1';
        input.value = (typeof CVTable !== 'undefined' && CVTable[cv] != null) ? CVTable[cv] : 0;

        input.addEventListener('input', function() {
          const value = parseInt(this.value, 10);
          if (!Number.isNaN(value) && value >= -32768 && value <= 32767) {
            CVTable[cv] = value;
          } else {
            CVTable[cv] = 0;
          }
        });

        cell.appendChild(label);
        cell.appendChild(input);
        return cell;
      }

      for (let cv = 0; cv <= 255; cv++) {
        grid.appendChild(createCvCell(cv));
      }

      window.saveAllCvValues = function() {
        SaveCvValues();
      };
    })();
  </script>
</div>
)???";
