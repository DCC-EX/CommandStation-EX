String CVTTableEditor_html=R"???(
<div class="cv-editor">
  <style>
    .cv-editor .form-group {
      margin-bottom: 10px;
    }
    .cv-editor label {
      display: block;
      margin-bottom: 4px;
      font-weight: bold;
    }
    .cv-editor input,
    .cv-editor select,
    .cv-editor button {
      font: inherit;
    }
  </style>

  <div class="form-group">
    <label for="cvNumberSelect">CV Number:</label>
    <select id="cvNumberSelect"></select>
  </div>

  <div class="form-group">
    <label for="cvValueInput">CV Value:</label>
    <input id="cvValueInput" type="number" min="0" max="32767" step="1" />
  </div>

  <button type="button" onclick="saveCurrentCvValue();SaveCvValues()">Save</button>
</div>

<script>

  const cvSelect = document.getElementById('cvNumberSelect');
  const cvValueInput = document.getElementById('cvValueInput');

  let currentCv = 0;

  function saveCurrentCvValue() {
    const cv = currentCv;
    const value = parseInt(cvValueInput.value, 10);

    if (!Number.isNaN(value) && value >= 0 && value <= 32767) {
      CVTable[cv] = value;
    }
  }

  function loadCvValue(cv) {
    cvValueInput.value = CVTable[cv] ?? 0;
  }

  for (let cv = 0; cv <= 255; cv++) {
    const opt = document.createElement('option');
    opt.value = cv;
    opt.textContent = 'CV ' + cv;
    cvSelect.appendChild(opt);
  }

  cvSelect.addEventListener('change', function() {
    saveCurrentCvValue();
    currentCv = parseInt(cvSelect.value, 10);
    loadCvValue(currentCv);
  });

  loadCvValue(currentCv);
  cvSelect.value = currentCv;

</script>
)???";
