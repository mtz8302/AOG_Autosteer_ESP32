//-------------------------------------------------------------------------------------------------
// Server Index Page for OTA update Apr 2023
//-------------------------------------------------------------------------------------------------

const char* serverIndex = 
"<body style='font-family: Verdana,sans-serif; font-size: 14px;'>"
"<div style='width:400px;padding:20px;border-radius:10px;border:solid 2px #e0e0e0;margin:auto;margin-top:20px;'>"
"<div style='width:100%;text-align:center;font-size:18px;font-weight:bold;margin-bottom:12px;'>ESP32 firmware update</div>"
"<div style='width:100%;text-align:center;font-size:10px;margin-bottom:12px;'>Version 7. Apr. 2023</div>"
"<form method='POST' action='#' enctype='multipart/form-data' id='upload-form' style='width:100%;margin-bottom:8px;'>"
"<input type='file' name='update'>"
"<input type='submit' value='Update' style='float:right;'>"
"</form>"
"<div style='width:100%;background-color:#e0e0e0;border-radius:8px;'>"
"<div id='prg' style='width:0%;background-color:#2196F3;padding:2px;border-radius:8px;color:white;text-align:center;'>0%</div>"
"</div>"
"</div>"
"</body>"
"<script>"
"var prg = document.getElementById('prg');"
"var form = document.getElementById('upload-form');"
"form.addEventListener('submit', e=>{"
"e.preventDefault();"
"var data = new FormData(form);"
"var req = new XMLHttpRequest();"
"req.open('POST', '/update');"
"req.upload.addEventListener('progress', p=>{"
"let w = Math.round((p.loaded / p.total)*100) + '%';"
"if(p.lengthComputable){"
"prg.innerHTML = w;"
"prg.style.width = w;"
"}"
"if(w == '100%') prg.style.backgroundColor = '#04AA6D';"
"});"
"req.send(data);"
"});"
"</script>";

