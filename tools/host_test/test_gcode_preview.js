// Ender Smart (lote 19): casos do gcodeParser() da tela (main/web/app.html), no PC com o Node.
// Uso (dentro de firmware/tools/host_test): node test_gcode_preview.js [../../main/web/app.html]
const fs = require('fs');
const html = fs.readFileSync(process.argv[2] || require('path').join(__dirname, '..', '..', 'main', 'web', 'app.html'), 'utf8');
const gcodeParser = new Function(html.slice(html.indexOf('function gcodeParser()'), html.indexOf('function drawGcodePreview(')) + '\nreturn gcodeParser;')();
let falhas = 0;
function roda(txt) { const p = gcodeParser(); txt.split('\n').forEach(l => p.line(l)); const r = p.end(); const s = []; for (let i = 0; i < r.n; i += 5) s.push(Array.from(r.seg.slice(i, i + 5)).map(v => +v.toFixed(3))); return { s, marks: r.marks }; }
function caso(nome, txt, esperado) {
  const r = roda(txt), got = JSON.stringify(r.s), exp = JSON.stringify(esperado);
  if (got === exp) console.log('ok    ' + nome); else { falhas++; console.log('FALHA ' + nome + '\n  veio     ' + got + '\n  esperado ' + exp); }
}
// absoluto (M82): extrusão quando E cresce
caso('absoluto', ';LAYER:0\nG1 X0 Y0 Z0.2\nG1 X10 Y0 E1\nG1 X10 Y10 E1\nG1 X0 Y10 E2', [[0, 0, 10, 0, 0.2], [10, 10, 0, 10, 0.2]]);
// relativo (M83): E>0 extruda; E igual a 0 não
caso('relativo M83', 'M83\n;LAYER:0\nG1 X0 Y0 Z0.2\nG1 X10 E0.5\nG1 X20 E0\nG1 X30 E0.5', [[0, 0, 10, 0, 0.2], [20, 0, 30, 0, 0.2]]);
// Marlin 2.0: G90 depois de M83 volta o E para absoluto
caso('M83 depois G90', 'M83\nG90\n;LAYER:0\nG1 X0 Y0 Z0.2 E5\nG1 X10 E5\nG1 X20 E6', [[10, 0, 20, 0, 0.2]]);
// G91 deixa tudo relativo, inclusive o E
caso('G91', ';LAYER:0\nG1 X5 Y5 Z0.2\nG91\nG1 X10 E1\nG1 Y10 E1\nG90', [[5, 5, 15, 5, 0.2], [15, 5, 15, 15, 0.2]]);
// G92 E0 zera o extrusor
caso('G92', ';LAYER:0\nG1 X0 Y0 Z0.3\nG1 X10 E10\nG92 E0\nG1 X20 E1', [[0, 0, 10, 0, 0.3], [10, 0, 20, 0, 0.3]]);
// linha de purga antes da 1ª camada fica de fora
caso('purga', 'G1 X0.1 Y20 Z0.3\nG1 X0.1 Y200 E15\n;LAYER:0\nG1 X50 Y50 Z0.2\nG1 X60 Y50 E16', [[50, 50, 60, 50, 0.2]]);
// sem nenhuma marca de camada, tudo entra
caso('sem marcas', 'G1 X0 Y0 Z0.2\nG1 X10 Y0 E1', [[0, 0, 10, 0, 0.2]]);
// contorno: Cura SKIRT, Orca Skirt, Prusa Skirt/Brim
caso('skirt Cura', ';LAYER:0\nG1 X0 Y0 Z0.2\n;TYPE:SKIRT\nG1 X10 E1\n;TYPE:WALL-OUTER\nG1 X20 E2', [[10, 0, 20, 0, 0.2]]);
caso('skirt Prusa', ';LAYER_CHANGE\nG1 X0 Y0 Z0.2\n;TYPE:Skirt/Brim\nG1 X10 E1\n;TYPE:External perimeter\nG1 X20 E2', [[10, 0, 20, 0, 0.2]]);
// Orca: a linha de configuração "; layer_change_gcode = ;LAYER_CHANGE" no início não é marca de camada
caso('Orca config', '; layer_change_gcode = ;LAYER_CHANGE\nG1 X0.1 Y20 Z0.3\nG1 X0.1 Y200 E15\n;LAYER_CHANGE\nG1 X50 Y50 Z0.2\nG1 X60 Y50 E16', [[50, 50, 60, 50, 0.2]]);
// "G28 X" sem número: só o X vai a 0
caso('G28 X', 'G1 X50 Y50 Z5\nG28 X\n;LAYER:0\nG1 Z0.2\nG1 Y60 E1', [[0, 50, 0, 60, 0.2]]);
// curvas: trechos de 0,1 mm viram cordas de 0,3 mm
caso('cordas', ';LAYER:0\nG1 X0 Y0 Z0.2\nG1 X0.1 E0.1\nG1 X0.2 E0.2\nG1 X0.3 E0.3\nG1 X0.4 E0.4', [[0, 0, 0.3, 0, 0.2], [0.3, 0, 0.4, 0, 0.2]]);
// sem espaços e com comentário no fim da linha
caso('compacto', ';LAYER:0\nG1X0Y0Z0.2\nG1X10Y0E1 ; parede', [[0, 0, 10, 0, 0.2]]);
// mudança de Z quebra a sequência
caso('troca de camada', ';LAYER:0\nG1 X0 Y0 Z0.2\nG1 X10 E1\n;LAYER:1\nG1 Z0.4\nG1 X0 E2', [[0, 0, 10, 0, 0.2], [10, 0, 0, 0, 0.4]]);
console.log(falhas ? falhas + ' FALHARAM' : 'todos passaram');
process.exit(falhas ? 1 : 0);
