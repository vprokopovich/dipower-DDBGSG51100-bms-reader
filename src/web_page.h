#pragma once
#include <pgmspace.h>

static const char INDEX_HTML[] PROGMEM = R"HTML(<!doctype html>
<html lang="ru"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>BMS монитор</title>
<style>
:root{--bg:#f4f5f7;--card:#fff;--fg:#1c1e21;--mut:#6b7280;--ok:#16a34a;--warn:#d97706;--bad:#dc2626;--line:#e5e7eb}
@media(prefers-color-scheme:dark){:root{--bg:#111315;--card:#1b1e22;--fg:#e8eaed;--mut:#9aa0a6;--line:#2c3036}}
*{box-sizing:border-box}body{margin:0;padding:16px;background:var(--bg);color:var(--fg);font:15px/1.4 system-ui,sans-serif}
main{max-width:760px;margin:auto}h1{font-size:18px;margin:0 0 12px;display:flex;justify-content:space-between;align-items:center}
.pill{font-size:12px;padding:3px 10px;border-radius:99px;background:var(--line);color:var(--mut)}
.pill.ok{background:var(--ok);color:#fff}.pill.bad{background:var(--bad);color:#fff}
.card{background:var(--card);border-radius:12px;padding:16px;margin-bottom:12px}
.soc{display:flex;align-items:baseline;gap:12px}.soc b{font-size:52px;line-height:1}
.bar{height:12px;border-radius:6px;background:var(--line);overflow:hidden;margin-top:12px}
.bar i{display:block;height:100%;background:var(--ok);transition:width .5s}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(150px,1fr));gap:12px}
.k{color:var(--mut);font-size:12px}.v{font-size:20px;font-weight:600}
.cells{display:grid;grid-template-columns:repeat(8,1fr);gap:6px;font-size:12px;text-align:center}
.cells div{padding:6px 2px;border-radius:6px;background:var(--bg)}
.cells .lo{outline:2px solid var(--warn)}.cells .hi{outline:2px solid var(--ok)}
.alarm{color:var(--bad);font-weight:600}.mut{color:var(--mut);font-size:12px}a{color:inherit}
@media(max-width:480px){.cells{grid-template-columns:repeat(4,1fr)}}
</style></head><body><main>
<h1>Батарея Dipower 51.2 В <span id="st" class="pill">…</span></h1>
<div class="card"><div class="soc"><b id="soc">–</b><span id="mode"></span></div>
<div class="bar"><i id="bar" style="width:0"></i></div><div class="mut" id="eta" style="margin-top:8px"></div></div>
<div class="card grid">
<div><div class="k">Напряжение</div><div class="v" id="u">–</div></div>
<div><div class="k">Ток</div><div class="v" id="i">–</div></div>
<div><div class="k">Мощность</div><div class="v" id="p">–</div></div>
<div><div class="k">Остаток / ёмкость</div><div class="v" id="cap">–</div></div>
<div><div class="k">Температура</div><div class="v" id="t">–</div></div>
<div><div class="k">Циклов / SOH</div><div class="v" id="cyc">–</div></div>
</div>
<div class="card"><div class="k" id="ch">Ячейки</div><div class="cells" id="cells" style="margin-top:8px"></div></div>
<div class="card"><div class="k">Состояние BMS</div><div id="fl" style="margin-top:6px"></div><div id="al" class="alarm"></div></div>
<p class="mut" id="ft"></p>
<p class="mut"><a href="/api/telemetry">JSON</a> · <a href="/log">журнал</a> · <a href="#" onclick="fetch('/api/rescan',{method:'POST'});return false">искать BMS заново</a></p>
</main><script>
const $=id=>document.getElementById(id),f=(v,d)=>v==null?'–':v.toFixed(d);
const yn=v=>v==null?'?':v?'вкл':'выкл';
async function tick(){let d;try{d=await(await fetch('/api/telemetry')).json()}catch(e){$('st').textContent='нет связи с устройством';$('st').className='pill bad';return}
const s=$('st');s.textContent=d.online?'на связи':(d.searching?'поиск BMS…':'BMS не отвечает');s.className='pill '+(d.online?'ok':'bad');
$('ft').textContent=`${d.protocol}, адрес ${d.address}, ${d.baud} бод · опросов ${d.ok_count}, ошибок ${d.err_count} · Wi-Fi ${d.rssi} дБм · работает ${Math.round(d.uptime_s/60)} мин`;
if(d.soc==null)return;
$('soc').textContent=f(d.soc,0)+'%';$('bar').style.width=d.soc+'%';
$('bar').style.background=d.soc<20?'var(--bad)':d.soc<40?'var(--warn)':'var(--ok)';
$('mode').textContent={charging:'⚡ заряжается',discharging:'↓ разряжается',idle:'покой'}[d.state]||'';
$('eta').textContent=d.eta_min==null?'':(d.state=='charging'?'до полного заряда ≈ ':'до разряда ≈ ')+Math.floor(d.eta_min/60)+' ч '+(d.eta_min%60)+' мин';
$('u').textContent=f(d.voltage,2)+' В';$('i').textContent=f(d.current,2)+' А';$('p').textContent=f(d.power,0)+' Вт';
$('cap').textContent=f(d.remaining_ah,1)+' / '+f(d.full_ah,1)+' А·ч';
const ts=(d.temps_c||[]).filter(x=>x!=null);$('t').textContent=ts.length?Math.min(...ts).toFixed(1)+'…'+Math.max(...ts).toFixed(1)+' °C':'–';
$('cyc').textContent=d.cycles+' / '+f(d.soh,0)+'%';
$('ch').textContent=`Ячейки: мин ${d.cell_min_mv} мВ, макс ${d.cell_max_mv} мВ, разброс ${d.cell_delta_mv} мВ`;
$('cells').innerHTML=(d.cells_mv||[]).map((v,n)=>`<div class="${v==d.cell_min_mv?'lo':v==d.cell_max_mv?'hi':''}">${n+1}<br><b>${(v/1000).toFixed(3)}</b></div>`).join('');
$('fl').textContent=`Заряд разрешён: ${yn(d.charge_fet)} · разряд разрешён: ${yn(d.discharge_fet)} · MOSFET ${f(d.temp_mosfet_c,1)} °C · окружение ${f(d.temp_env_c,1)} °C`;
$('al').textContent=[d.protections&&'Защита: '+d.protections,d.faults&&'Неисправность: '+d.faults,d.warnings&&'Предупреждение: '+d.warnings].filter(Boolean).join(' · ');
}
tick();setInterval(tick,3000);
</script></body></html>
)HTML";
