# Snímání čáry
Cvičící: Ing. Tomáš Jílek, Ph.D.

## Cíle
* Realizovat zpracování signálu z emulovaného optočlenu CNY70.
* Implementovat výpočet pozice/orientace čáry vzhledem k ose jízdy robotu s využitím jednoho nebo více emulovaných optočlenů CNY70.

## Prerekvizity
* Funkční komunikace se simulátorem (ověřit pomocí zprávy `PING`).
* Funkční parsování NMEA řetězců.
* Funkční ovládání motorů.
* Funkční vyčítání dat z emulovaného KM2 pro výpočet odometrie.
* znalost práce s daty v MATLABu/GNU Octave/Excelu nebo jiném SW pro analýzu dat.

## Výstupy
* Spolehlivě fungující měření pozice vodicí čáry vzhledem k ose jízdy robotu s jedním optočlenem CNY70,
* Základní verze měřicího systému složeného z více optočlenů CNY70 (např. diferenční zapojení se 2 ks CNY70).

## Úkol č. 1: Realizace měření převodní charakteristiky senzoru CNY70
Proveďte implementaci automatizovaného měření převodní charakteristiky emulovaného senzoru CNY70. Jedná se o závislost `u_raw=f(dD)` nebo `u_raw=f(dA)`, kde `u_raw` je celočíselná hodnota reprezentující měřené napětí, která je získána z AD převodníku s 12 bitovým registrem pro uložení výsledku AD převodu. Veličina `dD` odpovídá délkové odchylce senzoru od vodicí čáry. Veličina `dA` odpovídá úhlové odchylce senzoru od vodicí čáry.

### Možný způsob řešení (nápověda)
Charakteristiku `u_raw=f(dD)` můžete získat např. tak, že robotem nakolmo přejedete čáru a uložíte si změřené hodnoty `u_raw`, `dGamma1` a `dGamma2`. `dGamma1` a `dGamma2` jsou počty mikrokroků ujeté každým kolem během příslušné periody vzorkování. Charakteristiku `u_raw=f(dA)` můžete získat obdobným způsobem, t.j. robot stojí na místě a otáčí se kolem své osy.