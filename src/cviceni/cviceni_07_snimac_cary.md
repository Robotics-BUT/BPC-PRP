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
* Znalost práce s daty v MATLABu/GNU Octave/Excelu nebo jiném SW pro analýzu dat.

## Výstupy
* Spolehlivě fungující měření pozice vodicí čáry vzhledem k ose jízdy robotu s jedním optočlenem CNY70.
* Základní verze měřicího systému složeného z více optočlenů CNY70 (např. diferenční zapojení se 2 ks CNY70).

## Odevzdání výsledku řešení cvičení (do pátku 2. dubna 2021)
* Do 'odevzdávací' složky v repozitáři Vašeho projektu uložte:
  - graf změřené převodní charakteristiky samotného senzoru bez jakéhokoliv zpracování,
  - graf převodní charakteristiky senzoru po Vašem zpracování,
  - textový soubor formátovaný v jazyce Markdown, který krátce popisuje Vámi zvolený způsob zpracování dat ze senzoru a odkazuje se na dva předchozí grafy (krátká technická zpráva, rozsah postačí ekv. 1/2 A4, mnohem důležitější je úroveň obsahu, než množství textu).

## Úkol č. 1: Realizace měření převodní charakteristiky senzoru CNY70
Proveďte implementaci automatizovaného měření převodní charakteristiky emulovaného senzoru CNY70. Jedná se o závislost `u_raw=f(dD)` nebo `u_raw=f(dA)`, kde `u_raw` je celočíselná hodnota reprezentující měřené napětí, která je získána z AD převodníku s 12 bitovým registrem pro uložení výsledku AD převodu. Veličina `dD` odpovídá délkové odchylce senzoru od vodicí čáry. Veličina `dA` odpovídá úhlové odchylce senzoru od vodicí čáry.

Hodnotu, která je výsledkem A/D převodu, získáte pomocí NMEA zprávy `SENSOR`, která je ve formátu: `$SENSOR,<id>*<chksum>`, kde `id` odpovídá pořadovému číslu senzoru, který je součástí robotu. Senzory jsou indexované od nuly. Registr, ze kterého jsou data čteny, je 12bitový. Skutečné rozlišení převodníku je ale pouze 10bitové, spodní 2 bity jsou tedy vždy nulové. Emulátor navrací zprávu `$SENSOR,<id>,<value>*<chksum>`, kde `value` je celočíselná hodnota, která je výsledkem A/D převodu.

### Možný způsob řešení (nápověda)
Charakteristiku `u_raw=f(dD)` můžete získat např. tak, že robotem nakolmo přejedete čáru a uložíte si změřené hodnoty `u_raw`, `dGamma1` a `dGamma2`. `dGamma1` a `dGamma2` jsou počty mikrokroků ujeté každým kolem během příslušné periody vzorkování. Charakteristiku `u_raw=f(dA)` můžete získat obdobným způsobem, t.j. robot stojí na místě a otáčí se kolem své osy.

## Úkol č. 2: Návrh detekčního systému s jedním optočlenem CNY70 pro měření pozice čáry vzhledem k ose jízdy robotu
Navrhněte vlastní systém pro měření pozice čáry s využitím optočlenu CNY70. Pro snažší pochopení chování optočlenu v uvedené aplikaci Vám může pomoci proměření vlivu vzdálenosti odrazné roviny na převodní charakteristiku. Umístěte senzor na robot tak, aby bylo možné snímat vychýlení optočlenu od osy vodící čary v co největším rozsahu a současně spolehlivě! Výstupní hodnotu algoritmu pro zpracování měřených dat kalibrujte v metrech příp. milimetrech.

## Úkol č. 3: Návrh detekčního systému s více optočleny CNY70 pro měření pozice/orientace čáry vzhledem k ose jízdy robotu

Pro zvýšení měřicího rozsahu a robustnosti měření je možné použít více optočlenů CNY70, které budou vhodně prostorově rozmístěny. Nejprve je doporučeno vyzkoušet diferenční zapojení dvou optočlenů a až potom se pouštět do sofistikovanějších zapojení s více optočleny.

Při návrhu měřicího systému vhodného pro detekci pozice/orientace vodicí čáry je doporučeno zvážit minimálně tyto parametry/vlastnosti:

* minimální počet potřebných optočlenů CNY70,
* prostorové rozmístění optočlenů CNY70 (jejich souřadnice `x`, `y`, `z` v souřadnicovém systému robotu),
* linearizace převodní charakteristiky jednotlivých optočlenů/celého detekčního systému,
* kalibrace jednotlivých optočlenů (statická - po zapnutí / dynamická - po celou dobu jízdy),
* způsob vyhodnocení dat ze senzoru - charakter výstupní veličiny (počet diskrétních stavů - 2 a nebo více).

## Nápověda č. 1: Zápis naměřených dat do souboru
Naměřená data je doporučeno prozatím logovat do souborů. Kdo zvládne logovat přes síť, může použít i tento způsob. Real-time sběr telemetrie bude náplní jednoho z dalších cvičení.

Pro snadnou, rychlou a bezproblémovou práci je doporučeno ukládat měřená data do snáze čitelného formátu souboru, který bude používán v aplikaci, kterou budete naměřená data analyzovat. Doporučen je primárně `MATLAB` nebo `GNU Octave`. Kdo s tím má problém, je to jeho problém. Příznivci jazyka Python mohou použít např. prostředí `Spyder` a v něm balíček `NumPy`. V nouzi postačí i tabulkový procesor (`MS Office Excel`, `LibreOffice Calc`, atd.).

### Formát CSV (Comma-separated values) souboru

Čísla jsou v textovém vyjádření, desetinným oddělovačem je tečka, odělovačem hodnot v řádku je čárka, oddělovačem nových řádků v tabulce je konec řádku `<CR><LF>` pro platformu Windows, pro Linux je to jen `<LF>`.

### Zápis dat do souboru v jazyce C

    #include <stdio.h>
    FILE *fopen(const char *pathname, const char *mode);
    int fprintf(FILE *stream, const char *format, ...);
    int fflush(FILE *stream);
    int fclose(FILE *stream);

## Nápověda č. 2: Přístup k souborům a jejich přenos mimo OS Linux

Naměřená data, které jste si uložili v OS Linux budete chtít nejspíše analyzovat mimo tento systém. Uložené soubory tedy budete v tomto případě potřebovat přenést na jiné zařízení nebo např. z hostovaného OS (Ubuntu), který vám běží ve VM do hostujícího OS (např. Windows), na kterém běžně pracujete. Existuje několik způsobů jak toto realizovat:
* Zpřístupnit lokální složku v OS Linux prostřednictvím protokolu SMB (externí přístup z platforem Windows, Linux i Mac k souborům na Ubuntu přes sdílenou síťovou složku)
* Přenést soubory přes síť s využitím protokolu SFTP (dostupné aplikace: `WinSCP`, `FileZilla`, `psftp`, `Total Commander` + `SFTP plugin`, atd.)
* Soubory přenést na externí úložiště připojené v Ubuntu přes rozhraní USB (flash disk, HDD/SSD) nebo síť (NAS, atd.)

## Nápověda č. 3: Zobrazení naměřených dat v MATLABu / GNU Octave

### Načtení souboru ve formátu CSV s naměřenými daty

    M = csvread(filename)

### Vykreslení grafu

    figure(f)
    clf
    plot(X, Y, LineSpec)
    plot(X, Y, LineSpec, 'LineWidth', width, 'MarkerSize', size)
    xlabel(txt)
    ylabel(txt)
    title(txt)
    legend(label1,...,labelN)

## Nápověda č. 4: Textový bargraf v konzoli

    // funkce na vytvoreni jedne hodnoty grafu do konzole
    // buffer ... predalokovany pracovni buffer do ktereho se bude zapisovat text
    // nchars ... pocet znaku v bufferu
    // val ... hodnota (0...1) k vytisteni
    // navratovou hodnotu lze rovnou pouzit jako parametr do printf ("%s", s)
    const char *graf(char* buffer, int nchars, float value)
    {
      int end = (int)(value * nchars) - 3; // start pipe, end pipe a koncova nula
 
      buffer[0] = '|';
      int i=1;
 
      while (i < end)
        buffer[i++] = '=';
 
      buffer[i++] = '|';
 
      while (i < nchars)
        buffer[i++] = ' ';
 
      buffer[nchars-1] = 0;
      return buffer;
    }

    printf("%d %%: %s", value, graf(buffer, 20, value));
