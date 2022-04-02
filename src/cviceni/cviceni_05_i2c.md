# Přístup k hardware - I2C, GPIO, ADC
Cvičící: Ing. František Burian Ph.D.

## Cile
* Pochopit princip sběrnice I2C která propojuje součásti robotu
* Pochopit činnost GPIO expanderu
* Pochopit činnost A/D převodníku

## Prerekvizity
* Předinstalované Raspberry PI s HATem v laboratoři

## I2C (30 minut)

### Aktivace I2C hardwarového, softwarového, device tree overlay

Konfigurace device tree overlayů se dělá v `/boot/config.txt`. Po každé změně tohoto souboru je potřeba rasoberry 
rebootovat příkazem `sudo reboot`

Pro aktivaci hardwarového i2c na pinech 2 a 3 platí následující výstřižek v `/boot/config.txt`:

```
  dtparam=i2c_arm=on
  dtparam=i2c_baudrate=10000
```

Pro zjištění aktuálního baudrate můžete použít `sudo cat /sys/kernel/debug/clk/clk_summary`

[ERRATA](https://elinux.org/BCM2835_datasheet_errata) Je třeba si všimnout že **errata nevydal výrobce čipu**, ten se k
němu nezná, je to levný výrobce!

**BUG1:**

Výrobce čipu implementoval chybně clock stretching, a pomalá periferie kompletně zruší přenos po sběrnici (vymaže jeden nebo více CLK pulsů). Sběrnice tedy musí běžet rychlostí podle nejpomalejší periferie.

**BUG2:**

Výrobce čipu neimplementoval správně nastavení rychlosti a od RpiV4 prakticky nejde zpomalit rychlost HW I2C sběrnice pod určitou mez (cca 100kbps - každé rpi to má jinak), ikdyž je v příkazu viz výše reportovaná hodnota jak byla nastavená. 

Díky těmto chybám prakticky nelze použít konfiguraci viz výše a musíme použít softwarově definované I2C, které těmito problémy netrpí. Použijte konfiguraci v `/boot/config.txt` (předchozí řádky musí být zakomentované aby se dvě implementace nebily):


```
  dtoverlay=i2c-gpio,i2c_gpio_sda=2,i2c_gpio_scl=3,i2c_gpio_delay_us=6,bus=3
```

Tato konfigurace vytvoří na pinech 2 a 3 sběrnici /dev/i2c-3 s periodou CLK pulsu 12us tj cca 90kHz, ale s výjimkou že bude 
funkční clock stretching, takže si pomalá periferie může sběrnici pozdržet (případ KM2, bude probírán na dalším cvičení)

Poznámka: Takto je v laboratoři Rpi připraveno, konfiguraci tedy nemusíte měnit, pouze ji zkontrolujte.

### Shell: Detekce všech připojených zařízení na I2C sběrnici

Pro rychlé zjištění stavu I2C existuje spousta dobrých nástrojů z balíku `i2c-tools`. Omezíme se pouze na autodetekci 
připojených čipů:

```bash
  i2cdetect -y 3
```

Vysvětlete z výpisu, a s použitím datasheetů, které čipy máte na sběrnici připojené (různé RPI budou mít různé výstupy!)

### Stažení vzorového projektu

Vzorový projekt obsahuje následující kód: 

```cpp
#include <roboutils/io/I2C.h>
#include <roboutils/util/timing.h>

using namespace RoboUtils;
using namespace RoboUtils::IO;

int main()
{
  I2C i2c;
  i2c.open("/dev/i2c-3");

  uint8_t value = 0;

  while (true) {
    i2c.write(0x20, 0x06, ++value);
    delay(100);
  }
}
```

spusťte jej, popište jeho funkci, prohlédněte si header, způsob jeho dokumentace a jaké možnosti Vám nabízí.

### Odchycení zpráv na I2C osciloskopem, identifikace clock stretchingu

Připojte na osciloskop sběrnici I2C, osciloskop sesynchronizujte tak,
abyste viděli stabilní průběh jedné úplné zprávy a ten popište.

Hledejte START, STOP, hodnoty jednotlivých bitů dat i adresy, ACK.

## GPIO (60 minut)

### Vzorový projekt

Založte na aktuáním "master" novou větev pojmenovanou "hw-cv5-gpio", přepněte se do ní a upravte kód vzorového projektu tak, 
aby obsahoval následující kód:

```cpp
#include <roboutils/io/GPIO.h>
#include <roboutils/util/timing.h>

using namespace RoboUtils;
using namespace RoboUtils::IO;

const auto LED1 = Pin::PB4;

int main()
{
  I2C i2c;
  GPIO gpio{i2c};
  
  i2c.open("/dev/i2c-3");

  gpio.output(LED1);

  while (true) {
      gpio.high(LED1);
      delay(50);
      gpio.low(LED1);
      delay(50);
  }
}
```

Popište jeho funkci, a ověřte ji spuštěním. Opět si prohlédněte dokumentaci a nabízené funkce.

COMMIT / PUSH

### Reakce na tlačítko, ovládání běhu programu pomocí tlačítek

Zadefinujte tlačítka jako proměnné LBUT a RBUT, nastavte je jako
vstupní s pullupem.

Po stisknutí tlačítka RBUT ukončete program

Přenastavte výstupní port pro PB5 do režimu "otevřený kolektor",
duplikujte stav LED1 a diskutujte viditelné rozdíly na pinech
PB4 a PB5 s použitím osciloskopu.

COMMIT / PUSH

### Ošetření zákmitů tlačítka

Vytvořte si proměnnou value, a inicializujte ji na nulu.

Vytvořte z portu PB0 - PB5 "bargraf" zobrazující hodnotu value v
každém cyklu supersmyčky okamžitě, tedy bez zpoždění (odstraňte
všechny delaye).

Upravte program tak, aby tlačítkem při každém **stisknutí** zvýšil
hodnotu value o 1. Volitelně můžete použít i tisk na obrazovku.
Pozor na zákmity tlačítka !!

COMMIT / PUSH

### NMEA (simulátor)

Pro obsluhu GPIO v simulátoru jsou vyhrazeny příkazy [BEEP](./../simulator/zpravy/BEEP.md), [LED](./../simulator/zpravy/LED.md), [BTN](./../simulator/zpravy/BTN.md), případně nízkoúrovňové [GPIO.GET](./../simulator/zpravy/GPIO.GET.md), [GPIO.SET](./../simulator/zpravy/GPIO.SET.md), [GPIO.DIR](./../simulator/zpravy/GPIO.DIR.md), avšak tyto nejsou nijak simulátorem zpracovávány. Implementujte je do svého programu pouze v okamžiku, pokud budete chtít získat 5b navíc jízdou reálného robotu na konci semestru.

## ADC (60minut)

### Vzorový projekt

Založte na aktuáním "master" novou větev pojmenovanou "hw-cv5-adc", přepněte se do ní a upravte kód vzorového projektu tak, 
aby obsahoval následující kód:

```cpp
#include <roboutils/io/ADC.h>
#include <roboutils/util/timing.h>

using namespace RoboUtils;
using namespace RoboUtils::IO;

int main()
{
  I2C i2c;
  ADC adc{i2c};
  
  i2c.open("/dev/i2c-3");

  while (true) {
    auto result = adc.Mode2Measure(0);
    std::cout << result[0] << std::endl;
    delay(100);
  }
}
```

Popište jeho funkci, a ověřte ji spuštěním. Opět si prohlédněte dokumentaci a nabízené funkce.

COMMIT / PUSH

### Princip měření

Popište, jak funguje A/D převodník, a jaké hodnoty na svém výstupu dává.

S pomocí datasheetu popište, minimální, a maximální čtenou hodnotu na pinu ADC2 a případně chybějící kódy. 
Ověřte na reálném hardware s pomocí dodaného obyčejného potenciometru.

### Vliv vstupního děliče

Podívejte se do schematu od desky MAINBOARD na dělič napětí na pinu ADC0 a spočítejte převodní konstantu, která
převede změřenou analogovou hodnotu na absolutní hodnotu napětí.

Konstantu implementujte v kódu a porovnejte s údajem zobrazeným na zdroji DIAMETRAL. Diskutujte
výsledky.

COMMIT / PUSH

### Dosažitelná přesnost měření

Na desku byly osazeny rezistory s tolerancí 5%. Spočítejte, jaká bude tolerance měřeného napětí
v 10 bitovém, 14bitovém a 18bitovém režimu měření ? U všech tolerancí počítejte s obdélníkovým
tolerančním polem (značně si usnadníte práci)

Odpovídá pozorovaný údaj vypočítané toleranci měření ?

### NMEA (simulátor)

Pro obsluhu analogově/digitálního převodníku v simulátoru je vyhrazen příkaz [SENSOR](./../simulator/zpravy/SENSOR.md). 
Vraťte se tedy ke kódu předchozího cvičení (jiný projekt) a zkuste změřit na reálném robotu (nebo i simulovaném doma) 
úroveň napájecího napětí, tentokráte s pomocí NMEA zpráv.

(tuto část můžete již dělat na simulátoru doma, nezapomeňte commitovat/ pushovat aby i ostatní změny viděli !)


# Očekávané výstupy práce v tomto cvičení

✅ Jste schopni na reálném hardware nastavit I2C sběrnici

✅ Chápete možné problémy a důsledky použití sběrnice I2C

✅ Rozumíte obecným GPIO na obvyklých embedded zařízeních

✅ Dokážete ošetřit jednoduché GPIO periferie (tlačítko, LED)

✅ Rozumíte principu AD převodníku

✅ Dokážete binární hodnotu převodníku převést na reálnou měřenou hodnotu dle schematu

✅ Dokážete diskutovat přesnost měření AD převodníku včetně vlivu měřicího řetězce