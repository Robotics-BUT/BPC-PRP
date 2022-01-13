# Přístup k hardware - I2C, GPIO, ADC
Cvičící: Ing. František Burian Ph.D.

## Cile
* Pochopit princip sběrnice I2C která propojuje součásti robotu
* Pochopit činnost GPIO expanderu
* Pochopit činnost A/D převodníku

## Prerekvizity
* Předinstalované Raspberry PI s HATem v laboratoři

## I2C (45 minut)

### Aktivace I2C hardwarového, softwarového
### Detekce všech připojených zařízení na I2C sběrnici
### Stažení vzorového projektu

Upravte vzorový projekt tak, aby obsahoval následující kód:

```
using namespace RoboUtils;

int main()
{
  I2C i2c{"/dev/i2c-3"};

  uint8_t value = 0;

  while (true) {
    i2c.write(0x20, 0x06, ++value);
    delay(100);
  }
}
```

### Odchycení zpráv na I2C osciloskopem, identifikace clock stretchingu

Připojte na osciloskop sběrnici I2C, osciloskop sesynchronizujte tak,
abyste viděli stabilní průběh jedné úplné zprávy a ten popište

## GPIO (45 minut)

### Stažení vzorového projektu

```
using namespace RoboUtils;

const auto LED1 = Pin::PB4;

int main()
{
  I2C i2c{"/dev/i2c-3"};
  GPIO gpio{&i2c};

  gpio.output(LED1);

  while (true) {
      gpio.high(LED1);
      delay(50);
      gpio.low(LED1);
      delay(50);
  }
}
```

### Reakce na tlačítko, ovládání běhu programu pomocí tlačítek

Zadefinujte tlačítka jako proměnné LBUT a RBUT, nastavte je jako
vstupní s pullupem

Po stisknutí tlačítka RBUT ukončete program

Přenastavte výstupní port pro PB5 do režimu "otevřený kolektor",
duplikujte stav LED1 a diskutujte s osciloskopem výsledek.

### ošetření zákmitů tlačítka

Vytvořte si proměnnou value, a inicializujte ji na nulu.

Vytvořte z portu PB0 - PB5 "bargraf" zobrazující hodnotu value bez 
zpoždění (odstraňte všechny delaye).

Upravte program tak, aby tlačítkem při každém stisknutí zvýšil
hodnotu value o 1 (a aktualizoval bargraf). Volitelně můžete
použít i tisk na obrazovku. Pozor na zákmity tlačítka !!

## ADC (45minut)

### Stažení vzorového projektu

### Princip měření

### Převod ADC hodnoty na napětí

### Vliv vstupního děliče

### Dosažitelná přesnost měření

## Očekávané výstupy práce v tomto cvičení

✅ Jste schopni na reálném hardware nastavit I2C sběrnici

✅ Chápete možné problémy a důsledky použití sběrnice I2C

✅ Rozumíte obecným GPIO na obvyklých embedded zařízeních

✅ Dokážete ošetřit jednoduché GPIO periferie (tlačítko, LED)

✅ Rozumíte principu AD převodníku

✅ Dokážete binární hodnotu převodníku převést na reálnou měřenou hodnotu dle schematu