# Přístup k hardware - I2C, GPIO, ADC
Cvičící: Ing. František Burian Ph.D.

## Cile
* Pochopit princip sběrnice I2C která propojuje součásti robotu
* Pochopit činnost GPIO expanderu
* Pochopit činnost A/D převodníku

## Prerekvizity
* Předinstalované Raspberry PI s HATem v laboratoři

## I2C (45 minut)

* Aktivace I2C hardwarového, softwarového
* Detekce všech připojených zařízení na I2C sběrnici
* Stažení example komunikace s I2C
* Odchycení zpráv na I2C osciloskopem, identifikace clock stretchingu

## GPIO (45 minut)

* Stažení example blikání LED
* Reakce na tlačítko, ovládání běhu programu pomocí tlačítek
* Inkrement / dekrement, ošetření zákmitů tlačítka

## ADC (45minut)

* Stažení example měření ADC
* Princip měření
* Převod ADC hodnoty na napětí
* Vliv vstupního děliče
* Dosažitelná přesnost měření

## Očekávané výstupy práce v tomto cvičení

✅ JJste schopnii na reálném hardware nastavit I2C sběrnici

✅ Chápete možné problémy a důsledky použití sběrnice I2C

✅ Rozumíte obecným GPIO na obvyklých embedded zařízeních

✅ Dokážete ošetřit jednoduché GPIO periferie (tlačítko, LED)

✅ Rozumíte principu AD převodníku

✅ Dokážete binární hodnotu převodníku převést na reálnou měřenou hodnotu dle schematu