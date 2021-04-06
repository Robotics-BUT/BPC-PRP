# Návrh regulátoru
Cvičící: Ing. Tomáš Jílek, Ph.D.

## Cíle
* Implementovat ovládání a měření pozice+orientace pro diferenciálně řízený podvozek
* Navrhnout a implementovat spolehlivý regulátor pro sledování trajektorie

## Prerekvizity
* Funkční komunikace se simulátorem - ovládání motorů + čtení stavu senzorů.
* Funkční měření odchylky robotu od trajektorie.

## Výstupy
* Spolehlivě fungující implementace sledování trajektorie.

## Odevzdání výsledku řešení cvičení (do pondělí 12. dubna 2021)
* Do 'odevzdávací' složky v repozitáři Vašeho projektu uložte:
  - 3 grafy časového vývoje regulační odchylky + akčních zásahů při:
    - jízdě po přímé trajektorii,
    - jízdě po kruhovém oblouku o poloměru 20 cm,
    - jízdě po kruhévém oblouku o poloměru 10 cm.
  - textový soubor formátovaný v jazyce Markdown, který krátce popisuje vaše zvolené a implementované řešení včetně postupu nalezení konstant regulátoru.

## Úkol č. 1: Implementace ovládání diferenciálně řízeného podvozku



## Úkol č. 2: Ověření správnosti implementace z přechozího bodu



## Úkol č. 3: Návrh a implementace regulátoru pro sledování trajektorie

Navrhněte a implementujte regulační strukturu tak, aby robot spolehlivě sledoval trajektorii (nikdy z ní nesjel) a současně byla rychlost sledování trajektorie maximální možná.

Pro ty, kteří zatím netuší, jak postupovat, zde je malý návod, co vyzkoušet:
* Lineární rychlost pohybu robotu nastavte na pevnou hodnotu (nižší jednotky cm/s, např. 2 cm/s). Pro regulaci úhlové rychlosti pohybu robotu použijte P regulátor. Zesílení P složky postupně zvyšujte od nulové hodnoty.
* Přidejte sumační a diferenční složku do regulátoru. Nastavte konstanty PSD regulátoru.
* Implementujte ovládání/regulaci lineární složky pohybu robotu (tj. tento akční zásah již nebude konstantní)

