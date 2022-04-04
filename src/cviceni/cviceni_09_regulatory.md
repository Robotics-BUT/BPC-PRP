# Návrh regulátorů a regulační smyčky pro spolehlivé sledování čáry při vyšších rychlostech jízdy
Cvičící: Ing. Tomáš Jílek, Ph.D.

## Cíle
* Implementovat spolehlivou navigaci robotu podél žádané trajektorie při současné maximalizaci průměrné rychlosti jízdy.

## Prerekvizity
* Funkční komunikace se simulátorem - ovládání motorů + čtení stavu senzorů.
* Funkční generátor žádané trajektorie.
* Funkční kalibrovaný senzorický systém měřící jeho vzdálenost od referenční oblasti na vodicí čáře nebo jeho orientaci k vodicí čáře.
* Funkční kalibrované ovládání pohybu robotu.
* Funkční základní regulační smyčka.

## Výstupy
* Správně implementovaný a fungující PSD regulátor pro regulaci úhlové rychlosti robotu.
* Ovládání rychlosti jízdy robotu maximalizující jeho průměrnou rychlost jízdy na celé trase.
* (NEPOVINNÉ) Parametrizovatelný senzorický systém pro měření úhlové a/nebo vzdálenostní odchylky robotu od vodicí čáry.

## Odevzdání výsledku řešení cvičení (do následujícího cvičení)
* Do složky `labs/lab-09` v repozitáři Vašeho projektu uložte:
  - schéma výsledné regulační struktury,
  - 3 grafy časového vývoje regulační odchylky + VŠECH akčních zásahů do soustavy (výsledná reg. struktura) při:
    - sledování přímé trasy,
    - sledování trasy reprezentované kruhovým obloukem o poloměru 20 cm,
    - sledování trasy reprezentované kruhovým obloukem o poloměru 5 cm,
  - textový soubor formátovaný v jazyce Markdown, který krátce popisuje navržené a implementované řešení včetně postupu nalezení konstant regulátorů.

## Úkol č. 1: Implementace PSD regulátoru

Proveďte implementaci PSD regulátoru a všech souvisejících potřebných interních algoritmů regulátoru, které jsou nutné pro použití v reálné regulační smyčce (anti-windup, atd.).

## Úkol č. 2: Nastavení konstant PSD regulátoru

Nastavte konstanty PSD regulátoru úhlové rychlosti robotu tak, aby zajistil sledování žádané trajektorie s vyhovující přesností při maximální možné průměrné rychlosti jízdy. Porovnejte mezi sebou různá nastavení regulátoru získaná pomocí návrhových metod, které byly probrány v kurzech Řízení a regulace.

## Úkol č. 3: Návrh a úprava regulační struktury

Identifikujte nedostatky stávající regulační struktury a navrhněte a implementujte její úpravu tak, aby došlo k potlačení nebo úplné eliminaci nežádoucích vlastností stávající regulační struktury. Do návrhu zakomponujte např. maximalizaci průměrné rychlosti jízdy na celé trajektorii.

## Úkol č. 4: Implementace měření pozice a orientace robotu

Implementujte výpočet odometrie ze zpráv ODO:
* ujetá vzdálenost levým a pravým kolem za poslední periodu vzorkování,
* ujetá vzdálenost robotem za poslední periodu vzorkování,
* změna orientace robotu za poslední periodu vzorkování,
* změna pozičních souřadnic robotu za poslední periodu vzorkování,
* aktuální pozici a orientaci robotu.

## Úkol č. 5: (NEPOVINNÉ) Implementace parametrizovatelného senzorického systému

## Úkol č. 6: (NEPOVINNÉ) Přenos a parametrizace řešení na reálný HW (platforma KAMbot)
