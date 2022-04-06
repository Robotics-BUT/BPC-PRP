# Doporučená organizace projektu

Rozdělení do tříd (abecedně):

  * `Comm` - Obsluha kompletní komunikace
  * `Configuration` - Konfigurace načtená z yaml souboru
  * `Controller` - Hlavní logika ovládání programu
  * `Drive` - Obsluha a řízení podvozku
  * `Nmea` - Implementace převodu NMEA zpráv
  * `Sensor` - Obsluha měření a identifikace čáry
  * `main.cpp` - Supersmyčka

Doporučen způsob práce kdy za funkcionalitu jedné třídy zodpovídá jedna osoba 
(ideálně se jménem napsaným v komentáři na začátku souboru). Nikdo jiný 
než vybraná osoba nesmí daný soubor commitovat.

Main je kolizní, můžou do něj všichni, ideálně v malých jednořádkových commitech. 
Main pokud možno modifikovat jen když jste všichni spolu a definujete API 
(rozdáváte práci)

## main.cpp

Řeší:

 * obsluha argumentů příkazové řádky
 * instanciace ostatních tříd
 * hlavní supersmyčka

Závisí na:

 * `Comm` - instanciace a obsluha v supersmy4ce
 * `Drive` - instanciace a obsluha
 * `Sensor` - instanciace a obsluha
 * `Configuration` - instanciace, načtení
 * `Controller` - instanciace a volání v supersmyčce

POZOR KOLIZNÍ SOUBOR (commitovat vždy zvlášť, modifikovat pouze velmi malé změny kvůli častým konfliktům)

## Nmea

Řeší:

 * převod z frame na zprávu
 * převod ze zprávy na frame

Závisí na:

 * nic

Realizace:

  * Implementace 4. týden
  * Spolehlivost 6. týden

## Comm

Řeší:

 * obsluha příjmu a odesílání UDP zpráv
 * Mediátor zpráv do ostatních objektů

Závisí na

 * `Nmea` - převod protokolu
 * `Configuration` VOLITELNE - port

Realizace:

 * Implementace 6. týden
 * Spolehlivost 8. týden

## Drive

Řeší:

 * Obsluha komunikace s podvozkem
 * Výpočty nad podvozkem
 * Odometrie

Závisí na

 * `Nmea` - deklarace zpráv
 * `Configuration` VOLITELNE - poloměr kola, rozteč

Realizace:

  * Implementace 6. týden
  * Spolehlivost 8. týden

## Sensor

Řeší:

 * Obsluha komunikace se senzory
 * Parametrizace /detekce pozice čáry

Závisí na

 * `Nmea` - deklarace zpráv
 * `Configuration` VOLITELNE - pozice senzoru

Realizace:

 * Implementace 7. týden
 * Spolehlivost 8. týden

## Configuration

 * Načítání parametrů z yaml souboru
 * Volitelná třída, lze implementovat za pomocí konstant v kódu
 * výhodné použití `yaml-cpp` knihovny

Závisí na

 nic

Realizace:

 * Implementace 8. týden
 * Spolehlivost 10. týden

## Controller

Řeší:

 * Logika jízdy robotu
 * Stavový automat pro:
   * inicializace / kalibrace senzorů
   * jízda po čáře (regulátor)
   * jízda po přerušené čáře
   * jízda po křižovatce

Závisí na

 * `Drive` - zápis rychlostí robotu
 * `Sensor` - čtení čáry
 * `Configuration` VOLITELNE - délka přerušení

Realizace:

 * Implementace 8. týden
 * Spolehlivost 10. týden