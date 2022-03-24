# Rozhraní simulátoru / robotu

Pro komunikaci se simulátorem i robotem je používán protokol NMEA-0183.

Komunikace pomocí NMEA zpráv je vedena protokolem UDP na jediném portu formou dotaz-odpověď. Simulátor bude v textu značen 
jako "server" a uživatelský program který mu odesílá dotazy jako "klient".

Některé zprávy neobsahují žádnou odpověď serveru. Reálný robot implementuje více příkazů než simulátor z důvodu lepšího   
ladění na hardware. Tyto zprávy jsou simulátorem ignorovány.

| SIM  | REAL| dotaz                  | odpověď                |
|------|-----|----------------------- |----------------------- |
| ANO  | ANO | `SPEED,<left>,<right>` | `OK`                   |
| ANO  | ANO | `ODO`                  | `ODO,<left>,<right>`   |
| ANO  | ANO | `SENSOR,<id>`          | `SENSOR,<id>,<hodnota>`|
| ANO  | ANO | `PING,[<data>]`        | `PONG,[<data>]`        |
| ANO  | ANO | `RESET`                | `RESET,DONE`           |
| 1.7+ | ANO | `KILL`                 |                        |
| 1.7+ | ANO | `HELP`                 | `HELP,<cmd1>,...`      |
|      | ANO | `BEEP,[<ms>]`          |                        |
|      | ANO | `LED,[<ms>]`           |                        |
|      | ANO | `BTN`                  | `BTN,<left>,<right>`   |
|      | ANO | `GPIO.GET,[<mask>]`    | `GPIO.GET,<mask>`      |
|      | ANO | `GPIO.SET,<mask>,<val>`|                        |
|      | ANO | `GPIO.DIR,<mask>,<dir>`|                        |

# Zprávy ovládání běhu a ladicí

## PING

Server odešle odpověď PONG obsahující stejná textová data která byla ve zprávě PING. Slouží k ladění NMEA komunikace, a k 
ověření Round-Trip-Time (RTT) serveru (časového zpoždění mezi serverem a klientem). Výpočet RTT si musí klient obhospodařit
sám.

## RESET

Po přijetí této zprávy se server nastaví na výchozí konfiguraci. 

Simulátor:
 * pozice

Robot:
 * konfigurace ADC
 * konfigurace GPIO

Po ukončení server odesílá zprávu s parametrem `DONE` oznamující úspěch

## KILL

Po přijetí této zprávy se server řízeně ukončí a nadále nepřijímá další příkazy.

## HELP

Po přijetí této zprávy server odpoví seznamem všech podporovaných příkazů, oddělených čárkou

# Zprávy ovládání robotu

## SPEED

| argument | formát | proměnná | jednotka    | 
|----------|--------|----------|-------------|
|        1 | float  | left     | ukrok / sec |
|        2 | float  | right    | ukrok / sec |

Server nastaví rychlost levého i pravého kola v mikrokrocích za sekundu na požadovanou hodnotu. 

Tato hodnota je držena po dobu 1sec a poté je motor odpojen od napájení (bezpečnost). Je třeba zprávu opakovat pro kontinuální chod.

## ODO

Požaduje od serveru zjištění ujeté vzdálenosti obou motorů v mikrokrocích od posledního zavolání tohoto příkazu, Odpoví zprávou `ODO`   
s následujícími hodnotami:

| argument | formát| proměnná| jednotka   | 
|----------|-------|---------|------------|
|        1 | int64 | left    | ukrok      |
|        2 | int64 | right   | ukrok      |

Poznámka: Reálný robot počítá pouze s 32bitovým intem, kdežto simulátor se 64bitovým !

## SENSOR

Požaduje od serveru změření hodnoty A/D převodníkem s daným indexem. 

| argument | formát| proměnná| jednotka        | 
|----------|-------|---------|-----------------|
|        1 | int   | index   | 0-7 bez rozměru |

Server odpoví zprávou `SENSOR` s následujícími hodnotami:

| argument | formát| proměnná| jednotka       | 
|----------|-------|---------|----------------|
|        1 | int   | index   | 0-7 bez rozměru|
|        2 | int   | value   | bez rozměru    |

# Zprávy interakce s uživatelem

## BEEP

Robot pípne pípákem s dobou písknutí stanovenou v milisekundách.

| argument | formát| proměnná| jednotka   | 
|----------|-------|---------|------------|
|        1 | int   | ms      | ms         |

Argument není nutné použit, pípák pak pískne s dobou 100ms.

V okamžiku překrývajících se písknutí se prodlužuje doba písknutí.

## LED

Robot blikne nakonfigurovanou LED dobou svitu stanovenou v milisekundách.

| argument | formát| proměnná| jednotka   | 
|----------|-------|---------|------------|
|        1 | int   | ms      | ms         |

Argument není nutné použit, led pak blikne s dobou 100ms.

V okamžiku překrývajících se bliknutí se prodlužuje doba svitu o stanovený čas.

## BTN

Robot navrátí stav stisknutí obou tlačítek ve zprávě: 

| argument | formát| proměnná| jednotka    | 
|----------|-------|---------|-------------|
|        1 | bool  | left    | 1=stisknuto |
|        2 | bool  | right   | 1=stisknuto |

## GPIO.GET

## GPIO.SET

## GPIO.DIR