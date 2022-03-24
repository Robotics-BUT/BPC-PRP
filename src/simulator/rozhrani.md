# Rozhraní simulátoru / robotu

Pro komunikaci se simulátorem i robotem je používán protokol NMEA-0183.

Komunikace pomocí NMEA zpráv je vedena protokolem UDP na jediném portu formou dotaz-odpověď. Simulátor bude v textu značen 
jako "server" a uživatelský program který mu odesílá dotazy jako "klient".

Pro spojení je využíváno následujících adres:

| CÍL                  | adresa                        |
|----------------------|-------------------------------|
| simulátor (v linuxu) | `127.0.0.1/8080`              |
| simulátor (přes síť) | `<ip adresa simulátoru>/8080` |
| robot (přes síť)     | `<ip adresa robotu>/6666`     |

Aktuální adresu robotu je nutné zjistit na routeru z DHCP, nebo lze využít příkazů pro zjištění okolních počítačů 
(na linuxu `fping`, na windows existuje spousta okenních nástrojů)
Adresu simulátoru lze zjistit linuxovým příkazem `ip a` z terminálu virtuálního stroje kde běží simulátor.

Některé zprávy neobsahují žádnou odpověď serveru. Reálný robot implementuje více příkazů než simulátor z důvodu lepšího   
ladění na hardware. Tyto zprávy jsou simulátorem ignorovány.

| SIM  | REAL | dotaz                    | odpověď                 |
|------|------|--------------------------|-------------------------|
| ANO  | ANO  | `SPEED,<left>,<right>`   | `OK`                    |
| ANO  | ANO  | `ODO`                    | `ODO,<left>,<right>`    |
| ANO  | ANO  | `SENSOR,<id>`            | `SENSOR,<id>,<hodnota>` |
| ANO  | ANO  | `PING,[<data>]`          | `PONG,[<data>]`         |
| ANO  | ANO  | `RESET`                  | `RESET,DONE`            |
| 1.7+ | ANO  | `KILL`                   |                         |
| 1.7+ | ANO  | `HELP`                   | `HELP,<cmd1>,...`       |
|      | ANO  | `BEEP,[<ms>]`            |                         |
|      | ANO  | `LED,[<ms>]`             |                         |
|      | ANO  | `BTN`                    | `BTN,<left>,<right>`    |
|      | ANO  | `GPIO.GET,[<mask>]`      | `GPIO.GET,<mask>`       |
|      | ANO  | `GPIO.SET,<mask>,<val>`  |                         |
|      | ANO  | `GPIO.DIR,<mask>,<dir>`  |                         |
|      |      | `PWM,<output>,<value>`   |                         |
|      |      | `SERVO,<output>,<value>` |                         |

# Zprávy ovládání běhu a ladicí

## PING

Server odešle odpověď PONG obsahující stejná textová data která byla ve zprávě PING. Slouží k ladění NMEA komunikace, a k 
ověření Round-Trip-Time (RTT) serveru (časového zpoždění mezi serverem a klientem). Výpočet RTT si musí klient obhospodařit
sám.

### Požadavek:

| argument | formát    | proměnná  | jednotka    | 
|----------|-----------|-----------|-------------|
| 0        | text      | "PING"    | -           |
| 1 ... x  | libovolný | argumenty | -           |

Pole argumenty je volitelné a nemusí být ve zprávě obsaženo

### Odpověď:

| argument | formát    | proměnná  | jednotka | 
|----------|-----------|-----------|----------|
| 0        | text      | "PONG"    | -        |
| 1 ... x  | libovolný | argumenty | -        |

pole argumenty obsahuje všechny argumenty které byly obsaženy ve zprávě PING

## RESET

Po přijetí této zprávy se server nastaví na výchozí konfiguraci. 

Simulátor:
 * pozice

Robot:
 * konfigurace ADC
 * konfigurace GPIO

### Požadavek:

| argument | formát | proměnná | jednotka    | 
|----------|--------|----------|-------------|
| 0        | text   | "RESET"  | -           |

### Odpověď:

| argument | formát | proměnná | jednotka    | 
|----------|--------|----------|-------------|
| 0        | text   | "RESET"  | -           |
| 1        | text   | výsledek | -           |

Pakliže byla operace úspěšná, navrací `DONE`, při neúspěchu navrací `ERROR` nebo `FAILED` v poli výsledek

## KILL

Po přijetí této zprávy se server řízeně ukončí a nadále nepřijímá další příkazy.

### Požadavek:

| argument | formát | proměnná | jednotka    | 
|----------|--------|----------|-------------|
| 0        | text   | "KILL"   | -           |

### Odpověď:

žádná

## HELP

Po přijetí této zprávy server odpoví seznamem všech podporovaných příkazů, oddělených čárkou

### Požadavek:

| argument | formát | proměnná | jednotka    | 
|----------|--------|----------|-------------|
| 0        | text   | "HELP"   | -           |

### Odpověď:

| argument | formát | proměnná | jednotka | 
|----------|--------|----------|----------|
| 0        | text   | "HELP"   | -        |
| 1 ... x  | text   | příkaz   | -        |

V odpovědi je seznam podporovaných příkazů serverem

# Zprávy ovládání robotu

## SPEED

### Požadavek:

| argument | formát | proměnná | jednotka    | 
|----------|--------|----------|-------------|
| 0        | text   | "SPEED"  | -           |
| 1        | float  | left     | ukrok / sec |
| 2        | float  | right    | ukrok / sec |

Server nastaví rychlost levého i pravého kola v mikrokrocích za sekundu na požadovanou hodnotu. 

Tato hodnota je držena po dobu 1sec a poté je motor odpojen od napájení (bezpečnost). Je třeba zprávu opakovat pro kontinuální chod.

### Odpověď:

Simulátor odpoví zprávou "OK", Reálný robot neodpoví nic.


| argument | formát | proměnná | jednotka    | 
|----------|--------|----------|-------------|
| 0        | text   | "OK"     | -           |

## ODO

### Požadavek:

| argument | formát | proměnná | jednotka    | 
|----------|--------|----------|-------------|
| 0        | text   | "ODO"    | -           |

### Odpověď:

Požaduje od serveru zjištění ujeté vzdálenosti obou motorů v mikrokrocích od posledního zavolání tohoto příkazu, Odpoví zprávou `ODO`   
s následujícími hodnotami:

| argument | formát | proměnná | jednotka | 
|----------|--------|----------|----------|
| 0        | text   | "ODO"    | -        |
| 1        | int64  | left     | ukrok    |
| 2        | int64  | right    | ukrok    |

Poznámka: Reálný robot počítá pouze s 32bitovým intem, kdežto simulátor se 64bitovým !

## SENSOR

Požaduje od serveru změření hodnoty A/D převodníkem s daným indexem. 

### Požadavek:

| argument | formát | proměnná | jednotka        | 
|----------|--------|----------|-----------------|
| 0        | text   | "SENSOR" | -               |
| 1        | int    | index    | 0-7 bez rozměru |

### Odpověď:

| argument | formát | proměnná | jednotka        | 
|----------|--------|----------|-----------------|
| 0        | text   | "SENSOR" | -               |
| 1        | int    | index    | 0-7 bez rozměru |
| 2        | int    | value    | bez rozměru     |

Index udává pořadové číslo senzoru a value udává 12-bitové číslo změřené A/D převodníkem na daném senzoru.

# Zprávy interakce s uživatelem

## BEEP

Robot pípne pípákem s dobou písknutí stanovenou v milisekundách. 

V okamžiku překrývajících se písknutí se prodlužuje doba písknutí.

Simulátor neimplementuje.

### Požadavek:

| argument | formát | proměnná | jednotka | 
|----------|--------|----------|----------|
| 0        | text   | "BEEP"   | -        |
| 1        | int    | ms       | ms       |

Argument není nutné použit, pípák pak pískne s dobou 100ms.

### Odpověď:

žádná

## LED

Robot blikne nakonfigurovanou LED dobou svitu stanovenou v milisekundách.

V okamžiku překrývajících se bliknutí se prodlužuje doba svitu o stanovený čas.

Simulátor neimplementuje.

### Požadavek:

| argument | formát | proměnná | jednotka | 
|----------|--------|----------|----------|
| 0        | text   | "LED"    | -        |
| 1        | int    | ms       | ms       |

Argument není nutné použit, led pak blikne s dobou 100ms.

### Odpověď:

žádná

## BTN

Robot navrátí stav stisknutí obou tlačítek ve zprávě: 

Simulátor neimplementuje.

### Požadavek:

| argument | formát | proměnná | jednotka | 
|----------|--------|----------|----------|
| 0        | text   | "BTN"    | -        |

### Odpověď:

| argument | formát | proměnná | jednotka    | 
|----------|--------|----------|-------------|
| 0        | text   | "BTN"    | -           |
| 1        | bool   | left     | 1=stisknuto |
| 2        | bool   | right    | 1=stisknuto |

## GPIO.GET

Příkaz přečte aktuální stav na digitálním portu desky MAINBOARD. Pole `mask` v odpovědi obsahuje aktuální hodnotou čtenou 
na portu GPIO (všech 16 bitů). Pakliže je v požadavku specifikována maska, je tato na výsledek aplikována.

mask = mask & GPIO.read()

Simulátor neimplementuje.

### Požadavek:

| argument | formát | proměnná   | jednotka | 
|----------|--------|------------|----------|
| 0        | text   | "GPIO.GET" | -        |
| 1        | int16  | mask       | -        |

Pole mask není v požadavku povinné.

### Odpověď:

| argument | formát | proměnná   | jednotka | 
|----------|--------|------------|----------|
| 0        | text   | "GPIO.GET" | -        |
| 1        | int16  | mask       | -        |

## GPIO.SET

Simulátor neimplementuje.

### Požadavek:


| argument | formát | proměnná   | jednotka | 
|----------|--------|------------|----------|
| 0        | text   | "GPIO.SET" | -        |
| 1        | int16  | mask       | -        |
| 2        | int16  | val        | -        |

### Odpověď:

žádná

## GPIO.DIR

Simulátor neimplementuje.

### Požadavek:

| argument | formát | proměnná   | jednotka | 
|----------|--------|------------|----------|
| 0        | text   | "GPIO.SET" | -        |
| 1        | int16  | mask       | -        |
| 2        | bool   | dir        | -        |

dir obsahuje 1 pro input, 0 pro output

### Odpověď:

žádná