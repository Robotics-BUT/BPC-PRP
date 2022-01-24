# Rozhraní simulátoru

Pro komunikaci se simulátorem je používán protokol NMEA. Reálný robot implementuje   
více příkazů než simulátor z důvodu lepšího ladění na hardware

| SIM | REAL | dotaz                   | odpověď                |
|-----|------|------------------------ |----------------------- |
| ANO | ANO  | `SPEED,<left>,<right>`  | -                      |
| ANO | ANO  | `ODO`                   | `ODO,<left>,<right>`   |
| ANO | ANO  | `SENSOR,<id>`           | `SENSOR,<id>,<hodnota>`|
| ANO | ANO  | `PING,[<data>]`         | `PONG,[<data>]`        |
| ANO | ANO  | `RESET`                 | `RESET,DONE`           |
|     | ANO  | `KILL`                  |                        |
|     | ANO  | `HELP`                  | `HELP,<cmd1>,...`      |
|     | ANO  | `BEEP,[<ms>]`           |                        |
|     | ANO  | `LED,[<ms>]`            |                        |
|     | ANO  | `BTN`                   | `BTN,<left>,<right>`   |
|     | ANO  | `GPIO.GET,[<mask>]`     | `GPIO.GET,<mask>`      |
|     | ANO  | `GPIO.SET,<mask>,<val>` |                        |
|     | ANO  | `GPIO.DIR,<mask>,<dir>` |                        |

# Zprávy ovládání běhu a ladicí

## PING

## RESET

## KILL

## HELP

# Zprávy ovládání robotu

## SPEED

nastaví rychlost levého i pravého kola v mikrokrocích za sekundu na požadovanou hodnotu. Tato hodnota je držena po dobu 1sec a poté je motor odpojen od napájení (bezpečnost). Je třeba zprávu opakovat pro kontinuální chod.

left i right float

## ODO

Požaduje od simulátoru zjištění ujeté vzdálenosti obou motorů v mikrokrocích od posledního zavolání tohoto příkazu, Odpoví zprávou `ODO`

left i right int64

## SENSOR

# Zprávy interakce s uživatelem

## BEEP

## BTN

## LED