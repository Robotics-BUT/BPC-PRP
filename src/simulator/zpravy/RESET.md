# RESET

Po přijetí této zprávy se server nastaví na výchozí konfiguraci.

Simulátor:
* pozice

Robot:
* konfigurace ADC
* konfigurace GPIO

## Požadavek:

| argument | formát | proměnná | jednotka    | 
|----------|--------|----------|-------------|
| 0        | text   | "RESET"  | -           |

## Odpověď:

| argument | formát | proměnná | jednotka    | 
|----------|--------|----------|-------------|
| 0        | text   | "RESET"  | -           |
| 1        | text   | výsledek | -           |

Pakliže byla operace úspěšná, navrací `DONE`, při neúspěchu navrací `ERROR` nebo `FAILED` v poli výsledek