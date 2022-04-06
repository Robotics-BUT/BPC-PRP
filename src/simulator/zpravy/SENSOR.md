# SENSOR

Požaduje od serveru změření hodnoty A/D převodníkem s daným indexem.

## Požadavek:

| argument | formát | proměnná | jednotka        | 
|----------|--------|----------|-----------------|
| 0        | text   | "SENSOR" | -               |
| 1        | int    | index    | 0-7 bez rozměru |

## Odpověď:

| argument | formát | proměnná | jednotka        | 
|----------|--------|----------|-----------------|
| 0        | text   | "SENSOR" | -               |
| 1        | int    | index    | 0-7 bez rozměru |
| 2        | int    | value    | bez rozměru     |

Index udává pořadové číslo senzoru a value udává 12-bitové číslo změřené A/D převodníkem na daném senzoru.
