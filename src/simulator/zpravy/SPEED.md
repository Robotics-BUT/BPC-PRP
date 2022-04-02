# SPEED

## Požadavek:

| argument | formát | proměnná | jednotka    | 
|----------|--------|----------|-------------|
| 0        | text   | "SPEED"  | -           |
| 1        | float  | left     | ukrok / sec |
| 2        | float  | right    | ukrok / sec |

Server nastaví rychlost levého i pravého kola v mikrokrocích za sekundu na požadovanou hodnotu.

Tato hodnota je držena po dobu 1sec a poté je motor odpojen od napájení (bezpečnost). Je třeba zprávu opakovat pro kontinuální chod.

## Odpověď:

Simulátor odpoví zprávou "OK", Reálný robot neodpoví nic.


| argument | formát | proměnná | jednotka    | 
|----------|--------|----------|-------------|
| 0        | text   | "OK"     | -           |
