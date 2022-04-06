# GPIO.GET

Příkaz přečte aktuální stav na digitálním portu desky MAINBOARD. Pole `mask` v odpovědi obsahuje aktuální hodnotou čtenou
na portu GPIO (všech 16 bitů). Pakliže je v požadavku specifikována maska, je tato na výsledek aplikována.

mask = mask & GPIO.read()

Simulátor neimplementuje.

## Požadavek:

| argument | formát | proměnná   | jednotka | 
|----------|--------|------------|----------|
| 0        | text   | "GPIO.GET" | -        |
| 1        | int16  | mask       | -        |

Pole mask není v požadavku povinné.

## Odpověď:

| argument | formát | proměnná   | jednotka | 
|----------|--------|------------|----------|
| 0        | text   | "GPIO.GET" | -        |
| 1        | int16  | mask       | -        |
