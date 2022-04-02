# LED

Robot blikne nakonfigurovanou LED dobou svitu stanovenou v milisekundách.

V okamžiku překrývajících se bliknutí se prodlužuje doba svitu o stanovený čas.

Simulátor neimplementuje.

## Požadavek:

| argument | formát | proměnná | jednotka | 
|----------|--------|----------|----------|
| 0        | text   | "LED"    | -        |
| 1        | int    | ms       | ms       |

Argument není nutné použit, led pak blikne s dobou 100ms.

## Odpověď:

žádná