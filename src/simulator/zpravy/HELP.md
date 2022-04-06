# HELP

Po přijetí této zprávy server odpoví seznamem všech podporovaných příkazů, oddělených čárkou

## Požadavek:

| argument | formát | proměnná | jednotka    | 
|----------|--------|----------|-------------|
| 0        | text   | "HELP"   | -           |

## Odpověď:

| argument | formát | proměnná | jednotka | 
|----------|--------|----------|----------|
| 0        | text   | "HELP"   | -        |
| 1 ... x  | text   | příkaz   | -        |

V odpovědi je seznam podporovaných příkazů serverem