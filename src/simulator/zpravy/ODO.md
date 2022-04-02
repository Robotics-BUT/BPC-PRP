# ODO

## Požadavek:

| argument | formát | proměnná | jednotka    | 
|----------|--------|----------|-------------|
| 0        | text   | "ODO"    | -           |

## Odpověď:

Požaduje od serveru zjištění ujeté vzdálenosti obou motorů v mikrokrocích od posledního zavolání tohoto příkazu, Odpoví zprávou `ODO`   
s následujícími hodnotami:

| argument | formát | proměnná | jednotka | 
|----------|--------|----------|----------|
| 0        | text   | "ODO"    | -        |
| 1        | int64  | left     | ukrok    |
| 2        | int64  | right    | ukrok    |

Poznámka: Reálný robot počítá pouze s 32bitovým intem, kdežto simulátor se 64bitovým !