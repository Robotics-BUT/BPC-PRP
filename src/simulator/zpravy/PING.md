# PING

Server odešle odpověď `PONG` obsahující stejn0 argumenty, jaké byly zadány ve zprávě `PING`. Slouží k ladění NMEA komunikace, a k
ověření Round-Trip-Time (RTT) serveru (časového zpoždění mezi serverem a klientem). Výpočet RTT si musí klient (studenti) obhospodařit
sám.

## Požadavek:

| argument | formát    | proměnná  | jednotka    | 
|----------|-----------|-----------|-------------|
| 0        | text      | "PING"    | -           |
| 1 ... x  | libovolný | argumenty | -           |

Pole argumenty je volitelné a nemusí být ve zprávě obsaženo

## Odpověď:

| argument | formát    | proměnná  | jednotka | 
|----------|-----------|-----------|----------|
| 0        | text      | "PONG"    | -        |
| 1 ... x  | libovolný | argumenty | -        |

pole argumenty obsahuje všechny argumenty které byly obsaženy ve zprávě PING