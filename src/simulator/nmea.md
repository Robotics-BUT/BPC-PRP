# nmea - utilita

slouží pro ověření funkčnosti zpráv (na vyzkoušení že vše funguje jak má. Po spuštění v interaktivním režimu je emulována konzole, uživatel může ručně zadat   
NMEA příkaz (bez počátečního $ a části s CRC s hvězdičkou. Program spočítá správné CRC a odešle jej na daný UDP port. (vypíše celou zprávu včetně správného CRC)

Příkaz slouží zejména pro ověření že správně počítáte CRC a že vám běží server (oddělení chyb špatně spuštěného simulátoru od vaší chyby v programu).

Příkaz je doporučeno ručně nainstalovat (zkopírovat) do `/usr/bin` dle platformy. Na platformě raspberry pi si soubor `nmea_armhf` před zkopírováním do `/usr/bin` přejmenujte na `nmea`. identicky pro `x86` a `arm64` na osobním počítači

## Spouštěcí argumenty

```shell
nmea [--help] [--version] [--local=<udp port>] [--remote=]<ip addr>/<udp port>
```

Jednotlivé argumenty příkazové řádky:

 `--help` : Vypiš stručnou nápovědu a ukonči se

 `--version` : Vypiš aktuální verzi nástroje a ukonči se.

 `--local=<udp port>` : nastaví lokální číslo UDP portu na kterém má naslouchat (default: 6667). `<udp port>` musí být celé číslo v rozsahu 1024 - 65535. UDP port nesmí být na lokálním počítači obsazený. Pro použití portu pod `1024` je potřeba program spustit jako `sudo`.

 `--remote=<ip addr>/<udp port>` nebo `<ip addr>/<udp port>` : **povinný parametr**. Nastaví adresu a port vzdáleného počítače. IP adresa musí být specifikovaná číselně v tečkové konvenci (např 127.0.0.1), DNS překlad není podporován. Port musí být celé číslo v rozsahu 1024-65535. Oddělovač adresy a portu je lomítko z důvodu budoucí kompatibility s IPv6 (adresa ipv6 má dvojtečku jako oddělovač částí takže nejde použít pro oddělení části portu od adresy). Adresy IPv6 prozatím nejsou podporovány, ale je na ně příprava. Pro použití portu pod `1024` je potřeba program spustit jako `sudo`

## Příklad použití

V následujícím příkladu se komunikuje s reálným robotem na adrese 10.16.0.247 na portu 6666. uživatel zadal na klávesnici `BEEP` a stiskl enter, `PING` a stiskl enter. Je vidět odeslaná zpráva i přijatá zpráva od serveru.

```shell
bufran@pcburian:~$ nmea 10.16.0.247/6666
NMEA sender v 0.2 (c) 2022 Frantisek Burian
 communicating with remote 10.16.0.247/6666
 using local port 6667
 Press Ctrl-C to exit
> BEEP                                                   
[ 10.16.0.247/6666 TX] $BEEP*12
> PING                                                   
[ 10.16.0.247/6666 TX] $PING*10
[ 10.16.0.247/6666 RX] $PONG*16                          
>                                                        
```

