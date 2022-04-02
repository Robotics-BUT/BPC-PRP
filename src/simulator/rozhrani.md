# Rozhraní simulátoru / robotu

Pro komunikaci se simulátorem i robotem je používán protokol NMEA-0183.

Komunikace pomocí NMEA zpráv je vedena protokolem UDP na jediném portu formou dotaz-odpověď. Simulátor bude v textu značen 
jako "server" a uživatelský program který mu odesílá dotazy jako "klient".

Pro spojení je využíváno následujících adres:

| CÍL                  | adresa                        |
|----------------------|-------------------------------|
| simulátor (v linuxu) | `127.0.0.1/8080`              |
| simulátor (přes síť) | `<ip adresa simulátoru>/8080` |
| robot (přes síť)     | `<ip adresa robotu>/6666`     |

Aktuální adresu robotu je nutné zjistit na routeru z DHCP, nebo lze využít příkazů pro zjištění okolních počítačů 
(na linuxu `fping`, na windows existuje spousta okenních nástrojů)
Adresu simulátoru lze zjistit linuxovým příkazem `ip a` z terminálu virtuálního stroje kde běží simulátor.

Některé zprávy neobsahují žádnou odpověď serveru. Reálný robot implementuje více příkazů než simulátor z důvodu lepšího   
ladění na hardware. Tyto zprávy jsou simulátorem ignorovány.

| SIM  | REAL | dotaz                    | odpověď                 | dokumentace                                |
|------|------|--------------------------|-------------------------|--------------------------------------------|
| ANO  | ANO  | `SPEED,<left>,<right>`   | `OK`                    | [SPEED](./simulator/zpravy/SPEED.md)       | 
| ANO  | ANO  | `ODO`                    | `ODO,<left>,<right>`    | [ODO](./simulator/zpravy/ODO.md)           | 
| ANO  | ANO  | `SENSOR,<id>`            | `SENSOR,<id>,<hodnota>` | [SENSOR](./simulator/zpravy/SENSOR.md)     |
| ANO  | ANO  | `PING,[<data>]`          | `PONG,[<data>]`         | [PING](./simulator/zpravy/PING.md)         |
| ANO  | ANO  | `RESET`                  | `RESET,DONE`            | [RESET](./simulator/zpravy/RESET.md)       |
| 1.7+ | ANO  | `KILL`                   |                         | [KILL](./simulator/zpravy/KILL.md)         |
| 1.7+ | ANO  | `HELP`                   | `HELP,<cmd1>,...`       | [HELP](./simulator/zpravy/HELP.md)         |
|      | ANO  | `BEEP,[<ms>]`            |                         | [BEEP](./simulator/zpravy/BEEP.md)         |
|      | ANO  | `LED,[<ms>]`             |                         | [LED](./simulator/zpravy/LED.md)           |
|      | ANO  | `BTN`                    | `BTN,<left>,<right>`    | [BTN](./simulator/zpravy/BTN.md)           |
|      | ANO  | `GPIO.GET,[<mask>]`      | `GPIO.GET,<mask>`       | [GPIO.GET](./simulator/zpravy/GPIO.GET.md) |
|      | ANO  | `GPIO.SET,<mask>,<val>`  |                         | [GPIO.SET](./simulator/zpravy/GPIO.SET.md) |
|      | ANO  | `GPIO.DIR,<mask>,<dir>`  |                         | [GPIO.DIR](./simulator/zpravy/GPIO.DIR.md) |
|      |      | `PWM,<output>,<value>`   |                         | [PWM](./simulator/zpravy/PWM.md)           |     
|      |      | `SERVO,<output>,<value>` |                         | [SERVO](./simulator/zpravy/SERVO.md)       |

## Zprávy dle kategorií

 * ***Zprávy ovládání běhu a ladicí***
   * [PING](./simulator/zpravy/PING.md)
   * [RESET](./simulator/zpravy/RESET.md)
   * [KILL](./simulator/zpravy/KILL.md)
   * [HELP](./simulator/zpravy/HELP.md)

 * **Zprávy ovládání robotu**

   * [SPEED](./simulator/zpravy/SPEED.md)
   * [ODO](./simulator/zpravy/ODO.md)
   * [SENSOR](./simulator/zpravy/SENSOR.md)

 * **Zprávy interakce s uživatelem**

   * [BEEP](./simulator/zpravy/BEEP.md)
   * [LED](./simulator/zpravy/LED.md)
   * [BTN](./simulator/zpravy/BTN.md)
   * [GPIO.GET](./simulator/zpravy/GPIO.GET.md)
   * [GPIO.SET](./simulator/zpravy/GPIO.SET.md)
   * [GPIO.DIR](./simulator/zpravy/GPIO.DIR.md)

 * **Zprávy dosud neimplementované**
   * [PWM](./simulator/zpravy/PWM.md)
   * [SERVO](./simulator/zpravy/SERVO.md)