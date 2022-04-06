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

| SIM  | REAL | dotaz                    | odpověď                 | dokumentace                        |
|------|------|--------------------------|-------------------------|------------------------------------|
| ANO  | ANO  | `SPEED,<left>,<right>`   | `OK`                    | [SPEED](./zpravy/SPEED.md)         | 
| ANO  | ANO  | `ODO`                    | `ODO,<left>,<right>`    | [ODO](./zpravy/ODO.md)             | 
| ANO  | ANO  | `SENSOR,<id>`            | `SENSOR,<id>,<hodnota>` | [SENSOR](./zpravy/SENSOR.md)       |
| ANO  | ANO  | `PING,[<data>]`          | `PONG,[<data>]`         | [PING](./zpravy/PING.md)           |
| ANO  | ANO  | `RESET`                  | `RESET,DONE`            | [RESET](./zpravy/RESET.md)         |
| 1.7+ | ANO  | `KILL`                   |                         | [KILL](./zpravy/KILL.md)           |
| 1.7+ | ANO  | `HELP`                   | `HELP,<cmd1>,...`       | [HELP](./zpravy/HELP.md)           |
|      | ANO  | `BEEP,[<ms>]`            |                         | [BEEP](./zpravy/BEEP.md)           |
|      | ANO  | `LED,[<ms>]`             |                         | [LED](./zpravy/LED.md)             |
|      | ANO  | `BTN`                    | `BTN,<left>,<right>`    | [BTN](./zpravy/BTN.md)             |
|      | ANO  | `GPIO.GET,[<mask>]`      | `GPIO.GET,<mask>`       | [GPIO.GET](./zpravy/GPIO.GET.md)   |
|      | ANO  | `GPIO.SET,<mask>,<val>`  |                         | [GPIO.SET](./zpravy/GPIO.SET.md)   |
|      | ANO  | `GPIO.DIR,<mask>,<dir>`  |                         | [GPIO.DIR](./zpravy/GPIO.DIR.md)   |
|      |      | `PWM,<output>,<value>`   |                         | [PWM](./zpravy/PWM.md)             |     
|      |      | `SERVO,<output>,<value>` |                         | [SERVO](./zpravy/SERVO.md)         |

## Zprávy dle kategorií

 * ***Zprávy ovládání běhu a ladicí***
   * [PING](./zpravy/PING.md)
   * [RESET](./zpravy/RESET.md)
   * [KILL](./zpravy/KILL.md)
   * [HELP](./zpravy/HELP.md)

 * **Zprávy ovládání robotu**

   * [SPEED](./zpravy/SPEED.md)
   * [ODO](./zpravy/ODO.md)
   * [SENSOR](./zpravy/SENSOR.md)

 * **Zprávy interakce s uživatelem**

   * [BEEP](./zpravy/BEEP.md)
   * [LED](./zpravy/LED.md)
   * [BTN](./zpravy/BTN.md)
   * [GPIO.GET](./zpravy/GPIO.GET.md)
   * [GPIO.SET](./zpravy/GPIO.SET.md)
   * [GPIO.DIR](./zpravy/GPIO.DIR.md)

 * **Zprávy dosud neimplementované**
   * [PWM](./zpravy/PWM.md)
   * [SERVO](./zpravy/SERVO.md)