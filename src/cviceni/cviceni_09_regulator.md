# Návrh regulátoru

Cvičící: Ing. Tomáš Jílek, Ph.D.

## Cíle
* Implementovat ovládání a měření pozice+orientace pro diferenciálně řízený podvozek.
* Navrhnout a implementovat spolehlivý regulátor pro sledování trajektorie.

## Prerekvizity
* Funkční komunikace se simulátorem - ovládání motorů + čtení stavu senzorů.
* Funkční měření odchylky robotu od trajektorie.

## Výstupy
* Spolehlivě fungující implementace sledování trajektorie.

## Odevzdání výsledku řešení cvičení (do pondělí 12. dubna 2021)
* Do 'odevzdávací' složky v repozitáři Vašeho projektu uložte:
  - 3 grafy časového vývoje regulační odchylky + akčních zásahů při:
    - jízdě po přímé trajektorii,
    - jízdě po kruhovém oblouku o poloměru 20 cm,
    - jízdě po kruhévém oblouku o poloměru 10 cm.
  - textový soubor formátovaný v jazyce Markdown, který krátce popisuje vaše zvolené a implementované řešení včetně postupu nalezení konstant regulátoru.

## Úkol č. 1: Implementace ovládání diferenciálně řízeného podvozku

Implementujte inverzní kinematiku pro diferenciálně řízený podvozek. Identifikujte hodnoty konstant, které jsou k výpočtu potřeba.

    motor_left_speed  = (robot_linear_speed + 0.5 * C_TW * robot_angular_speed) * C_L
    motor_right_speed = (robot_linear_speed - 0.5 * C_TW * robot_angular_speed) * C_R

V proměnných `left_motor_speed` a `right_motor_speed` jsou již přímo hodnoty, které je třeba zapsat do příslušných registrů modulu KM2. Fyzikální rozměr těchto veličin je [microstep/(10*0xFFF microsecond)], to znamená, že do registrů se zapisuje počet mikrokroků za 40,96 milisekund) - viz dokumentace k modulu KM2. Konstanta `C_TW` je rozteč kol (šířka nápravy) [m]. Konstanty C_L a C_R jsou v jednotkách `[100000/4096 microstep/m]`, konstanty tedy vyjadřují počet 24.4140625mikrokroku na 1 metr.

Parametry použitých komponent

                 Wheel: wheel_diameter * pi [m/turn]
         Stepper motor: 200 [step/turn]
          Motor driver: 32 [microstep/step]
    KM2 speed register: [microstep/(10*0xFFF microsecond)] -> 4096 [microstep/(10*0xFFF microsecond)] ~ 10^5 [microstep/s]

### Postup výpočtu

    ROBOT(v [m/s], omega [rad/s]) -> WHEEL(v_left, v_right [m/s]) -> MOTOR(omega_left, omega_right [microstep/s]) -> KM2_SPEED_REGISTERS(reg_left, reg_right [microstep/(40.96*milisecond)])

## Úkol č. 2: Ověření správnosti implementace z přechozího bodu



## Úkol č. 3: Návrh a implementace regulátoru pro sledování trajektorie

Navrhněte a implementujte regulační strukturu tak, aby robot spolehlivě sledoval trajektorii (nikdy z ní nesjel) a současně byla rychlost sledování trajektorie maximální možná.

Pro ty, kteří zatím netuší, jak postupovat, zde je malý návod, co vyzkoušet:
* Lineární rychlost pohybu robotu nastavte na pevnou hodnotu (nižší jednotky cm/s, např. 2 cm/s). Pro regulaci úhlové rychlosti pohybu robotu použijte P regulátor. Zesílení P složky postupně zvyšujte od nulové hodnoty.
* Přidejte sumační a diferenční složku do regulátoru. Nastavte konstanty PSD regulátoru.
* Implementujte ovládání/regulaci lineární složky pohybu robotu (tj. tento akční zásah již nebude konstantní)

