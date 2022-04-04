# Návrh regulační smyčky
Cvičící: Ing. Tomáš Jílek, Ph.D.

## Cíle
* Implementovat ovládání diferenciálně řízeného podvozku.
* Navrhnout a implementovat spolehlivý regulátor pro sledování trasy.

## Prerekvizity
* Funkční komunikace se simulátorem - ovládání motorů + čtení stavu senzorů.
* Funkční měření odchylky robotu od žádané trajektorie.
* Funkční generátor žádané trajektorie.

## Výstupy
* Spolehlivě fungující implementace sledování trajektorie.

## Odevzdání výsledku řešení cvičení (do následujícího cvičení)
* Do složky `labs/lab-08` v repozitáři Vašeho projektu uložte:
  - 3 grafy časového vývoje regulační odchylky + akčních zásahů při:
    - sledování přímé trasy,
    - sledování trasy reprezentované kruhovým obloukem o poloměru 20 cm,
    - sledování trasy reprezentované kruhovým obloukem o poloměru 5 cm,
  - textový soubor formátovaný v jazyce Markdown, který krátce popisuje zvolené a implementované řešení včetně postupu nalezení konstant regulátoru.

## Úkol č. 1: Implementace ovládání diferenciálně řízeného podvozku

Implementujte inverzní kinematiku pro diferenciálně řízený podvozek. Identifikujte hodnoty konstant, které jsou k výpočtu potřeba.

    motor_left_speed  = (robot_linear_speed + 0.5 * C_TW * robot_angular_speed) * C_L
    motor_right_speed = (robot_linear_speed - 0.5 * C_TW * robot_angular_speed) * C_R

V proměnných `left_motor_speed` a `right_motor_speed` jsou již přímo hodnoty, které je třeba zapsat do emulovaného ovladače krokových motorů. Konstanta `C_TW` je rozteč kol (šířka nápravy) [m]. Konstanty C_L a C_R jsou v jednotkách `[microstep/m]`, konstanty tedy vyjadřují potřebný počet mikrokroků na 1 metr ujeté vzdálenosti daným kolem.

Parametry použitých komponent

                 Wheel: wheel_diameter * pi [m/turn]
         Stepper motor: 200 [step/turn]
          Motor driver: 32 [microstep/step]

### Postup výpočtu

    ROBOT(v [m/s], omega [rad/s]) -> WHEEL(v_left, v_right [m/s]) -> MOTOR(omega_left, omega_right [microstep/s])

## Úkol č. 2: Ověření správnosti implementace řešené v předchozím bodu.

Než přistoupíte k návrhu a nastavování regulátoru, je vhodné ověřit, že robot opravdu vykonává pohyb, který je od něho očekáván. Správnou funkčnost je možné ověřit např. na těchto pohybech:
* jízda po přímce (`v` je nenulové, `omega = 0 deg/s`)
  - např. při `v = 1 cm/s` by měl robot za 10 sekund ujet vzdálenost 10 cm
* otáčení na místě (`v = 0 m/s`, `omega` je nenulová)
  - např. při `omega = 36 deg/s` by se robot měl za 10 sekund otočit kolem své osy o 360 deg, tj. dostat se do výchozí orientace
* jízda po kruhovém oblouku (`v` i `omega` jsou nenulové)
  - např. při `v = 1 cm/s` a `omega = 36 deg/s` by trajektorie robotu měla být kružnice o poloměru cca 1,6 cm (změna ujeté vzdálenosti bude 10 cm, změna orientace bude 360 deg)

## Úkol č. 3: Návrh a implementace regulátoru pro sledování trajektorie

Navrhněte a implementujte regulační strukturu tak, aby robot spolehlivě sledoval trajektorii (nikdy z ní nesjel) a současně byla rychlost sledování trajektorie maximální možná.

Pro ty, kteří zatím netuší, jak postupovat, zde je malý návod, co vyzkoušet:
* Lineární rychlost pohybu robotu nastavte na pevnou hodnotu (nižší jednotky cm/s, např. 2 cm/s). Pro regulaci úhlové rychlosti pohybu robotu použijte P regulátor. Zesílení P složky postupně zvyšujte od nulové hodnoty.
* Přidejte sumační a diferenční složku do regulátoru. Nastavte konstanty PSD regulátoru.
* Implementujte ovládání/regulaci lineární složky pohybu robotu (tj. tento akční zásah již nebude konstantní).
* Vyhodnoťte, které složky (P/S/D) jsou v regulátoru potřeba a jak pozitivně/negativně ovlivňují regulační děj (sledování trasy).
* Zkuste se trochu hlouběji zamyslet, jaký má vliv umístění senzorického systému na průběh regulačního děje, příp. jak ovlivňuje potřebnou strukturu regulačního obvodu a požadavky na regulátor.
* Na základě zjištěných informací se pokuste navrhnout vlastní regulační schéma pro dosažení co nejlepších výsledků sledování trasy.
