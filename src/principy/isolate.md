# Oddělení modulů - Zrychlení kompilace

Při návrhu objektového API se setkáváme s nutností oddělení jednotlivých modulů tak, aby **navzájem spolupracovaly** 
(jeden modul využívá druhý), ale současně **změna v impleentaci** jednoho modulu nevyvolala **nutnost rekompilace** 
modulu druhého. Typickým příkladem je vazba **nadřazený-podřazený**, ale lze takto vytvořit i **cyklickou závislost**

Velmi špatným řešením v aplikacích bývá vytvoření **globální proměnné**.

Řešení lze docílit měkkou vazbou přes **ukazatel na nedefinovaný objekt** (tzv forward).

**Příklad:** Máme objekt `Controller`, který ve svých metodách potřebuje používat funkce z objektů `Drive` a `Sensor`.
Objekty `Drive` a `Sensor` jsou singletony (existují v aplikaci v právě jedné instanci, vytvořené nejspíše jako
lokální proměnné ve funkci `main`.

## Forward

Aby mohly jednotlivé metody `Controller`u pracovat s `Drive`, musí mít `Controller` v sobě uložený odkaz na instanci
objektu `Drive`. Nejspíše takto:

```c++
#include "Drive.h"

class Controller {
public:
  Drive *drv;
}
```

Pokud však s třídou `Drive` pracuje pouze implementace `Controlleru`, kompilátor potřebuje pouze vědět, že
`Drive` je třída (a nic víc!) a není tak potřeba vkládat celý soubor s definicí `API` třídy `Drive`. Lze toto
zjednodušit na:

```c++
class Drive;    // pouze forward

class Controller {
public:
  Drive *drv;
}
```

a soubor s deklarací Drive vkládáme až do cpp souboru

```c++
#include "Controller.h"
#include "Drive.h"

Controller:: ......
```

Toto lze použít při splnění podmínek:
 * Všechny přístupy ke třídě `Drive` jsou přes ukazatel, nikoliv přímo 
   * instance objektu potřebuje znát velikost tedy plnou deklaraci !
 * k obsahu `drv` se nepřistupuje v headeru ale v kódu cpp.
   * Jakékoliv inline funkce používající obsah drv potřebují znát plnou deklaraci !
 

## Neměnný ukazatel

Dále `drive` by měl být konstantní ukazatel na nekonstantní instanci třídy `Drive`. Ukazatel nechceme nikdy měnit, ale
vlastní objekt měnit můžeme. S tím kam umístíme `const` bývá začátečnický problém, pro shrnutí: 

```c++
Drive *drv;               // ukazatel na Drive. Lze změnit ukazatel i Data na které ukazuje.
const Drive *drv;         // ukazatel na konstantní Drive. Lze změnit ukazatel, Data na které ukazuje lze pouze číst.
Drive * const drv;        // konstantní ukazatel na Drive. Lze změnit data, na které ukazuje. Ukazatel lze pouze číst.
const Drive * const drv;  // konstantní ukazatel na konstantní Drive. Data i ukazatel lze pouze číst.
```

Dále nechceme, aby do ukazatele v inicializaci instance `Controller`u někdo vložil `nullptr` (nedává to v API smysl, 
bez instance `Drive` neumí `Controller` žít). Navíc nechceme použít **pointerovou aritmetiku** (z principu se jí 
vyhýbáme, kde to jde). Místo konstantního ukazatele `Drive * const ptr` použijeme referenci `Drive & ptr`. 
Pozor reference je vždy konstantní, takže nám jedno `const` odpadá ! 

```c++
Drive & drv;              // reference na Drive. Lze změnit Data na které ukazuje. Ukazatel ne.
```

## Vzorové řešení

Objekt `Controller` tedy deklarujeme v headeru takto: 

```c++
class Drive;    // pouze forward

class Controller {
public:
  Controller(Drive & aDrive); 
  
  void Method();
  
private:
  Drive & drv;
}
```

A implementujeme v cpp souboru takto:

```c++
#include "Controller.h"
#include "Drive.h"          // metody controlleru už smí používat API Drive

Controller::Controller(Drive & aDrive)
 : drv{aDrive}
{
} 

void Controller::Method()
{
  drv->CallMe();            // Můžeme používat API
}
```

## Cyklická závislost

Stejným způsobem lze definovat, že `Drive` může přistupovat k prvkům `Controlleru`. Standardním způsobem deklarace 
bysme vytvořili cyklickou závislost mezi headery a kód by (logicky) nešel zkompilovat:

Drive.h:
```c++
#include "Controller.h"

class Drive {
 Controller & ctrl;
}
```

Controller.h:
```c++
#include "Drive.h"

class Controller {
 Drive & drv;
}
``` 

**Řešení** cyklické závislosti bylo již uvedeno:

```c++
class Controller;

class Drive {
 Controller & ctrl;
}
```

Controller.h:
```c++
class Drive;

class Controller {
 Drive & drv;
}
```