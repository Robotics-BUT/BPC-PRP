# Controller

Někdy je třeba řídit robot v různých okamžicích různým způsobem (např. robot nejprve musí zkalibrovat senzor čáry, aby 
mohl vyrazit na soutěžní trať ), přičemž potřebujeme, aby důležité části chodu supersmyčky probíhaly ve správném čase

Ideální je implementace stavového automatu. Jde ji řešit hloupým a nepřehledným způsobem **switch-case** nebo **funkcionálně**

## Funkcionální přístup

Nejprve musíme deklarovat **ukazatel** na prováděcí **funkci stavu**. 

Pokud budou všechny stavy obsluhovány jedním objektem, je možné vytvořit ukazatel na metodu aktuálního objektu. 

```c++
using State = void(StateController::*)();                                       // ukazatel na stavovou funkci
State state;                                                                    // stav automatu
state = &StateController::DoSearchLine;                                         // změna stavu
(this->*state)();                                                               // vyvolání stavové funkce
```

Pokud však bude obsluha ve více objektech, je nutné použít lambda funkcí a std::function

```c++
using State = void();                                                           // ukazatel na stavovou funkci
std::function<State> state;                                                     // stav automatu
state = [=](){ regulator->DoSearchLine(); }                                     // změna stavu
(*state)();                                                                     // vyvolání stavové funkce
```

Doporučuji první způsob.

### Řešení funkcionálním přístupem s pomocí metod aktuálního objektu

Deklarace Controlleru

```c++
class StateController {
  using State = void(StateController::*)();                                     // MAGIE: ukazatel na metodu controlleru
public:
  State state{&StateController::DoSearchLine}                                   // Robot po startu zacne hledat caru, prvni stav
  
  void Control();                                                               // funkce která bude volána z main
  
  // jednotlive stavove "funkce"
  void DoSearchLine();
  void DoFollowLine();
  void DoTryFollowMissingLine();
  void DoDanceOnFloor();
  void DoBurnEverything();
  // ...
}
```

Příklad implementací jednotlivých stavů automatu:

```c++
void StateController::Control()
{
  (this->*state)();                                                             // MAGIE: provede volání aktuálně vybrané funkce
}

// Robot hleda caru
void StateController::DoSearchLine() 
{
  if (!sensor.LineMissing)
    state = &StateController::DoFollowLine;
}  

// robot reguluje pozici na care
void StateController::DoFollowLine()
{
    drive.Regulate(ForwardSpeed, sensor.ComputedDistanceFromLine);
    
    if (sensor.LineMissing)
      state = &StateController::DoTryFollowMissingLine;
}

// robot se pokousi znovu nalezt prerusenou caru
void StateController::DoTryFollowMissingLine()
{
    if (!sensor.LineMissing)
       state = &StateController::DoTryFollowMissingLine;
}
```

A samozřejmě supersmyčka v main:

```c++
StateController controller{&drive, &sensor};                                    // konstrukce controlleru nad daty

while(rum) {
    comm.send(sensor.BuildReads());
    comm.send(drive.BuildReads());
    
    while (ms < millis())
      comm.loop();
      
    ms = millis() + T;
    
    sensor.ProcessLineSensors();
    drive.ComputeOdometry();
    
    controller.Control();                                                       // provede jeden krok aktualniho stavu
    
    drive.ComputeMotorRamps();        
    comm.send(sensor.BuildWrites());
    comm.send(drive.BuildWrites());
  }
```

## Přístup switch-case

Deklarace Controlleru

```c++
enum State {
    SearchLine,
    FollowLine,
    TryFollowMissingLine,
    DanceOnFloor,
    DoBurnEverything,
  };                                                                            // Deklarace stavu
  
class StateController {
public:
  State state{SearchLine};                                                      // Robot po startu zacne hledat caru, prvni stav
  
  void Control();                                                               // funkce která bude volána z main
}
```

Příklad implementací jednotlivých stavů automatu:

```c++
void StateController::Control()
{
  switch (state) {
  default:
  case SearchLine:
    if (!sensor.LineMissing)
      state = FollowLine;
    break;  
    
  case FollowLine:
    drive.Regulate(ForwardSpeed, sensor.ComputedDistanceFromLine);
    
    if (sensor.LineMissing)
      state = TryFollowMissingLine;
    break;
      
  case TryFollowMissingLine:
    if (!sensor.LineMissing)
      state = FollowLine;
    break;
  }  
}
```

**Poznámka:** V příkladech chybí deklarace lokálních proměnných objektu, příklady jsou uvedeny jen jako vzorové, pro pochopení funkce. Konkrétní implementaci si musíte vytvořit sami.

Chytrého nakopni, hloupého kopni, blbého zakopej 4 metry pod zem ...