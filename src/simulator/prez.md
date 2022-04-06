# Prezentační protokol (NMEA-0183)

## Popis protokolu

 * Používá se obvykle při komunikaci s GNSS přijímači (GPS)
 * Existují přijímače, které komunikují na sériové lince i UDP (sdílejí protokol)
 * Jeden přenášený UDP packet může obsahovat **více zpráv**
 * Každá NMEA zpráva je **textový řetězec** sestavený z **argumentů**
 * Každá zpráva je **bezestavová** (není potřeba znát předchozí zprávy na sběrnici)
 * První argument definuje **typ zprávy** a tím i **význam** následujících argumentů

## Formát jedné zprávy

```
$NMEA,MESSAGE,3,15,3.14,9E-5*FF\r\n
```

Každá zpráva obsahuje následující položky:

 * Před zprávou můžou být **libovolná** (i binární!) data
 * **Začátek** zprávy obsahuje znak `$` (nebo vyjímečně `!` nebo `?`)
   * Zprávy uvozené pomocí znaků `!` nebo `?` jsou ignorovány (původně dotaz-odpověď) 
 * Následuje seznam **argumentů** zprávy. V každém argumentu se může vyskytnout:
   * malá a velká písmena základní ASCII (`a-zA-Z`)
   * mezera a podtržítko (` _`) 
   * číslice (`0-9`)
   * desetinná tečka (`.`)
   * znak znaménka (`+-`)
   * čárka oddělující jednotlivé argumenty mezi sebou (`,`) 
 * Argumenty jsou ukončeny hvězdičkou `*`
 * Dále následují dva hexadecimální znaky pro **kontrolní součet** zprávy
   * Spočítán jako postupný 8-bitový XOR všech bajtů zprávy mezi `$` a `*`
   * Můžou být použita malá i velká písmena
 * Zpráva může být **volitelně ukončena** znaky `\r\n` pro lepší zobrazení v souboru 
 * Za zprávou můžou být **libovolná** (i binární!) data

## Doporučené rozhraní pro implementaci

###  Data

Nomenklatura:

 * Zprávu nižší úrovně (transportní) nazýváme **frame**
 * Zprávu vyšší úrovně (aplikační) nazýváme **logická zpráva**

Fakta:
 
 * Jelikož je protokol **bezestavový**, není potřeba stav držet
   * Není potřeba instance objektu, stačí `static` funkce
   * Dokonce není potřeba objekt, je vhodné ale funkce oddělit alespoň namespace
 * Zpráva na straně aplikace reprezentuje **seznam textových argumentů**
   * Vhodný typ pro logickou zprávu `std::vector(std::string)`
 * Zpráva na straně transportu reprezentuje **textový řetězec**
   * Vhodný typ pro frame `std::string`
 * Zpráva na straně aplikace bude hojně využívána
   * Vytvoříme vhodný logický **alias** `Msg` abysme nemuseli moc psát
 * Zprávu musí být **jednoduché vytvořit**, musí být přehledný zápis
   * Konstrukce s initializer-list : `return {"ODO", std::to_string(3.14)}`
 * Prázdná aplikační zpráva (**bez položek**) indikuje **neplatnou zprávu**
   * Konstrukce `return {}`
   * Test `if (msg.empty()) ...`
   * Výjimka: zprávu "bez dat" lze stále vytvořit: `return {""}` např jako heartbeat :o)

Implementace:

```c++
 using Msg = std::vector<std::string>;                                           // vytvoření aliasu
```

### Rozhraní komunikace k transportní vrstvě

Fakta:

 * Je třeba vytvořit **konverzi** argumentů z logické zprávy na frame
 * Funkce nesmí změnit obsah argumentů vyššího protokolu
   * Datová struktura na vstupu tedy odpovídá `Immutable` [viz odkaz](https://cs.wikipedia.org/wiki/Immutable_object) 
   * Realizace v `C++` - využijeme `const`
 * Z důvodu optimalizace výkonu je vhodné předat vektor jako ukazatel na data
   * Ukazatel formátujeme pomocí `*prom`
 * Je nelogické, aby byla formátována zpráva s neplatným ukazatelem (`nullptr`)
   * Reference je ukazatel který nesmí být `nullptr`
   * Použijeme referenci `&prom`
 * Naskýtá se kostra design patternu `Builder` [viz odkaz](https://cs.wikipedia.org/wiki/Builder)
   * Vytváříme objekt (`std::string`) na základě argumentů (`Msg`)
   * Proto tento přechod takto nazveme
 * Jeden frame může obsahovat více zpráv
   * Je vhodné vytvořit overload funkce `Build` pro `std::vector(Msg)`
   * Jiná možnost je přidat funkci `BuildAll` která vnitřně volá `Build`

Implementace: 

```c++
  //std::string Build(const std::vector<std::string> &msg);                      // bez aliasu
  std::string Build(const Msg &msg);                                             // alias
  std::string BuildAll(const std::vector<const Msg> &msg);                       // overload s aliasem
```

Overload funkce pro převod několika zpráv naráz je triviální:

```c++
  std::string BuildAll(const std::vector<const Msg> &msg) 
  {
    std::string result;
    
    for (const auto &i : msg)
      result += Build(i);
      
    return result;   
  }
```

### Rozhraní komunikace k aplikační vrstvě

Fakta:

 * Je třeba vytvořit **parser** argumentů z aplikační vrstvy na logickou zprávu
 * Jedna logická zpráva transportního protokolu **může obsahovat více zpráv**

#### Implementace I:

```c++
  //std::vector<std::string> Parse(const std::string &msg);                      // bez aliasu
  Msg Parse(const std::string &msg);                                             // alias  
```

V aplikačním kódu pak obsluha vypadá takto:

```c++
  auto packet = "$PING*10$PING*10";
  // obsluha transportni vrstvy                                              
  auto msg = Parse(packet))                                                      // Zde může být string const z vnitřního API transportního protokolu
  // obsluha aplikacni vrstvy
  if (msg[0] == "PING") ....
```

Tato implementace obsahuje chybu (prvotní implementace na cvičení), kdy veškeré další zprávy uvnitř packetu jsou ignorovány

Pro úspěšné splnění předmětu tato implementace plně dostačuje. Ostatní implementace jsou akademická diskuse nad správným řešením

#### Implementace II:

```c++
  //std::vector<std::vector<std::string>> ParseAll(const std::string &msg);         // bez aliasu
  std::vector<Msg> ParseAll(const std::string &msg);                             // alias  
```

V aplikačním kódu pak obsluha vypadá takto:

```c++
  auto packet = "$PING*10$PING*10";
  // obsluha transportni vrstvy
  for (auto &msg : ParseAll(packet)) {                                           // Zde může být string const z vnitřního API transportního protokolu
    // obsluha aplikacni vrstvy
    if (msg[0] == "PING") ....
  }
```

Tato implementace obsahuje výkonnostní problém v paměti (**každá zpráva bude v paměti vždy několikrát**), 
získáme tím ale optické oddělení vrstev v kódu


#### Implementace III:

 * Funkce vyjme každou zprávu ze vstupního řetězce, a zpracuje se postupně
   * Princip pipeliningu
   * Vstupní řetězec je modifikován !

```c++
  std::vector<std::string> Parse(std::string &msg);                              // bez aliasu
  Msg Parse(std::string &msg);                                                   // alias
```

V aplikačním kódu pak obsluha vypadá takto:

```c++
  auto packet = "$PING*10$PING*10";
  // obsluha transportni vrstvy
  std::string str{packet};                                                       // Zde musí být vytvořena nová lokální proměnná 
  for (auto msg = Parse(str); !msg.empty(); msg = Parse(str)) {
    // obsluha aplikacni vrstvy
    if (msg[0] == "PING") .... 
  }
```

#### Implementace IV:

Využití `std::string_view` - nejlepší možnost, bez kopírování ale korektně lze implementovat až od `C++23`, který není 
podporován současnými kompilátory použitými v raspbianu (GCC 9)



### Pomocné funkce

Pro obě rozhraní je potřeba vytvořit funkci, která spočítá checksum z předaných dat

```c++
std::string Crc(const std::string &text);
```