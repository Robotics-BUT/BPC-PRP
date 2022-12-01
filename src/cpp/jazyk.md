# Vlastnosti jakzyka C++17 / C++20

O těchto vlastnostech jazyka jste se pravděpodobně neučili a bude dobré když je začleníte do svých znalostí

## Vynucená inicializace

```c++
  struct Trida {
     int A;
     int B;
     double C;
  };
  
  // ...
  Trida instance{1, 2, 3.14159265};  // Vynucena inicializace
```

Vynucuje inicializaci položek A,B a C v daném pořadí podle deklarace uvnitř objektu. Všechny další nastaví na 0.

## Vynucené rozbalení

```c++
  struct Trida {
     int A;
     int B;
     double C;
  };
  
  // ...
  Trida instance{1, 2, 3.14159265};  // Vynucena inicializace
  
  auto [a, b, c] = instance;            // vynucene rozbaleni
```


## Jmenný alias

```c++
  using NovyNazev = StaryNazev 
```

Vytvoří zástupné jméno pro komplikovanější zápis. Používá se zejména pro zvýšení čitelnsoti.

Kdekoliv je použit `NovyNazev`, je to jako byl na daném místě použit datový typ `StaryNazev` (oba typy jsou zaměnitelné)

Priklad:

```c++
  using Msg = std::vector<std::string>;
  using Mediator = std::map<std::string, std::function<void(const Msg &)>>
  
  Mediator mediator;   // odpovica std::map<std::string, std::function<void(const std::vector<std::string>&)>>
```

## Definice callbacku

```c++
  std::function<deklarace funkce> Callback;  
```

Kdykoliv potřebujete zavolat funkci s neznámým cílem funkce (koho máte volat definuje někdo jiný, nadřazený) 
použijete callback jako ukazatel na funkci. Tato funkce ale narozdíl od C-čkového ukazatele na funkci může 
být i součástí objektu. Ukazatel na funkce z jazyka C prosím nepoužívejte! 

Příklad pouziti:

```c++
  void funkce(std::string par) 
  {
    // ...
  }
  
  // ...
  std::function<void(std::string)> Callback;    // Deklarace callbacku
  Callback = &funkce;                           // Přiřazení koho má volat
  
  // ...
  
  if (Callback)                                 // Test zdali odkaz vede na nejaky obsah
    Callback("Ahoj světe");                     // Zavolani obsahu

  // ... 
```

## Lambda

```c++
  [zachyt](argumenty) -> navratovy typ { kod funkce }  
```

Je to deklarace těla funkce bez názvu. 

Do zachyt patří všechny proměnné, které funkce potřebuje ke svému běhu, nebo `&` pokud chceme zachytit všechny automatické 
proměnné referencí, nebo `=` pokud kopií. Pokud je v zachyt znak `&` nebo `=` a deklarace se nachází uvntř kontextu objektu, 
tak se implcitně zachytí i tento objekt.

Návratový typ se šipkou nemusí být specifikován, odvodí se pak z kontextu (z datovéhé typu za return)

Argumenty jsou standardní argumenty funkce jako při deklaraci. Pokud je deklarace na vhodném místě ze kterého překladač pozná 
kontext, můžeme používat auto místo datových typů.

Příklad:

```c++
  int a = 42;
  auto f1 = [a]() { cout << a << endl; };                   // a zachyceno hodnotou
  auto f2 = [&a]() { cout << ++a << endl; };                // a zachyceno referenci, muzu jej menit
  auto f3 = [=]() { cout << a << endl; };                   // a zachyceno hodnotou
  auto f4 = [&]() { cout << --a << endl; };                 // a zachyceno referenci, muzu jej menit
  auto f5 = [&]() { return this->NejakaFunkce(); };         // Navratova hodnota dedukovana jako navrat z NejakeFunkce (muze modifikovat objekt protoze zachyt referenci)  
  auto f6 = [=]() { this->NejakaFunkce(); };                // Pozor NejakaFunkce pracuje nad KOPII objektu - modifikuje kopii, ne puvodni objekt
  
  f1();                                                     // lambdu volam stejne jako funkci
```

Lambdy je dobre pouzivat az od C++17 protoze v C++14 a nizsim maji neintuitivni chovani ve specifickych pripadech 
a kompilator vyhazuje chyby ktere nejsou na prvni pohled zrejme


## Procházení všech prvků kontejneru

```c++
  for (auto i: <kontejner>) {
    // operace s prvkem i
  }
```

## Validátor platnosti s deklarací

Pomucka pro citelnost a optimalizaci kodu - omezení platnosti proměnné pouze na úsek, kdy je platná. Pokud není platná, kompilátor ji může zahodit.

```c++
  if (auto obj = DejObjekt()) {
    // zde muzu pracovat s obj pokud neni null nebo operator bool() objektu ktery vratila funkce DejObjekt vraci true
  }
  // zde obj neexistuje
```

```c++
  if (auto obj = DejObjekt(); Podminka) {
    // zde muzu pracovat s obj pokud je podminka splnena (vraci true)
  }
  // zde obj neexistuje
```

Analogicky u `while`.
