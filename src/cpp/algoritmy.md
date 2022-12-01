# Standardn9 kontejnery v C++

## std::string

  Je to kontejner, pro obyčejný text. Obsahuje základní funkce pro práci s textem.
  
  Může sice obsahovat UTF-8, ale nepočítá jeden znak UTF-8 jako primitivu, ale jako sekvenci několika bajtů. 

## std::vector&lt;typ&gt;

  Je to dynamické pole, které obsahuje prvky specifikovaného datového typu. 

  Jedotlivé prvky jsou indexovány od nuly

```c++
  std::vector<int> vektor{
    1,3,5,7,11,13,17,
  };
  
  vektor[5] = 4;                              // zápis do položky
  cout << vektor[0] * vektor[2] << endl;      // čtení z položky
  
  if (auto p = vektor.find(13); p != vektor.end()) {  // hledání prvku
    cout << *p << endl;                       // prvek se nachází ve vektoru
  }
```

## std::unordered_map&lt;klíč,typ&gt;

  Je to adresovatelný kontejner, kde každá hodnota má specifikovaný klíč, pod kterým ji lze najít. 
  Kontejner zajišťuje oproti std::map co nejrychlejší adresaci za pomocí hashe.
  
```c++
  std::unordered_map<std::string, double> mapa{
    { "PI", 3.14159},
    { "E", 2.71828},
  };
  
  mapa.emplace_back
  mapa["FN"] = 598722.4879;                         // definice Baštincova čísla  (zápis do položky)
  cout << mapa["E"] * mapa["E"] << endl;            // čtení z položky
  
  if (auto p = mapa.find("ZY"); p != mapa.end()) {  // hledání prvku
    cout << p->second << endl;                      // ZY se nachází v mapě
  }
```

