# Instalace a seznámení se s prostředím
Cvičící: Ing. Adam Ligocki, Ph.D., Ing. Tomáš Horeličan

## Linux Command Line Interface - CLI (cca 45min)

V první řadě si projděte kapitolu [Linux a příkazová řádka](../chap_1_software/text/linux.md). Následně si procvičte použití příkazů
několika cvičnými úlohami:

Tip: při psaní příkazů používejte TAB pro doplnění příkazu a TAB-TAB pro výpis všech možných doplnění příkazů. 

✅ Zjistěte, kde se momentálně necházíte v rámci souborového systému

✅ Přepněte se do svého domovského adresáře

✅ Vytvořte si složku ve které buje projekt pro dnešní cvičení

✅ Následně v této složce vytvořte několik podsložek tak, aby struktura vypadala následovně (příkaz tree):

```
/MyProject
 |--build
 |--include
 | \-- MyProject
 \--src
```

✅ Ve Vašem domovském adresáři si pomocí textového editoru (nano nebo vim) vytvořte soubory main.cpp, lib.hpp a CMakeLists.txt

✅ Soubor main.cpp přesuňte (nikoliv zkopirujte) do podsložky "src"

✅ Soubor lib.hpp přesuňte do podsložky "include/MyProject"

✅ Soubor CMakeLists.txt přesuňte do hlavní složky projektu

Nyní by měl Váš projekt mít tuto podobu:

```
/MyProject
 |--build
 |--CMakeLists.txt
 |--include
 | \--MyProject
 |  \--lib.hpp
 \--src
  \--main.cpp
```

✅ Pomocí textového editoru napište v souboru lib.hpp funkci, která vypíše pozdrav do konzole.

✅ Pomocí textového editoru napište v souboru lib.hpp funkci, která vypíše pozdrav do konzole.

✅ Zeditujte CMakeLists.txt na následjící obsah:

```
cmake_minimum_required(VERSION 3.10)
project(MyProject)
set(CMAKE_CXX_STANDARD 17)
include_directories(include/)
add_executable(hello_world_program src/main.cpp)
```

Významem jednotlivých direktiv se budeme zabývat později.


✅ Nyní se přesuňte do složky "build" a zde zavolejte příkaz

```
cmake ..
```

CMake si přečte nastavení vašeho projektu, a připraví makefile pro následnou kompilaci.

✅ Příkazem make zkompilujte program.

✅ vypište si obsah složky build.

✅ Pokud kompilace proběhla úspěšně, spustěte program voláním "./hello_world_program".

✅ Optional: vyzkoušejte si kompilaci i přímo pomocí volání g++.

```
g++ <source1 source2 source3 ...> -I <include_directory> -o <output_binary> 
```

✅ Smažte celou složku s projektem (pozor, složky se mažou rekurzivně).

## CLion a C++ (cca 1.5h)

✅ Vytvořte identický CMake projekt z minulého bodu zadání pomocí CLionu. Progam zkompilujte a otestujte.

### OOP

Lorem Ipsum

### Reference

Lorem Ipsum

### Smart Pointers

Lorem Ipsum

### STL Structures

Lorem Ipsum



