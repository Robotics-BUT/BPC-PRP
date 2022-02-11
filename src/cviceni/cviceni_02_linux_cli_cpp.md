# Instalace a seznámení se s prostředím
Cvičící: Ing. Adam Ligocki, Ph.D., Ing. Tomáš Horeličan

## Linux Command Line Interface - CLI (cca 45min)

V první řadě si projděte kapitolu [Linux a příkazová řádka](../chap_1_software/text/linux.md). Následně si procvičte použití příkazů
několika cvičnými úlohami:

Tip: při psaní příkazů používejte TAB pro doplnění příkazu a TAB-TAB pro výpis všech možných doplnění příkazů. 

✅ Zjistěte, kde se momentálně necházíte v rámci souborového systému

✅ Přepněte se do svého domovského adresáře

✅ Vytvořte si složku ve které bude projekt pro dnešní cvičení

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
 |   \--lib.hpp
 \--src
   \--main.cpp
```

✅ Pomocí textového editoru napište v souboru lib.hpp funkci, která vypíše pozdrav do konzole.

✅ Zeditujte CMakeLists.txt na následjící obsah:

```cmake
cmake_minimum_required(VERSION 3.10)
project(MyProject)
set(CMAKE_CXX_STANDARD 17)
include_directories(include/)
add_executable(hello_world_program src/main.cpp)
```

Významem jednotlivých direktiv se budeme zabývat později.


✅ Nyní se přesuňte do složky "build" a zde zavolejte příkaz

```shell
cmake ..
```

CMake si přečte nastavení vašeho projektu, a připraví makefile pro následnou kompilaci.

✅ Příkazem make zkompilujte program.

✅ vypište si obsah složky build.

✅ Pokud kompilace proběhla úspěšně, spustěte program voláním "./hello_world_program".

✅ Optional: vyzkoušejte si kompilaci i přímo pomocí volání g++.

```shell
g++ <source1 source2 source3 ...> -I <include_directory> -o <output_binary> 
```

✅ Smažte celou složku s projektem (pozor, složky se mažou rekurzivně).

## CLion a C++ (cca 1.5h)

✅ Vytvořte identický CMake projekt z minulého bodu zadání pomocí CLionu. Progam zkompilujte a otestujte.

Detail k CLionu [zde](chap_1_software/text/clion.md).

### STL Structures

Součástí jazyka C++ je tzv. Standard Template Library (STL). Ta obsahuje širokou paletu různých datových struktur a naimplementovaných fukncí.
My se dnes zaměříme na část která implementuje datové konteinery.

Knihovna obsahuje implementace pro pole, zásobník, vektor, frontu, list, množinu, mapu (hash_tabulku/dictionary), atd. [Dokumentace zde](https://en.cppreference.com/w/cpp/container).

My se dnes zamšříme na tři struktury, a totiž std::array, std::vector a std::queue.

Struktury se mezi sebou liší a každá je vhodná pro jiný účel. std::array je struktura v paměti, která má známou svou velikost již během kompilace.
Pokusme se nyní takové pole vytvořit, naplnit jej hodnotami a vypočítat průměr.

```cpp
    #include <array>

    auto my_array = std::array<int, 5>{0, 1, 2, 3, 4};
    int sum = 0;
    for (const auto& val : my_array) {
        sum += val; 
        // sum += my_array.at(i); // equivalent approach
    }
    auto avg = sum / my_array.size();
```

Vektor se od pole liší tím, že má proměnnou velikost. Vždy když se naplní, tak se automaticky naalokuje navíc jednonásobek jeho současné velikosti.

Vyzkoušíme si naplnit vektor několika hodnotami a najít medián těchto hodnot.

```cpp
    #include <vector>
    #include <algorithm>
    
    auto my_vector = std::vector<float>{};
    my_vector.push_back(5.4);
    my_vector.push_back(-3.7);
    my_vector.push_back(10.9);
    my_vector.push_back(1.3);
    my_vector.push_back(-6.5);
    my_vector.push_back(-7.8);
    my_vector.push_back(6.4);

    std::sort(my_vector.begin(), my_vector.end());
    auto med = my_vector.at(my_vector.size()/2);
    my_vector.clear();
```

Strukturu fronty využijeme jako buffer v ilustračním scénáří zpracování příchozích dat z UDP.
Uvažujme multivláknový program. Jedno vlákno přijmá data po UDP a plní frontu. Druhé vlákno pracuje asynchronně a vždy,
když přijde na řadu, zpracuje všechny doposud přijaté zprávy v pořadí tak, jak příšly.

```cpp
    #include <queue>
        
    // queue shared between threads; Tip: mutex ?!
    auto my_queue = std::queue<std::string>{};
    
    // receive thread filling queue with messages
    my_queue.push("Message1");
    my_queue.push("Message2");
    my_queue.push("Message3");

    // message processing thread
    while (!my_queue.empty()) {
        parse_message(my_queue.front());
        my_queue.pop();
    }
```


### Reference

Reference, někdy také nazývané "alias", je datový typ, který směřuje (je aliasem) na již existujicí objekt v paměti. 
Při kompilaci je reference obvykle nahrazena ukazatelem, ale z pohledu programátora se jedná o výrazně bezpečnější formu
práce s daty a, či objekty, protože nedovoluje některé nebezpečné operace.

Reference se liší od ukazatele ve dvou základních vlastnostech:
 - Nemůže být NULL; reference je vždy nainicializovaná
 - Reference se nemůže přesměrovat na jiný objekt/data.

Pozor, nezaměňovat datový typ reference "<T>&" s operátorem reference "&variable" !

```cpp
    int a = 5;
    int& b = a;
    const int& c = a;
    b = 10;
    c = 15 // invalid (const ref)
    std::cout << a << std::endl; // a == 10
```

Reference je často používaná pro předání argumentů fukce bez nutnosti kopírování, či pro sdílení jedněch dat mezi více místy v programu.

```cpp
    class VeryLargeObject {
    public:
        VeryLargeObject() {}
        const std::array<double, 10000>& data() const {return data_;}
    private:
        std::array<double, 10000> data_;
    };
    
    void porocess_large_data(const VeryLargeObject& d) {
        auto& data = d.data(); // const reference
        auto data = d.data(); // mutable copy
        // ...
    }
    
    auto vlo = VeryLargeObject{};
    porocess_large_data(vlo);
```


Reference je často pužívaná pro vrácení hodnot z funkce skrze argument funkce.
Nejedná se však o best-practice metodu. Pokud je to jen trochu možné, měla by metoda vracet hodnotu skrze návratovou 
hodnotu. Pokud je potřeba vrátit více hodnot, použijte strukturu jako návratový typ.

```cpp
    void ops(float a, float b, float& sum, float& sub, float& mul, float& div) {
        sum = a + b;
        sub = a - b;
        mul = a * b;
        div = a / b;
    }

    float sum, sub, mul, div;
    ops(5, 10, sum, sub, mul, div);
    std::cout << sum << " " << sub << " " << mul << " " << div << std::endl;
```

### Smart Pointers

Smart pointery jsou náhradou C-čkových ukazatelů. V základu máme 3 typy těchto smart ukazatelů:
 - std::unique_ptr\<T>  
 - std::shared_ptr\<T>
 - std::weak_ptr\<T>

kde T je datový typ na který bude ukazatel ukazovat.

Vyhodou smart pointerů je, že nemusíme jako programátoři bezprostředně řešit alokaci a zejména uvolnění paměti. 
Jsou li splněny podmínky, smartpointer během svého zániku zavolá také destruktor objektu, na který ukazoval a uvolní naalokovanou paměť.

Výsledkem je, že programátoru už nemusí používat klíčová slova ```new``` a ```delete```.

Každý ze smart pointerů se však mírně liší.

#### std::unique_ptr\<T>

std::unique_ptr<T> je nejtriviálnější implementací smart pointeru. Smart pointer je vlastníkem objektu na který ukazuje a 
neumožní toto vlastnictví (ownership) předat jinému ukazateli. Když unique_ptr zanikne, zavolá destruktor nad vlasněným objektem a dealokuje paměť.

#### std::shared_ptr\<T>

std::shared_ptr<T> je příkladem tzv. Automatic Reference Counter (ACR). Idea je, že při vzniku objektu se vytvoří také čítač, který čítá kolik shared_pointerů na tento objekt ukazuje.
Když vytvářím nové kopie shared pointeru, čítač roste, když tyto smart pointery zanikají, hodnota čítače klesá.

Když čítač dosáhne nuly, to znamená, že na objekt už nic neukazuje, je automaticky zavolán destruktor a je uvolněná paměť.

Pozor, nezaměňovat s Garbage Collectorem (GC), ten funguje výrazně jinak.

Pozor na cyklické vazby. Pokud dva objekty na sebe navzájem ukazují shared pointerem, ani jeden z objektů nikdy nezanikne. Proto zde máme weak pointery.

#### std::weak_ptr\<T>

Obdoba shared_ptr, ale neinkrementuje čitač, který počítá, kolik je platných ukazatelů na daný objekt. To znamená, že pokud na objekt ukazuje 5 weak_ptr a žáden shared_ptr, objekt zanikne.

### OOP

Při tvobě Vaších programů se snažte dodržovat OOP paradigma. Přemýšlejte o programu, jako o sadě black-boxů, kdy tyto schránky 
jsou každá zaměřená na velmi specifický problém. Každou Vaší třídu by měla vystihovat jedna věta. Stejně tak každá funkce
by měla dělat právě jednu věc a nic víc.

Zmíněné blackboxy jsou mezi sebou propojeny a navzájem si předávají data.

Vyhněte se tvorbě "supertříd", tedy tříd, které řeší "všechno". Mějte své třídy úzce specializované.

Běžně by se měla třída vměstnant do 100 řádku. Pokud je třída nad 300 řádků, silně zvažte její rozdělení na více tříd.

Oddělte data od algoritmů. Vytvořte si oddělené třídy, které v sobě mají uložená data a oddelené třídy, které implementují algoritmy pro zpracování dat.


#### Příklad

Naimplementujte příklad pomocí OOP C++. Při implementaci využijte reference a smart pointery.

Mějme univerzitu. Každá univerzita má 5 ročníků, v každém ročníku je libovolný počet studentů.
Když studenti nastupují na univerzitu, jsou automaticky zařazeni do 1. ročníku. Vždy, když proběhne rok, 
tak univerzita prozkouší všechny studenty v ročnících a s pravděpodobností 0.9 posune studenta do vyžšího ročníku. Pokud student projde pátý ročníku, 
univerzita si jej zaznamená jako absolventa.
Na konci každého roku vytiskněte stav univerzity a všech studentů na ní.

Tip: Třídy a jejich členské proměnné:

```
Trida Student:
promenne:
    jmeno, 
    prijmeni
metody:
```


```
Trida Rocnik:
proměnné:
    seznam_vsech_studentu
metody:
    pridat_studenta_do_rocniku(student)
    evaluovat_ročník() -> seznam_uspesnych_studentu
```


```
Třída Univezita:
proměnné:
    seznam_rocniku
    seznam absolventu
metody:
    vykonat_akademicky_rok()
    vytisknout_stav_univerzity();
```

Implementace:

```cpp
#include <iostream>
#include <array>
#include <vector>
#include <memory>
#include <random>

class Student {
public:
    Student(const std::string& first_name, const std::string& surname)
        : first_name_{first_name}
        , surname_{surname} {}
    std::string first_name() const {return first_name_;};
    std::string surname() const {return surname_;};
private:
        const std::string first_name_;
        const std::string surname_;
    };
    
    
class Grade {
        static constexpr float change_of_student_passes_grade = 0.8f;
public:
    void add_student(std::shared_ptr<Student> stud) {students_.push_back(stud);}
    std::vector<std::shared_ptr<Student>> evaluate_year() {
        std::vector<std::shared_ptr<Student>> successful_students{};
        std::vector<std::shared_ptr<Student>> failed_students{};
        for (auto& stud : students_) {
            auto random_num = get_random_number(0.0f, 1.0f);
            if (random_num > change_of_student_passes_grade) {
                failed_students.push_back(stud);
            }
            else {
                successful_students.push_back(stud);
            }
        }
        students_ = failed_students;
        return successful_students;
    }
    std::vector<std::shared_ptr<Student>> students() const {return students_;}
    float get_random_number(float min, float max) {
        static std::random_device rd;
        static std::mt19937 gen(rd());
        static std::uniform_real_distribution<float> distr(min, max);
        return distr(gen);
    }
private:
    std::vector<std::shared_ptr<Student>> students_;
};
    
    
class University {
    static constexpr size_t no_of_grades = 5;
public:
    void add_student(std::shared_ptr<Student> stud) {grades_.at(0).add_student(stud);}
    void evaluate_year() {
        for (int i = no_of_grades-1 ; i >= 0 ; i--) {
            auto successfull_studs = grades_.at(i).evaluate_year();
            if (i == no_of_grades-1) { // last grade
                for (auto& stud : successfull_studs) {graduated_.push_back(stud);}
            } else {
                for (auto& stud : successfull_studs) {grades_.at(i+1).add_student(stud);}
            }
        }
    }
    void print_state() {
        for(size_t i = 0 ; i < no_of_grades ; i++) {
            std::cout << "    Grade:" << i+1 << std::endl;
            auto studs = grades_.at(i).students();
            for (const auto& stud : studs) {
                std::cout << "       " << stud->first_name() << " " << stud->surname() << std::endl;
            }
        }
        std::cout << "    Graduated:" << std::endl;
        for (const auto& stud : graduated_) {
            std::cout << "       " << stud->first_name() << " " << stud->surname() << std::endl;
        }
    }
private:
    std::array<Grade, no_of_grades> grades_;
    std::vector<std::shared_ptr<Student>> graduated_;
};
    
int main() {
    University Oxenfurt;
    Oxenfurt.add_student(std::make_shared<Student>("Triss", "Merigold"));
    Oxenfurt.add_student(std::make_shared<Student>("Geralt", "of Rivia"));
    Oxenfurt.add_student(std::make_shared<Student>("Zoltan", "Chivay"));
    Oxenfurt.add_student(std::make_shared<Student>("Yennefer", "of Vengerberg"));
    Oxenfurt.add_student(std::make_shared<Student>("Cirilla", "of Cintra"));
    for (size_t i = 0 ; i < 6 ; i++) {
        std::cout << " ---------- " << std::endl;
        std::cout << "Year " << i+1 << std::endl;
        Oxenfurt.evaluate_year();
        Oxenfurt.print_state();
    }
    return 0;
}
```

### Const

Rychlý přehled užití const v kódu

```cpp

    // Helpre Object
    class Object {
    public:
        void do_non_const_work() {counter++;} // non-const member method
        void do_const_work() const {std::cout << counter << std::endl;} // const method, can not modify member variables
    private:
        int counter = 0;
    };


     // Variables
     
    int a = 1; // mutable variable
    const int b = 2; // non-mutable (const) variable
    
    
    // References
    
    int& c = a; // mutable reference to a
    const int& d = a; // const reference to a
    
    
    // Pointers
    
    int* e = &a;    // pointer to a
    const int* f = &a;  // pointer to constant a (value of a can not be changed)
    int const* g = &a;  // the same
    *f = 5; // error
    f = e;  // ok
    
    int *const h = &a;  // non-mutable (const) pointer to mutable variable
    h = e;  // error
    *h = 5; // ok
    
    const int * const i = &a; // const pointer to const variable
    *i = 5; // error
    i = e;  // error
    
    
    // Data Structures
    
    std::vector<Object> v1 = {Object{}, Object{}, Object{}};    // Vector of 3 objects
    const std::vector<Object> v2 = {Object{}, Object{}, Object{}};   // constant vector (can not add or remove values from it); returns const refs to object
    v2.push_back(Object{}); // error
    v2.clear(); // error
    v2.at(0).do_non_const_work(); // error
    v2.at(0).do_const_work(); // ok
    
    
    // Smart Pointers
    
    std::shared_ptr<int> sp1 = std::make_shared<int>(5); // normal shared pointer
    std::shared_ptr<const int> sp2 = sp1; // shared pointer to const value
    *sp2 = 5; // error
    sp2 = sp1; // ok
    const std::shared_ptr<int> sp3 = sp1;   // constant pointer to mutable value
    *sp3 = 5; // ok
    sp3 = sp2; // error
    const std::shared_ptr<const int> sp4 = sp1;
    *sp4 = 5; // error
    sp4 = sp2; // error
    
    
    // Const vs Constexpr vs Define
    
    const int x = 5; // this variable can be initialized in runtime (read user input)
    #define Y = 5 // const defined for preprocessor (non type safe)
    constexpr int y = 5; // this variable MUST be initialized in compile-time (similar to #define Y 5, but type-safe)
    
    
    // "Rustification"
    
    #define let const auto
    #define mut auto
    
    let a = 5;  // cosnt variable
    mut b = 3;  // mutable variable
    let& c = a; // const reference
    mut& d = b; // mutable reference
```


