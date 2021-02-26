# UDP komunikace

Cvičící: Ing. Tomáš Lázna

## Založení projektu (cca 10 minut)

Zapněte CLion a vytvořte nový projekt (File > New Project) typu *C++ Executable*. Zvolte umístění a název svého projektu (např. jméno týmu) a *Language standard* nastavte na *C++17*.

CLion by vám měl generovat prázdnou šablonu C++ aplikace. V prvé řadě budeme editovat soubor *CMakeLists.txt*, abychom kompilátoru řekli, kde má hledat hlavičkové a zdrojové soubory. 
Doporučuji si v adresářové struktuře projektu vytvořit složku *include* pro umístění .h souborů a složku *src* pro .cpp soubory (včetně `main.cpp`, který je defaultně v rootu projektu). 
Zároveň si k projektu přilinkujeme knihovny pro práci s YAML soubory pro načítání konfiguračních souborů (volitelné) a pro práci s vlákny (rovněž volitelné). Soubor by měl mít následující strukturu:

```
cmake_minimum_required(VERSION 3.17)
project(bpc_prp_cool_name)

set(CMAKE_CXX_STANDARD 17)

find_package(Yaml-cpp)
find_package(Threads REQUIRED)

include_directories(include)

set(SOURCES
        src/main.cpp
        src/cool_code.cpp
        src/another_source_file.cpp)

add_executable(bpc_prp_cool_name ${SOURCES})
target_link_libraries(bpc_prp_cool_name yaml-cpp Threads::Threads) 
```
 
Pokud CMake nereloaduje konfiguraci automaticky, v CLionu vám vyskočí panel *CMake project needs to be reloaded*, kliknete na tlačítko *Reload changes*.

## Zpracování NMEA zpráv (cca 1 hodina)

Komunikace se simulátorem probíhá formou NMEA zpráv posílaných přes UDP protokol. Pro sestavení, kontrolu a parsování NMEA zpráv je výhodné si vytvořit sadu utilitárních metod a zabalit je do třídy. Přikládám návrh deklarace
4 metod, kterým se můžete (ale nemusíte) inspirovat, pozornost věnujte zejména vstupním a návratovým typům:

```
std::string string_to_nmea_message(const std::string& message);
uint16_t get_message_checksum(const std::string& message);
std::string extract_nmea_message_content(const std::string& nmea_message);
bool is_nmea_message_valid(const std::string& nmea_message);
```

Obecně je v C++ dobrým nápadem předávat metodám objekty, které nechcete modifikovat, jako konstantní reference (const type&), tím pádem se funkci předá jen odkaz na daný objekt (nemusí se kopírovat) 
a přitom se (díky const) chráníte před tím, abyste omylem objekt modifikovali.

Metody můžete definovat jako statické, tj. pro jejich volání nebude nutné vytvářet instanci třídy.

Pozn. Struktura NMEA zprávy je následující:

```
$MESSAGE_TYPE,PARAMETERS,SEPARATED,BY,COMMAS*checksum
```

Checksum je dvoučíslicové hexadecimální číslo (tj. v rozsahu 0-255), které vznikte jako bitová nonekvivalence (XOR) ASCII kódů všech znaků mezi $ a * (ty nejsou zahrnuty).

Následuje příklad volání jednotlivých metod s očekávanými výstupy tak, abyste si mohli ověřit správnost své implementace.

```
string_to_nmea_message("PING,HELLO") -> "$PING,HELLO*7E"
get_message_checksum("RESET,") -> 126 (= 7E v hexa)
extract_nmea_message_content("$PONG,HELLO*78") -> "PONG,HELLO"
is_nmea_message_valid("$PONG,HELLO*78") -> true
is_nmea_message_valid("$PONG,HELLO*AA") -> false
```

Pozn. Pro sestavování a parsování C++ řetězců (`std::string`) doporučuji využít třídu `std::stringstream` ze standardní knihovny `<stringstream>`.

Úspěšná implementace metod pro práci s NMEA zprávami je nutnou podmínkou pro úspěšnou komunikaci se simulátorem v dalším kroku.

## UDP komunikace přes socket API (cca 40 minut)

Komunikace se simulátorem je realizována ve formě UDP paketů, které budeme odesílat a přijímat pomocí socket API, které je k dispozici ve standardních C knihovnách.

Pro úspěšné fungování je nutné includovat několik hlavičkových souborů:

```
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
```
 
Pro inicializaci socketu zavoláme funkci `socket`, která má následující předpis:

```
int fd = socket(int domain, int type, int protocol);
```

Ve filosofii jazyka C se se sockety pracuje jako se soubory (lze do nich zapisovat a číst z nich), proto funkce vrací celé číslo *file descriptor*, pomocí kterého se daný socket referencuje dále v kódu.

Nápovědu k tomu, s jakými parametry socket inicializovat naleznete v dokumentaci funkce `socket` - budeme pracovat v doméně IPv4 a typem bude UDP (připomeňte si, co tato zkratka znamená). 
V běžných případech (jako i tento) je protokol již definován prostřednictvím typu, třetí parametr tedy může být nastaven na 0.

Korektní je po skončení práce se socketem jej uzavřít pomocí volání:

```
close(fd)
``` 


### Odesílání zpráv

Existují v zásadě dvě varianty, jak odesílat UDP zprávy. 

1. Datagramy je možné posílat i přes tzv. nepojmenované sockety (nemají přiřazenou žádnou adresu a port), přičemž adresát je specifikován ve volání funkce `sendto`. 
Pro pojmenování příjemce využijeme strukturu `sockaddr_in`, a to následujícím způsobem:

```
struct sockaddr_in addr;
addr.sin_family = AF_INET;
addr.sin_addr.s_addr = inet_addr("ip.adresa.pri.jemce");
addr.sin_port = htons(11111);
```

Odeslání zprávy je realizováno následujícím voláním:

```
std::string msg = "message";
ssize_t no_of_sent_bytes = sendto(fd, msg.c_str(), msg.length(), 0, (struct sockaddr *) &addr, sizeof(addr));
```

Funkci jsou předány tyto parametry: deskriptor socketu, obsah zprávy (typ char* = řetězec v C), délka zprávy, příznaky (flags), struktura s popisem adresáta, velikost této struktury. Vrácen je počet odeslaných bytů.

2. Pokud komunikuji opakovaně se stejným příjemcem, je možné socket tzv. pojmenovat a odesílání realizovat funkcí `send`, která nevyžaduje strukturu s adresátem (tato informace bude již předána přes deskriptor socketu).
Způsob pojmenování závisí na tom, zda je vaše aplikace klientem (aktivně se připojuje k jinému socketu) nebo serverem (naopak čeká, než se jiný socket připojí k vašemu). Pro variantu klienta použijeme funkci `connect`:

```
int res = connect(fd, (struct sockaddr *) &addr, sizeof(addr));
```   

Funkci jsou předány tyto parametry: deskriptor socketu, struktura s popisem serveru, velikost této struktury. V případě úspěchu je vrácena 0, při chybě -1.

Varianta serveru bude popsána později v sekci Přijímání zpráv, využívá se funkce `bind`.

K odeslání zprávy slouží následující volání:
 
```
std::string msg = "message";
ssize_t no_of_sent_bytes = send(fd, msg.c_str(), msg.length(), 0);
```

Parametry a návratová hodnota jsou analogické s funkcí `sendto`.

Při využití pojmenovaného socketu může nastat situace, kdy se spojení přeruší, v takovém případě je nutné jej obnovit opětovným voláním funkce `connect`. 

### Přijímání zpráv

Obdobně jako v případě odesílání zpráv máme k dispozici dvě alternativy, jak zprávy přijímat.

1. Datagramy lze přijímat i přes tzv. nepojmenované sockety (nemají přiřazenou adresu a port), přičemž rozhraní, ze kterého zprávu přijímáme, je specifikováno ve volání funkce `recvfrom`.
Pro pojmenování rozhraní využijeme opět strukturu `sockaddr_in`, a to následujícím způsobem:

```
struct sockaddr_in addr;
addr.sin_family = AF_INET;
addr.sin_addr.s_addr = htonl(INADDR_ANY);
addr.sin_port = htons(22222);
```

Všimněte si rozdílu na 3. řádku, kdy tentokrát nespecifikujeme konkrétní adresu, ale pomocí `INADDR_ANY` říkáme, že chceme přijímat zprávy přes libovolné místní síťové rozhraní.

Příjem zprávy je poté realizován následujícím voláním:

```
unsigned char buffer[100];
socklen_t senderlen = sizeof(addr);

ssize_t no_of_received_bytes = recvfrom(fd, buffer, sizeof(buffer), 0, (struct sockaddr *) &addr, &senderlen);
std::string received_message(buffer, buffer + no_of_received_bytes);
```

Funkci jsou předány tyto parametry: deskriptor socketu, buffer pro příjem, délka bufferu, příznaky (flags), struktura s popisem rozhraní, *reference* na velikost této struktury. Vrácen je počet přijatých bytů. 
Volání je obdobné jako v případě `sendto`, všimněte si ale, že tentokrát předáváme referenci na délku struktury, ne přímo délku; to proto, že funkce `recvfrom` do struktury zapisuje údaje o odesilateli zprávy 
a může její délku změnit.

2. Pokud komunikuji opakovaně se stejným odesilatelem, je možné socket tzv. pojmenovat a odesílání realizovat funkcí `recv`, která nevyžaduje strukturu s popisem rozhraní (tato informace bude již předána přes deskriptor socketu).
Způsob pojmenování závisí na tom, zda je vaše aplikace klientem (aktivně se připojuje k jinému socketu) nebo serverem (naopak čeká, než se jiný socket připojí k vašemu). Varianta s klientem byla popsána výše. 
Pro variantu serveru použijeme funkci `bind`:

```
int res = bind(fd, (struct sockaddr *) &addr, sizeof(addr));
```

Můžete si povšimnout, že volání je stejné jako v případě funkce `connect`. V případě úspěchu je vrácena 0, při chybě -1.
  
K příjmu zprávy poté slouží následující volání:
 
```
ssize_t no_of_received_bytes = recv(fd, buffer, sizeof(buffer), 0);
```

Parametry a návratová hodnota jsou analogické s funkcí `recvfrom`.



## Testování komunikace se simulátorem (cca 10 minut)

Jak již bylo řečeno, se simulátorem komunikujeme UDP protokolem pomocí NMEA zpráv. V případě, že simulátor máte spuštěný na stejném stroji jako svůj program (což vřele doporučuji), budete zprávy odesílat na adresu *localhostu* 
(127.0.0.1). Simulátor komunikuje na portech specifikovaných v souboru `config.yaml`, vypíše je do konzole po spuštění. Všimněte se, že příjem a odesílání dat probíhá na jiných portech, budete tedy potřebovat dva sockety.

Základní fungování komunikace můžete ověřit pomocí zprávy `PING,<dalsi-parametry>`. Po přijetí vám simulátor vrátí zprávu `PONG,<dalsi-parametry>`.

Pozn. Simulátor očekává, že všechny příchozí zprávy mají minimálně dva segmenty (tj. minimálně jednu čárku), proto nestačí poslat pouze `PING`.

✅ Na konci cvičení musíte být schopní odeslat do simulátoru zprávu Ping a vypsat do konzole odpověď extrahovanou z NMEA zprávy (tj. bez znaků $, * a checksumu).

Doporučujeme, abyste dbali na kulturu kódu a používali třídy. Komunikace bude fungovat i pokud veškerý potřebný kód zahrnete do souboru `main.cpp`, ale s tímto přístupem by v brzké budoucnosti funkce `main` rychle nabobtnala
a stala se těžko přehlednou. Pokud se budete řídit příklady dobré praxe od počátku projektu, bude vás to na začátku stát více času, ale v pozdějších fázích jej naopak budete šetřit.   

## Bonus: vyčtení dat ze senzorů

Vyzkoušejte další zprávy pro komunikaci se simulátorem, které vrací hodnoty ze senzorů:

✅ Zpráva `SENSOR,<id>` posílá požadavec na měření senzoru číslo `<id>` (indexováno od nuly).

✅ Zprávy `LODO` a `RODO` posílají požadavek na zjištění ujeté vzdálenost levého, resp. pravého motoru.

✅ Ve zbývajícím čase můžete začít pracovat na funkci, která rozdělí příchozí zprávu na segmenty (oddělené čárkou) a převede texty na čísla (v závislosti na typu zprávy).