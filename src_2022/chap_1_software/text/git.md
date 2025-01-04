# Git - Verzovací systému

Git je distribuovaný systém pro verzováni a management zálohování zdrojových kódů. Obecně ale Git funguje dobře pro verzování libovolného textu. Primární motivací k výuce Gitu v rámci tohoto předmětu je fakt, že Git je dnes nejrozšířenějí verzovací systém v konerční svéře a zároveň je na webu dostupná obrovská paleta Git-based online verzovacích služeb.


## Základní terminologie

Vymezme si několik základních pojmů, abychom si rozuměli.

### repozitář (repo)

Sada verzovaných souborů a záznamy o jejich historii. Pokud je repozitář uložen na našem počítači, nazýváme jej lokální repozitář (local repo). Jeli uložen na jiném stroji, hovoříme o vzdáleném repozitáři (remote repo).

### klonování (cloning)
Stažení repozitáře z remote repa. Klonujeme v okamžiku, kdy na lokálním počítači repozitář neexistuje.

### snapshot
Stav repozitáře v konkrétním bodě v historii.

### diff
Rozdíl mezi dvěmi snapshoty. Tedy rozdíl stavu verzovaných souborů.

### commit
Záznam, který obsahuje referenci na předchozí, následujicí snapshot a diff mezi nimi. Zároveň každý commit má svůj unikátní dvaceti bytový hash, který jej jednoznačně identifikuje v rámci repozitáře.

### push
Nahrání nových comitů na remote repo.

### fetch
Stažení commitů z remote repo na lokál. Fetchujeme, pokud na lokále máme repozitář naklonovaný, ale nemáme stažené nejnovější commity.

### větev (branch)
Řetězec na sebe navazujicích commitů. Ze základu má každý repozitáž jednu větev ("master", někdy "main"). Probíha-li však vývoj několika funkcionalit vedle sebe, je možné tyto vývoje rozdělit do zvláštnich větví a připojit je spátky k hlavní větni, až je funkcionalita dokončená.



## Princip fungování

Primární funkcí Gitu je verzování textových souborů. Jedním dechem je potřeba dodta, že Git NENÍ vhodny pro verzování binárních souborů. Vyvíjíme-li tedy program a verzujeme vývoj v Gitu, vždy verzujeme pouuze zdrojové kódy, nikdy ne zkompilované spustitelné soubory (binárky).

Zároveň Git umožňuje velmi efektivní spolupráci mnoha lidí na stejném projektu (repozitáři). Vývojáři mohou pracovat společně, případně každý na separátním branchi. Důležité pravidlo však je, že dva lidé nesmí přepsat stejný řádek kódu ve dvou různých commitech. To způsobi tzv. konflikt. Obecné doporučení je, aby dva lidé neměnili stejný soubor.

Ve srovnání s SVN je ale Git tzv. decentralizovaný systém. To znamená, že v systému repozitářů neexistuje žaden nadřazeny, důležitější repozitář, či něco ve smyslu centrálního serveru. Všechny repozitáře mají stejnou funkcionalitu a jsou schopny udržovat kompletní historii celého repozitáře a ponohodnotně komunikovat se všemi ostatními klony. Praxe je však taková, že obvykle existuje repozitář, který funguje jako centrální místo pro výměnu commitů mezi vývojáři. Takový repozitář se obvykle jmenuje "origin". Důležité však je, že kterýkolik repozitář, si může z originu stáhnout kompletní historii a tak v případě selhání originu nedojde ke ztrátě dat, protože každý vývojář může mít jeho plnohodnotnou kopii na svém počítačí.

Obvykla práce s Gitem vypadá následovně:

 - Na serveru vytvoříme repozitář projektu.
 - vývojáři si naklonujou repozitář na lokální počítače. Z jejich pohledu loklálních repozitářů je server tzv "origin".
 - vývojáři na lokálních počítačích vytváří kód a commitujou.
 - na konci dne každý vývojáž pushne (nahraje) své denní commity na origin.
 - na druhý den ráno si každý fetchne (stáhne) commity kolegů z dne předchozího.

## Instalace Gitu na Linuxu

V případě, že pracujeme na distribuci Debian, Git nainstalujeme následovně:
```
sudo apt install git
```
nebo
```
sudo snap install git
```


## Přehled příkazů

### git init

Inicializace repozitře. Z obvyklé složky v souborovém systému vytvořím repozitář.

Repozitář se od obyčejné složky liší tím, že v sobě obsahuje skrytou složku s názvem .git a ta obsahuje historii repozitáře.

```
git init     ...      inicializuje repozitář
```

### git add

Příkaz přidává změny vytvořené od posledního commitu do tzv. indexu. Index je soubor změn, které budou součástí nejbližšího commitu. Díky mezistupni index je možné commitnout jen některé změny, které jsme od posledního commitu vytvořili.

```
git add myfile.txt     ...      přidá do indexu změny provedené nad souborem myfile.txt
git add .              ...      přidá do indexu všechny aktuální změny
```

### git commit

Vytvoř nový commit, který je odvozený od posledního commitu v současné větví, a zahrni do commitu změny (diffy), které jsou v indexu.

```
git commit -m "komentář k danému commitu"     ...      vytvoří nový commit v rámci větve, ve které se nacházíme
```

### git checkout

Příkaz slouží k přecházení mezi snapshoty.

```
git checkout .          ...     vrať větev do stavu posledního commitu (zahoď všechny do té doby vytvořené změny)
git checkout abcdef     ...     přepni mě do stavu, který vznikl po commitu s hexadecimálním označením abcdef
git checkout master     ...     přepni mě do stavu posledního dostupného commitu na větvi master
```

### git clone

Příkaz vytvoří klon vzdáleného repozitáře na lokále. Klonujeme-li, není potřeba inicializovat repozitář pomocí git init. Metadata repozitáře se stáhnou automaticky s obsahem.

```
git clone https://adresa_vzdaleneho_repozitare.git     ...      vytvoří klon daného repozitáře na lokálním stroji
```

### git remote

Příkaz vytvoří klon vzdáleného repozitáře na lokále. Klonujeme-li, není potřeba inicializovat repozitář pomocí git init. Metadata repozitáře se stáhnou automaticky s obsahem.

```
git remote -v                                            ...      vypíše konfiguraci vzdálených repozitářů
git remote add origin https://adresa_repozitare.git      ...      přidá do lokálního repozitáře alias vzdáleného repozitáře s danou adresou
git remote remove origin                                 ...      smaže alias origin na vzdálený repozitář  
```

### git push

Odešle nové commity vytvořené na lokále na vzdálený repozitář.

```
git push origin master     ...     odešle na mastera nové commity vytvořené v rámci větvě (branche) master
```

### git fetch

Stáhne z remotu commity do lokálního repozitáře. Stažené komity se ale nestanou součástí větve. Změny zůstanou pouze zapsány v paměti.

```
git fetch origin           ...     stáhne nové commity ve všech větvích z originu na lokál
git fetch origin master    ...     stáhne nové commity pouze pro větev master z originu na lokál
```

### git merge

Na aktuální větví vytvoří nový commit tak, že spojí naagregované diffy dvou různých větví. Tím pádem se v součacné větvi objeví všechny změny, které byly vytvořeny v jiné větví. Větve se tak spojí.

```
git merge cool_branch        ...      na současné větví vytvoří nový commit, který obsahuje všechny změny větve cool_branch
```

### git pull

Kombinace příkazů git fetch a git merge. Obvykle se používá při stažení změn ze serveru. Příkaz nejprve stáhne commity z vzdáleného repozítáře (provede fetch) a následně je připojí do současné větve (provede merge).

```
git pull origin master        ...      stáhne z originu nové commity na větvi master a přidá je do lokální větve master
```

### git diff

Vytiskne rozdíl stavu repozitáře mezi dvěma commity.

```
git diff abcdef 012345        ...      vytiskne rozdíl mezi commity, které jsou identifikovány hexadecimálními hashy abcdef a 012345
```

### git status

Zobrazí současný stav změn provedených od posledního commitu, včetně zobrazení změn, které jsou již přidány do indexu.

```
git status        ...      vytiskne současný stav změn
```

### git log

Vytiskne chronologicky výpis commitů spolu s jejich metadaty (časem vytvoření commitu, popiskem, identifikačním hashem, atd.)

```
git log        ...      vytiskne historii současné větve
```

### git stash

Slouží pro ukládání a načítání změn do zásobníku. Vhodné například, když si všimnete, že píšete kód na jiné větvi, než byl záměr. Pomocí git stash uložíte změny do zásobníku, přepnete se na jinou větev a změny si ze zásobníku vytáhnete.

```
git stash        ...      Uloží změny provedené od posledního commitu do zásobníku a vrátí větev do stavu, v jakém byla po posledním commitu (jako by jste zdrojový kód nikdy nenapsali).
git stash pop      ...      Vytáhne změny uložené ze zásobníku a aplikuje je na současný stav (jako by jste kód právě ručně napsali).
```

## Cvičení

Několik scénářů se kterými se můžete během vývoje software potkat. Vyzkoušejte si je opakovaně, aby jste si vryli do paměti způsob práce s Gitem. Zároveň doporučuji si příklady nejprvé projít v příkazové řádce, aby jste chápali zůpsob, jakým Git funguje na nejnižší vrstvě a následně si cvičení absolvovali i v grafickém rozhraní Vašecho vývojového prostředí.

### Základní obsluha

  - Vytvořte si repozitář.
  - Vytvořte v něm 2 textové soubory a do každého napište několik řádků.
  - Přídejte provedené změny do indexu a následně změny commitněte.
  - Nyní zeditujte jeden soubor a opět jej commitněte.
  - Zeditujte druhý soubor a změny commitněte.
  - Vytvořte si účet na [GitHubu](https://github.com), a založte si tam nový repozitář.
  - Přidejte vzdálený repozitář jako "origin" do lokálního repozitáře a pushněte změny na origin.
  - Ve vebovém prostředí ověřte obsah repozitáře.
  - Na jiním místě v počítači, nebo na jiném počítači si naklonujte právě pushnutý repozitář.
  - V novém klonu proveďte změnu a commitněte jí pushnete na origin.
  - V původní složce pullněte nové commity z originu.
  - Příkazem git log si prohlédněte historii.

### Konflikt

  Příklad, co se stane, když dva vývojáří změní tentýž kód.

  - Po vzoru předchozího cvičení si vytvořte na počítači, případně na dvou počítačích dvě kopie repozitáře, který bude mít společný origin na webu.
  - V prvním klonu upravte konkrétní řádek souboru, commitněte a pushněte.
  - V druhém klonu upravte tentýž řádek, commitněne a pushněte (push zahlásí chybu).
  - Nyní jsme si vyrobili konflikt. Ve stejném bodě v historii větve repozitáře proběhly dvě změny,které se navzájem vylučují (tzv. conflict).
  - Konflikt opravíme tak, že v druhém klonu, který nedokázal pushnout provedeme pull z originu.
  - Nyní nahlédněme do souboru, který obsahuje konflikt. Konflikt je označen speciální syntaxí <<<<<<< lokalni_zmena ======= zmena_z_originu >>>>>>>. Vyberte verzi, která je žádoucí a speciální syntaxi odstraňte. Tím je konflikt vyřešen.
  - Zavolejte příkaz git commit bez dalších parametrů a provede se commit s automatickým popiskem, že se jedná o řešení konflitku.
  - Pushněte nový commit na origin a poté pullněte jej v původním repozitáři.
  - Příkazem git log si prohlédněte historii.

## Doporučené materiály

[Tutoriál Atlassianu](https://atlassian.com)

[Oficiální dokumentace Gitu](https://git.com)

[Užitečný rádce při potížích s Gitem (cs)](https://ohshitgit.com/)
