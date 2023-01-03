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