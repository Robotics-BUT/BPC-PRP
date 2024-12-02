# Linux a příkazová řádka (terminál)

Tuto kapitolu berte jako rozcestník pro práci v operačním systému Linux.

Není potřeba znát všechny příkazy, včetně jejich parametrů z paměti, avšak měli by jste mít o nich povědomí a případně na internetu, či v Linux manuále být schopni dohledat jejich přesný způsob použití.

## Přehled příkazů v terminále

### příkaz

vysvětlení funkce

```
příklad použití      ...      vysvětlení
```
### ls - (list)

Vypiš všechny soubory a složky (složka je taky typ souboru) v aktuálním bodě file systému.

```
ls
ls -la      ...      vypíše všechny soubory, včetně sktytých a přidá k výpisu detailní informace
```

### cd - (change directory)

Změna složky.

```
cd my_directory      ...      přesun do adresáře s názvem my_directory
cd ~                 ...      návrat do domovské složky (v linuxu nazýváme "home")
cd ..                ...      návrat o adresář výš (dvojtečka)
cd /                 ...      návrat do kořene file systému (v linuxu nazýváme "root")
cd ../my_folder      ...      vrať se o adresář výš a pak se přesuň do adresáře "my_folder"
cd .                 ...      přesuň se do "současného adresáře". V podstatě nic neudělá. Příklad ilustruje existenci symbolu pro aktuální adresář (tečka).
```

### pwd - print working directory

Vypíše aktuální pozici ve file systému.

```
pwd
```

### mkdir - (make directory)

Vytvoření nového adresáře.

```
mkdir my_folder      ...      vytvoří nový adresář s názvem "my_folder"
```

### cp - (copy)

Kopíruje soubor.

```
cp zdrojovy_soubor cilovy_soubor                    ...      vytvoří novou kopii zdrojovy_soubor nazvanou cilovy_soubor
cp ../secter.txt secret_folder/supersecret.txt      ...      vem soubor secret.txt, který se nachází o složku výš a zkopíruj ji do složky secret_folder. Kopie původního souboru se bude jmenovat "supersecret.txt"     
```

### mv - (move)

Příkaz původně pro přesun souboru, hlavně se však využívá pro přejmenováni soborů.

```
mv old_name.txt new_name.html      ...      přejmenuje soubor "old%name.txt" na "new_name.html"
```

### rm - (remove)

Smaže soubor/složku.

```
rm old_file.txt      ...      vymaže soubor "old_file.txt"
rm -r my_folder      ...      smaže složku. Při mazání složky vždy musíme použít modifikátor rekurze (-r). Ten říká, že se má rekurzivně smazat také obsah složky.
```

### chmod - (change mode)

Změní přístupová práva k souboru.

```
chmod 777 /dev/ttyUSB0      ...      umožní všem uživatelům PC přístup na USB port s pořadovým číslem 0. Pro detail fungováni přístupových práv ve file systému viz [7].
```

### sudo

Meta příkaz. Operace provedená v rámci tohoto příkazu bude provedena v režimu oprávnění administrátora operačního systému. Obvykle používáme, když zasahujeme do systémových souborů.

```
sudo mkdir /etc/config      ...      vytvoří složku "config" v systémovém adresáři "/etc".
sudo rm -r /                ...      příkaz rekurzivně smaže celý adresář "root" (v podstatě smaže celý disk včetně OS)
```

### cat - (Concatenate FILE(s) to standard output)

Program vypíše do termínálu obsah souboru.

```
cat ~/my_config_file.txt      ...      vytiskne v terminále obsah zvoleného souboru
```

### man - (manual) referenční manuál operačního systému

Rychlá pomoc když zapomenu, jak pracovat s daným programem

```
man ls      ...      vytiskne v terminále manuál k programu ls
```

### Distribuce Linuxu

Hovoříme-li o Linuxu, máme na mysli jádro operačního systému, které je zpravováno autoritou (tvůrce linuxu Linus Torvards) a ta zajišťuje integritu veškerého kódu, který je do jádra OS zaintegrován.

Nad jádrem operačního systému je však nádstavka balíčkovacích systémů, grafického rozhraní, a dalšího podpůrného software. Distribucí se v Linuxu rozumí balíček těchto podpůrných software, který je dodáván a garantován konkrétní právnickou osobou (komerční subjekt, organizace, atd.).

Často používané distribude:

  Debian - nejrošířenější distribuce Linuxu.

  Ubuntu - derivát Debianu. Nejrozšířenějí distribuce na domácích stanicích.

  Mint - derivát Ubuntu. GUI se blíží Windows.

  RaspberryOS (dříve Raspbian) - derivát Debianu pro Raspberry Pi

  Arch Linux - Distribuce orienovaná na profi uživatele - velká volnost při konfiguraci systému

  Fedora - Alternativa k Debianu.

  ElementaryOS - Minimalistická a rychlá distribuce. Vhodná pro slabé počítače.

  ... mnoho dalších

## Orientace v systému

Souborová struktura Linuxu se odvozuje od tzv. kořene (root), který značíme jako / (vzdáleny ekvivalent C:/ na Windows).

V kořenovém adresáři pak nalezneme složky jako:

  - bin/ - obsahuje binárky (spustitelné soubory operačního systému).
  - home/ - adresář, který obsahuje domovské složky uživatelů.
  - dev/ - obsahuje soubory které mapují fyzické interfacy počítače (interní a externí disky, sériovou linku, usb, síťové rozhraní, atd.).
  - tmp/ - temporary složka. Zde si programy odkládají svá dočasná data.
  - media/ - místo kde se mountují (připojují) externí disky.
  - etc/ - složka obsahuje klíčové systémové soubory.

Po příhlášení se obvykle nacházíte v domovském adresáři, tj na místě /home/<jmeno_uzivatele>/

## Základní programy

### apt

Jedná se o Balíčkovací systém Debianu. Na Linuxu nejčastěji instalumeme programy tak, že si jej stáhneme z veřejného repozitáře, tedy obvykle ověřeného a bezpečného serveru.

Při instalaci musíme vždy disponovat administrátorskými právy.

Příklad instalace Gitu:
```
sudo apt update
sudo apt install git
```

Říkáme: "s administrátorskými právy zavolej program apt a aktualizuj si záznamy o repozitářích", a "s administrátorskými právy zavolej program apt a ten neinstaluje git".

### nano

Editace textu podobná poznámkovému bloku

Ctrl + X - ukončení programu. Program se zeptá, zda má uložit změny

### vim

Profi editace textu. Ovládání programu je však poněkud složitější a vyžaduje pochopení několika principů. Práce s vim je však mnohonásobně rychlejší než s nano. Před použitím doporučuji projít libovolny "vim noob tutoriál" na youtube.

Kdyby se Vám přeci jen povedlo vim zapnout, vězte že jej vypnete kobinací kláves Shift + Z + Z (držíme shift a dvakrát zmáčkneme klávesu 'Z').

### mc

Midnight Commander - grafické prostředí pro pohyb v souborovém systému. Připomíná MS Dos.

Vypíná se klávesou F10.

### curl

Nástroj přikazového řádku pro přenos dat mnoha protokoly. Curl je často používán pro http komunikaci, instalaci programu, či stahování souborů.

### wget

Program pro stahování souborů ze sítě.

Příklad stažení posledního releasu wordpresu:

```
wget https://wordpress.org/latest.zip
```


### Slovo na závěre

Pokud jste v Linuxu nováčky, hlavně se nebote experimentovat. Ideálně si nainstalulte systém do Virtual Boxu a udělejte si zálohu virtuálního disku. Když se Vás podaří systém rozhasit, stačí si natánout backup a jedete dál.
