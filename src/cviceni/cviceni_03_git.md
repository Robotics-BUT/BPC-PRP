# GIT, simulátor 

Cvičící: Ing. Adam Ligocki, Ph.D., Ing. Tomáš Horeličan

## GIT (cca 1 hod)

Každý student se nejprve zaregistruje na studentském [GitLab serveru](https://student.robotika.ceitec.vutbr.cz/). Doporučuji také verifikovat emailovou adresu, bude se Vám to hodiť při některých operacích.

Tento server bude po zbytek kurzu BPC-PRP Vás tzv. `origin`, tedy vzdálené repozitář.

Vyučující mají náhled do všech Vaších repozitářů, včetně jejich historii a mohou přůběžně sledovat Vaší práci, případně kdo, kdy a jak intenzivně "commitoval".

Na serveru si můžete vytvořit repozitář, ve kterém budete následne udržovat kód po zbytek kurzu.
Nejprve je ale potřebné zajistit by Váš počítač měl k tomuto repozitáři přístup a mohl si z něj `"pullovat"` nové změny a `"pushovat"` na něj své lokálni změny.
Pro toto budete využívat SSH (Secure SHell) klíče.

### Vytvoření SSH klíčů pro přístup k repozitářům

Pár klíčů vygenerujte pouze jeden krát na jednom počítači.

Klíče se ukádají lokálně na počítači a jsou pak spárované s účtem na gitlabu. Pokud tedy pracujete na školním PC, vyberte si jednoho člena týmu, který svůj účet na gitlabu s tímto párem následně v dalších krocích propojí.

Pokud pracujete na svém vlastním počítači postupujte s vaším vlastním gitlab účtem.

> **_POZNÁMKA:_**  Kdykoliv uvidíte špicaté závorky s nějakým textem `<text>`, znamená to, že máte tento text včetě závorek něčím nahradit.


1. Vygenerujte si klíče pomocí příkazu `ssh-keygen -t ed25519 -C "<comment>"`
<details>
    <summary>Tip!</summary>
Jako koment doporučuji zadat emailovou adresu, kterou máte spojenou s vaším gitlab účtem. Snadněji pak spoznáte k čemu klíče patří. 
</details>

2. Poté (po vyzváni) zadejte adresářovou cestu, kam se mají klíče uložit. Doporučuji ponechat "defaultni" cestu
ale upravit jméno, například takto: `/home/<user-name>/.ssh/id_ed25519_gitlab_uamt_<group-number>`.
<details>
    <summary>Tip!</summary>

Tá poslední část (tedy `id_ed25519_gitlab_uamt_<group-number>`) je jméno, se kterým se klíče vygenerují.
Můžete ho nahradit čím chcete ale musíte to samé pak taky použít v dalších krocích. Tady při psaní nepoužívejte šípky (nebudou Vám fungovat).
</details>

3. Poté (po vyzváni), pokud chcete, zadejte heslo pro zabezpečení klíče. Jinak jen potvrďte stisknutím `Enter`.

4. Otevřete soubor `/home/<user-name>/.ssh/config` (například v nano) a přidejte následujíci:
```
# Student gitlab instance at student.robotika.ceitec.vutbr.cz
Host student.robotika.ceitec.vutbr.cz
  PreferredAuthentications publickey
  IdentityFile ~/.ssh/id_ed25519_gitlab_uamt_<group-number>
```
<details>
    <summary>Tip!</summary>
Pokud jste si zvolili jiné jméno klíče, musíte ho stejně zadat i zde.
</details>

5. Zkopírujte obsah souboru `/home/<user-name>/.ssh/id_ed25519_gitlab_uamt_<group-number>.pub`
<details>
    <summary>Tip!</summary>
Pokud jste si zvolili jiné jméno klíče, musíte ho opět stejně zadat i zde.
</details>

> **POZOR!** Kopírujte jen ze souboru s příponou `.pub`!
> Pokud má obsah více než jeden řádek a nebo začíná řádkem `-----BEGIN OPENSSH PRIVATE KEY-----`, pravdepodobne kopírujete z nesprávneho souboru!

6. V gitlabu přejdete do `Preferences > SSH Keys`, vložte zkopírovaný obsah, pojmenujte tady svůj klíč (např. "Notebook-Ubuntu") a potvrďte stisknutím `Add key`.

7. Teď by ste meli mít z Vašeho počítače přístup ke všem Vaším (budoucím) repozitářům.
Zároveň ste se naučili jak můžete v budoucnu spravovat více různych klíčů pro různe git servery.

<sup>*</sup>Detailnejší návody najdete také na stránkach [GitLabu](https://student.robotika.ceitec.vutbr.cz/help/ssh/index#generate-an-ssh-key-pair) nebo [GitHubu](https://docs.github.com/en/authentication/connecting-to-github-with-ssh). <br />

V rámci týmu si teď vyzkoušejte následujicí cvičení:

### Základní nastavení GITu

✅ Jeden člen vytvoří repozitář na serveru přímo v gitlabu.

✅ Všichni členové si naklonují repozitár k sobě na PC (na školním PC klonuje, zamozřejmě, jen jeden užívatel).

<details>
    <summary>Jak naklonovat vdálený repozitář?</summary>

`git clone <ssh-link-to-repo.git>`
</details>

✅ Každý si ve svém naklonovaném repozitáři nastavi jméno a emailovou adresu, se kterýma bude do repozitáře commitovat.
Na školním PC to bude pouze ten člen, který si pro ňej pároval i SSH klíče.

<details>
    <summary>Jak nastavit jméno a email?</summary>

    git config user.name "<committing-user-name>"
    git config user.email <committing-user-email>
Doporučuji nastavit stejnou emailovou adresu jakou máte ve svém gitlab účtu.
</details>

<details>
    <summary>Tip!</summary>

Podívejte se jak vypadá soubor `.git/config`. Nastavení můžete měnit i editováním přímo v tomto souboru.
</details>

### Základní GIT workflow (Bash)

✅ Jeden člen týmu vytvoří na lokále hello world program, `"commitne"` jej a `"pushne"` jej na `origin`.

<details>
    <summary>Jak se commituje a pushuje?</summary>

    git add <files-to-add>
    git commit -m "my first commit"
    git push
</details>

<details>
    <summary>Tip!</summary>

Pomocí příkazů `git status` a `git diff` sledujte průběžně změny v repozitáři.
</details>

✅ Zbytek týmu si `"pullne"` změny k sobě na lokál.

<details>
    <summary>Jak se pulluje?</summary>

    git pull
</details>

✅ Dva členové týmu si skusí vyrobit konflikt tak, že zároveň změní stejný řádek v kódu a pokusí se jej `"pushnout"` na server.
Druhému v pořádí, který se pokusí o `push`, git zahlásí chybu.

✅ Nešťastný člen týmu, který má nahlášený konflikt, se pokusí o jeho `fix` a opravenou verzi `"pushne"` na origin.

<details>
    <summary>Jak vyřešit konflikt?</summary>

Nejprve si `"pullnete"` aktualni verzi z `"originu"`. Podívejte se co se stalo s Vaším kódem.
Upravte ho tak aby v nem bylo vše co požadujete. Opět `"commitnete"` a zkuste `"pushnout"`.
</details>

✅ Všichni členové týmu si `"pullnou"` aktuální verzi repozitáře.

✅ Jeden člen týmu vytvoří nový lokálni `branch`, `"checkoutne"` se do nej a vyrobí si vlastní `.h` soubor a v něm funkci, která vytiskne jeho jméno.
Změny standardně `"commitne"` a pokusí se `"pushnout"`.

<details>
    <summary>Jak udělat branch a checkoutnout?</summary>

    git branch <name-of-the-new-branch>
    git checkout <name-of-an-existing-branch>
</details>

<details>
    <summary>Tip!</summary>

Co se stalo když ste se pokusili `"pushnout"` nový branch? Co Vám poradil `git`?
</details>

✅ Všichni si `"pullnou"` nové zmeny k sobě a `"checkoutnou"` se na nový branch v `"originu"`.
Každý člen týmu si pak vyrobí vlastní `.h` soubor a v něm taky funkci, která vytiskne jeho jméno.
Všichni `"pushnou"` své změny na server.

<details>
    <summary>Tip!</summary>

Při `"checkoutu"` se nažte využívat doplňování pomocí `Tab`.
</details>

✅ Jeden člen týmu si `"pullne"` nově vytvořené `.h` soubory a upraví hello world program tak aby využíval všechen nově vytvořený kód.
Změny pushne na `origin` a v gilabu vytvoří nový `Merge Request` z nového `"branchu"` do `"branchu" main`.

> **POZNÁMKA:** Pokud ste v gitlabu ve svém repozitáři a vlevo nevidíte možnost `Merge requests`, musíte si je v nastaveních povolit.
> Přejdete do (vlevo) `Settings > General`, v části `Visibility, project features, permissions` klikněte na `Expand` a povolte `Merge requests`.
<details>
    <summary>Tip!</summary>

Velmi užitečným nástrojem jsou i takzvané `Issues`, které povolíte v týchž nastaveních jako `Merge requesty`.
Pomocí nich můžou developeři zaznamenávat chyby, které je třeba opravit nebo diskutovat nové "featury".
`Issues` v tomto cviku používat nebudete ale můžete se s nima pohrát samostatně a také je můžete pak využívat při práci na projektu.
</details>

✅ Nyní si všichni skuste pohrát s `MR ("Merge Requestem")` v gitlabu.
Přidávejte komentáře a zkuste v nich také vkládat odkazy na `"commity"` a části kódu.

✅ Jeden člen týmu nakonec `Merge Request` potvrdí.

✅ Všichni si `"pullnou"` aktuální stav repozitáře.

<details>
    <summary>Tip!</summary>

Zkuste se pomocí příkazu `git log` podívat na aktuálni historii `"commitů"`.
Pokud je `log` příliž dlouhej, vrátite se zpět stisknutím klávesy `q`.
</details>

### Integrace GITu v Clion IDE

Všechny operace, které ste právě dělali se dají mnohem snadněji a efektivněji udělat i v Clionu.
Dálší část budete delat už přímo v Clionu.

✅ Všichni si v Clionu otevřou složku se svým repozitářem.

<details>
    <summary>Tip!</summary>

Když si vlevo dolu rozklikněte modul `Git` uvidíte přehledne celou vaší historii `"commitů"` a také všechny `"branche"`, které git právě sleduje.
`"Checkoutovat"` můžete (pravé tlačítko myši) nejen do branchů ale také do jekéhokoliv `"commitu"`.
</details>

✅ Všichni si `"pullnou"` aktuální stav repozitáře pomocí modré sípky napravo nahoře (`Update project...`) a `"checkoutnou"` do lokálního `"branchu" main`.

✅ Jeden člen týmu vytvoří nový soubor s názvem `README.md` (pokud takový soubor ješte neexistuje), jeden soubor s názvem `.gitignore` (včetně tečky) a jeden textový soubor s názvem `random_file.txt`.

✅ Do souboru `README.md` společně doplňte stručný popis repozitáře pomocí `markdown` syntaxe.

<details>
    <summary>Jak vypadá markdown?</summary>

```
# Main title
This repository contains source files for the BPC-PRP cource ...

# Table of contents
* [Main title](#main-title)
* [Subtitle](#this-is-a-subtitle)
* [SubSubtitle](#this-is-a-sub-subtitle)
* [Itemizing](#itemizing)
* [Enumerating](#enumeration)
* [Tables](#tables)

## This is a subtitle
Our solution for winning the competition was ...

### This is a sub-subtitle
**BALD TEXT**: Our C++ class `short name in code format` uses the ... algorithm to solve the ...

    Long text displayed in code format.
    It contains excerptions of our code
    or other stuff.

[This](https://google.com) is a web page link.

### Itemizing
* Line 1.
* Line 2.

### Enumeration
1. Step one.
2. Step two.

### Tables
| Col 1  | Col 2  | Col 3  | Col 4  |
|--------|--------|--------|--------|
| Row 11 | Row 12 | Row 13 | Row 12 |
| Row 21 | Row 22 | Row 23 | Row 12 |
```
</details>

✅ Člen týmu, který soubor právě edituje klikne na modul `Commit` (vlevo), kde uvidí všechny aktuálne rozpracované soubory.
Po rozkliknutí souboru také přehledně uvidí všechny změny, které v tom souboru udělal.

✅ V části `Untracked files` uvidíte všechny nové soubory, které ste právě vytvořili.

✅ Člen týmu, který vytvářel soubory si otevře `.gitignore` a do něj přímo napíše název posledně vytvořeného textového souboru (tedy `random_file.txt`).
Pak se pak vráti do modulu `Commit` (vlevo).
<details>
    <summary>Tip!</summary>

Sledujte co se stalo v části `Untracked files`. Pokud tam zůstali ješte nejaké jiné neznáme soubory, můžete je stejně přidat do `.gitignore`.
Typicky to bývají soubory z `cmake` nebo od Clionu. Když do `.gitignore` vložíte `cmake*` bude git ignorovat všechno, co začína textem cmake (včetně složek).
</details>
<details>
    <summary>Pro Tip!</summary>

Podobné chování dosáhnete i přes soubor `.git/info/exclude`.
Rozdíl je v tom, že ten je přístupnej vždy jen lokálne pro Vás a to co do něj vložíte nikto jiný neuvidí a tedy ani u nikoho jiného fungovat nebude.
</details>

✅ Do okna dolu vloží info o `"commitu"` (takzvaný `commit message`), `"pushne"` stlačením `Commit and Push...` a potvrdí.

✅ Dva členové se opět pokusí vytvořit konflikt editovaním stejného řádku v jednom souboru.

✅ Nešťastný člen týmu opět dostane varováni o konfliktu a stačením `Rebase` se ho pokusí řešit.

✅ Řešitel konfliktu pak vybere možnost `Merge` a sloučí změny tak aby soubor obsahoval změny od obou členů.
Stlačením `Push...` (zelená šipka vpravo nahoře) odešle finálni verzi do `"originu"`.

Nový `branch` v Clionu vytvoříte v modulu `Git` stlačením ikonky `plus` vlevo.
Můžete také udělat nový `branch`, který přímo navazuje na jiný než ten aktuálni a to kliknutím pravým tlačítkem miší a výběrem `New branch from selected...`.

### Podepisování commitů pomocí GPG klíčů (optional)

Pokud chcete svoje `"commity"` podepisovat jako "ověřený" užívatel, je nutné mít verifikovaný email ve Vašem gitlab účtu.

1. Klíče vygenerujete pomocí příkazu `gpg --full-gen-key`, případne `gpg --gen-key`.

<details>
    <summary>Tip!</summary>

Balíček `gpg` nainstalujete standardně příkazem `sudo apt install gpg`
</details>

2. Následně jen stiskňete klávesu `Enter` (vybírame "default" možnost `RSA and RSA`).

3. Po vyzváni pak zadejte velikost klíče 4096.

4. V dalším kroku můžete opět jen stisknout `Enter` (vaším klíčem nikdy neskončí platnost) a pak potvrdíte zadaním `y`.

5. Teď budete postupně zadávat údaje o účtu, se kterým budou klíče asociovány:

    5.1 Jméno doporučuji zvolit stejné jako máte nastavené v účtu na gitlabu.

    5.2 Email musí být stejný jako máte ve svém účtu na gitlabu a musí být verifikován. Email musí být také stejný jako ste si nastavili v konfiguraci gitu (email `"commitujicího"`). Jinak Vám gitlab bude zobrazovat `"commity"` jako neověřené.

    5.3 Jako poznámku můžete zadat třeba "Gitlab VUT" abyste pak klíč rozeznali.

6. Vyberte si heslo (tento krok nezle přeskočit).

7. Když se Vaše klíče vygenerují, pomocí příkazu `gpg --list-secret-keys --keyid-format LONG <your-email>` si zobrazíte váš privátni klíč.

8. Skopírujte sekvencií znaků (ID Vašeho klíče), která následuje hned po `sec rsa4096/` v prvním řádku.

9. Pomocí příkazu `gpg --armor --export <skopirovane-ID-klice>` vygenerujete veřejnou část vašeho páru klíčů.

10. Skopírujte všechno od `-----BEGIN PGP PUBLIC KEY BLOCK-----` až po `-----END PGP PUBLIC KEY BLOCK-----`, včetně obou.

11. V gitlabu přejdete do `Preferences > GPG Keys`,  vložte skopírovaný obsah a potvrďte stlačením `Add key`.

12. V Clionu (kde máte otevřený svůj repozitář) přejdete do `Settings > Git > Configure GPG key` a vyberte se seznamu Váš klíč.

<details>
    <summary>Tip!</summary>

Podívejte co se změnilo v souboru `.git/config`. To samé by se dalo opět dosáhnout editováním tohto souboru, avšak v Clionu je to mnohem jedndušší.
</details>

13. Jeden člen týmu zkusí cvične udělat nejaký `commit` (měl by dostat výzvu na zadáni hesla) a `"pushnout"`.

14. Pokud všechno probehlo správně měli by ste v gitlabu v historii `"commitů"` vidět svůj commit označený jako `verified`.

<details>
    <summary>Tip!</summary>

Pro `"commitování"` s podpisem přes `Bash` se používa `git commit -S -m "my signed commit"`
</details>

<sup>*</sup>Detailnejší návody najdete také na stránkach [GitLabu](https://git.robotika.ceitec.vutbr.cz:2223/help/user/project/repository/gpg_signed_commits/index.md) nebo [GitHubu](https://docs.github.com/en/authentication/managing-commit-signature-verification). <br />


## Simulátor (cca 30 min)

Ze [stránek kurzu](https://github.com/Robotics-BUT/BPC-PRP) si `"naclonujte"` aktuální stav repozitáře.

Repozitář obsahuje ve složce `bin/` skompilovanou binárku simulátoru nazvanou "simulator". Pokud máte korektně nainstalovaný ROS, otevřete si 3 terminály.
Pokud ne, nainstalujte si ho podle návodu na [stránkach kurzu](https://robotics-but.github.io/BPC-PRP/chap_1_software/text/ros.html) nebo přímo [stránkach rosu](http://wiki.ros.org/noetic/Installation/Ubuntu).

<details>
    <summary>Tip pro práci s více terminály</summary>

Pro práci s více okny terminálů je dobré si nainstalovat pomocí balíčkovacího manažeru program "terminator" nebo "tilix". Ten Vám umožní v jednom okně mít otevřených více terminálu.
</details>

<details>
    <summary>Vím jak se pracuje s balíčkovacím manažerem, jen potřebuji osvěžit paměť.</summary>

`sudo apt install <package-name>`
</details>

Ve složce `resources/` naleznete soubor `config.yaml`. Z něj si bude simulátor načítat hodnoty pro Vaší simulaci, mezi jinými nastavení síťové komunikace, simulované rozměry robota, parametry podvozku, rozmístění snímačů, a hlavně cestu k mapě, se kterou bude simulátor pracovat. Tu si upravte pro svůj vlastní souborový systém.

Dále se ve složce `resources/` nachází také jedna vzorová mapa. Nahlédněté do ní a všiměnte si struktury YAML dat. Nachází se zde informace o šířce čáry a následně je zde pole úseček definovaných vždy `[bod1_x, bod1_y, bod2_x, bod2_y]`. V budoucnu si budete vytvářet taky vlastní mapy.
<details>
    <summary>Tip!</summary>

Zkuste se zamyslet nad tím jak si zjednodušit tvorbu map. Můžete např. využít nějaký grafický nástroj, soubor si vyexportovat do nějakého vhodného formátu a pak prekonvertovat do `.yaml`. Řešení je na Vás.
</details>

✅ Nastavte si v souboru `config.yaml` validní cestu k mapě `route_1.yaml`.

✅ Po změně obsahu souboru si vyzkoušejte validitu `.yaml` souboru v [online nástroji](http://www.yamllint.com/).

Nyní zbývá si vyzkoušet oživit celý systém.

> **POZNÁMKA:** Pokud nemáte ve svém `/home/<user-name>/.bashrc` souboru přidán příkaz `source /opt/ros/noetic/setup.bash` budete muset v každém novém okně vždy tento příkaz zavolat, jinak Váš počítač nepozná, že máte nainstalovaný ROS. 

✅ V 1. terminále zapněte `roscore`

✅ V 2. terminále spusťte binárku simulátoru, jako argument programu přidejte absolutní cestu ke konfiguračnímu souboru, který jste dříve editovali.

✅ V 3. terminále zapněte rviz a přidejte si vizualizaci topicků 

### Konfigurace RVizu

![empty RViz](../images/rviz_1.jpg)

Pokud běží simulátor, v levé dolní sekci tlačítkem `Add` otevřete okno pro přidání vizualizací.

![RViz add topic by display type](../images/rviz_2.jpg)

V záložce `By Display Type` vyberte položku `TF` a dvojitým polikem jí přidejte do zobrazení. Dále v levé horní části v záložce `Global Options` nastavte položku `Fixed Frame` na hodnotu `origin`. Tímto jsme RVizu řekli, že má zobrazovat souřadný systém `origin` a vůči němu vykreslovat všechny ostatní souřadné systémy, se kterými simulátor pracuje.

Opětovně otevřete okno pro přidání vizualizací a přejděte do záložky `By Topic`. 

![RViz add topic by topic](../images/rviz_3.jpg)

Postupně si přidejte všenchy vizualizace markerů pod topicky (v obdélniku `Display Name` můžete vizualizaci topicu pojmenovat):
  - /bpc_prp/line
  - /bpc_prp/robot_body
  - /bpc_prp/sensors
  - /bpc_prp/wheels/left
  - /bpc_prp/wheels/right

Ihned po přidání nové vizualizace si jí můžete také přejmenovat použitím tlačítka `Rename` na penelu vlevo dole.

✅ Výsledek by měl vypadat následovně:

![RViz Ready](../images/prp_1.jpg)

Současné nastavení RVizu uložíte klávesovou zkratkou CTRL+S

✅ Prozkoumejte další možnosti nastavení vizualizací v RVizu, jako velikost mříždy, průhlednost, barva pozadí, atd.
