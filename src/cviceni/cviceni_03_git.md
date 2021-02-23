# GIT, simulátor 

Cvičící: Ing. Adam Ligocki

## GIT (cca 1 hod)

Zaregistrujte se na studentském [GitLab serveru](https://student.robotika.ceitec.vutbr.cz/)

Tento server bude po zbytek kurzu BPC-PRP Vás tzv. "origin", tedy vzdálené repozitář.

Vyučující mají náhled do všech Vaších repozitářů, včetně jejich historii a mohou přůběžně sledovat Vaší práci, případně kdo, kdy a jak intenzivně "commitoval".

Na serveru si vytvořte repozitář, ve kterém budete udržovat kód po zbytek kurzu.

V rámci týmu si vyzkoušejte následujicí cvičení.

✅ Jeden člen vytvoří repozitář na serveru.

✅ Všichni členové si naklonují repozitár k sobě na PC.

✅ Jeden člen týmu vytvoří na lokále hello world program a "commitne" jej, a "pushne" jej na origin.

✅ Zbytek týmu si "pullne" změny k sobě na lokál.

✅ Dva členové týmu si skusí vyrobit konflikt tak, že zároveň změní stejný řádek v kódu a pokusí se jej pushnout na server. Druhému v pořádí, který se pokusí o push, git zahlásí chybu.

✅ Nešťastný člen týmu, který má nahlášený konflikt, se pokusí o jeho "fix" a opravenou verzi pushne na origin.

✅ Všichni členové týmu si pullnou aktuální verzi repozitáře. A každý člen týmu si vyrobí vlastní .h soubor a v něm funkci, která vytiskne jeho jméno. Všichni pushnou změny na server.

✅ Jeden člen týmu si pullne nově vytvořené .h soubory a upraví hello world program tak aby využíval všechen nově vytvořený kód. Změny pushne na origin.

✅ Všichni si pullnou aktuální stav repozitáře.

## Simulátor (cca 30 min)

Ze [stránek kurzu](https://github.com/Robotics-BUT/BPC-PRP) si pullněte aktuální stav repozitáře.

Repozitář obsahuje ve složce "bin/" skompilovanou binárku simulátoru nazvanou "simulator". Pokud máte korektně nainstalovaný ROS, otevřete si 3 terminály

<details>
    <summary>Tip pro práci s více terminály</summary>
<br>
Pro práci s více okny terminálů je dobré si nainstalovat pomocí balíčkovacího manažeru program "terminator". Ten Vám umožní v jednom okně mít otevřených více terminálu.
</details>
<br>

<details>
    <summary>Vím jak se pracuje s balíčkovacím manažerem, jen potřebuji osvěžit paměť.</summary>
<br>
sudo apt install terminator
</details>
<br>

Ve složce "resources/" naleznete soubor config.yaml. Z něj si bude simulátor načítat hodnoty pro Vaší simulaci, mezi jinými nastavení síťové komunikace, simulované rozměry robota, parametry podvozku, rozmístění snímačů, a hlavně cestu k mapě, se kterou bude simulátor pracovat. Tu si upravte pro svůj vlastní souborový systém.

Dále se ve slořce serources nachází také jedna vzorová mapa. Nahlédněté do ní a všiměnte si struktury YAML dat. Nachází se zde informace o šířce čáry a následně je zde pole úseček definovaných vždy [bod1_x, bod1_y, bod2_x, bod2_y]. V budoucnu si budete vytvářet taky vlastní mapy.

✅ Nastavte si v souboru config.yaml validní cestu k mapě route_1.yaml.

✅ Po změně obsahu souboru si vyzkoušejte validitu yamlu souboru v [online nástroji](http://www.yamllint.com/).

Nyní zbývá si vyzkoušet oživit celý systém.

✅ V 1. terminále zapněte roscore

✅ V 2. terminále spusťte binárku simulátoru, jako argument programu přidejte absolutní cestu ke konfiguračnímu souboru, který jste dříve editovali.

✅ V 3. terminále zapněte rviz a přidejte si vizualizaci topicků 

### Konfigurace RVizu

![empty RViz](../images/rviz_1.jpg)

Pokud běží simulátor, v levé dolní sekci tlačítkem "ADD" otevřete okno pro přidání vizualizací.

![RViz add topic by display type](../images/rviz_2.jpg)

V záložce "By Display Type" vyberte položku "TF" a dvojitým polikem jí přidejte do zobrazení. Dále v levé horní části v záložce "Global Options" nastavte položku "Fixed Frame" na hodnotu "origin". Tímto jsme RVizu řekli, že má zobrazovat souřadný systém "origin" a vůči němu vykreslovat všechny ostatní souřadné systémy, se kterými simulátor pracuje.

Opětovně otevřete okno pro přidání vizualizací a přejděte do záložky "By Topic". 

![RViz add topic by topic](../images/rviz_3.jpg)

Postupně si přidejte všenchy vizualizace markerů pod topicky:
  - /bpc_prp/line
  - /bpc_prp/robot_body
  - /bpc_prp/sensors
  - /bpc_prp/wheels/left
  - /bpc_prp/wheels/right

Ihned po přidání nové vizualizace si jí můžete přejmenovat použitím tlačítka "Rename" na penelu vlevo dole.

✅ Výsledek by měl vypadat následovně:

![RViz Ready](../images/prp_1.jpg)

Současné nastavení RVizu uložíte klávesovou zkratkou CTRL+S

✅ Prozkoumejte další možnosti nastavení vizualizací v RVizu, jako velikost mříždy, průhlednost, barva pozadí, atd.
