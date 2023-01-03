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

