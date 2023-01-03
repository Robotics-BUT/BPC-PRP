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

