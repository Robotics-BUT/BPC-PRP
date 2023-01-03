## Operační systém Linux

Důvodem pro použití Linux ve výuce předmětu je, že momentálně neexistuje distribuce Windows pro výkonnou výpočetní jednotku mobilního robotu, tj RaspberryPi. 
Pravděpodobně ani nikdy existovat nebude. Kdysi existovala Windows10 IoT, takové demo, které se firmě Microsoft nechtělo udržovat, a tak projekt zařízlo 
již v raných fázích vývoje.

Program pro řízení robotu tedy musí být psán tak, aby běžel na OS Linux a studenti musí dokázat takový program vytvořit a odladit. V této a následujících 
kapitolách bude popsána instalace, jednoduchá reference základních příkazů, které je potřeba znát proto abyste dokázali program úspěšně odladit. Pro mnohé 
to bude vcelku nová, mnohde bolestivá zkušenost, ale pokud máme zpětnou vazbu od studentů po pár letech, většinou hodnotí pozitivně a mnohé tento předmět 
navnadil na trvalý přechod od Windows na alternativní operační systém. 

Víme, že většina studentů je cvičena v předchozích předmětech na operačním systému MS Windows. Rýpavý žák může namítnout "však můžeme ladit pod windows 
a jen výsledek po odladění překompilujeme a nahrajeme do raspbery". Ano, to můžete ale pak mohu dodat "děj se vůle boží". Protože je nejspíše pánubohu 
jedno, jak Váš projekt dopadne, určitě vyvstane některý z myriády problémů které programátoři multiplatformních aplikací musí řešit, a program fungovat 
nebude. Z důvodu Vám přiděleného času jsme tedy rozhodli, že nebudeme řešit multiplatformní problémy a tím pádem musíte znát OS Linux.

Covidová léta tento požadavek výrazně umocnily, protože nebylo možné se setkávat a pracovat na reálném HW, práce byla z velké části z domu, bylo rozhodnuto 
že budete pracovat v simulátoru. A protože celý svět je poháněn ROS, byl tento simulátor vytvořen v ROS, který je nativní v Linuxu.

Pro spoustu studentů byla instalace Linuxu ve virtuálním prostředí první volbou. Bohužel výkonnost notebooků mnohdy nedosahovala plnohodnotnému nativnímu 
běhu a simulátor takto spouštěný byl velmi pomalý, prodlužovalo se dopravní zpoždění - a sami víte co dopravní zpoždění s regulací dělá.

Proto doporučuji zvážit, kterou z následujících konfigurací si zvolíte:

 - ***Jsem Linuxák/Linuxačka***
   - Zde nastávají neřešitelné problémy, linuxáci jsou "divní" lidé kteří používají mnoho distribucí které nejsou kompatibilní (viz Distribuce).
   - Pokud nemáte Debian/Ubuntu konkrétní verze, musíte jít cestou virtualizace. Ne, ROS nezrozchodíte na Vaší oblíbené distribuci. Zkoušeli jsme to. Fakt. Ani Fedora. Ani Mint.
   - Pokud jste Linuxák/Linuxačka do morku kostí, nemusíte dál číst a přeskočte na další kapitolu, další text je pro nooby.
   - Nejspíš budete velmi cenní pro ostatní členy týmu :-)

 - ***Multiiboot Windows/Linux, IDE v Linuxu***
   - Velmi rychlé, využíváte prostředků vašeho počítače přímo
   - Nebezpečné vůči windows (při instalaci a používání je potřeba dávat pozor co děláte, můžete přijít o data na Windows, TPM)
   - "Blbě se ten parazit odstraňuje" aneb po skončení semestru je to na kompletní přeinstalaci windows (což stejně doporučuji dělat každý semestr, aspoň otestujete funkčnost zálohování)

 - ***Virtuální Linux na Windows OS, IDE v Linux***
   - Nejpomalejší varianta, IDE je v Javě která je interpretovaná ve virtuálním stroji který běží na virtuálním stroji ... 
   - Musíte mít opravdu rychlý počítač (spíš stolní, toto není nic pro levné notebooky) a hodně paměti (u Win11 tak 32GiB, spíš víc, u Win10 stačí 16GiB)
   - Návody jsou uzpůsobeny této variantě
   - Na konci semestru stačí jednou zmáčknout klávesu delete a "je čisto", po Linuxu ani památky

 - ***Virtuální Linux na Windows OS, IDE ve Windows***
   - Relativně rychlá varianta kdy se kód píše dobře na systému který student zná a kompilace probíhá na virtuálu
   - Oproti návodům je třeba provádět kroky konfigurace navíc
   - Výsledný setup více odpovídá ladění na reálném hardware
   - Na konci semestru kromě smazání virtuálu zbude svinčík který je potřeba odstranit (nebo si jej zamilovat)
 
 - ***Nativní Linux na RaspberryPI, IDE ve Windows***
   - Vpodstatě identické ladění jako na hardware, žádná virtualizace
   - Nutné vlastnit těžko dostupný a relativně drahý hardware
   - V počítači máte jen IDE, vše s Linuxem je oddělené a můžete kdykoli na konci semestru demonstrativně zlikvidovat.
   - Pro akademický rok 2022/2023 nepodporovaná varianta (můžete ji zkusit ale nebudeme dodávat podporu - pouze ve volném čase, kterého je málo)

Podle toho se liší způsob instalace, popsaný v následujících kapitolách. Pro virtualizaci existují dva významné nástroje VirtualBox a VmWare. 
Obecně je VmWare rychlejší, lépe podporovaný ale je též placený ale základní verze Playeru je zcela zdarma pro nekomerční použití. 

Pozor Windows 11 není momentálně podporován u VmWare Playeru, tato volba je tedy možná jen u staršího Linuxu a u Windows 10 (kterému končí podpora).

VirtualBox na Windows 11 sice nainstalujete, ale není zaručeno že pod Windows 11 poběží (Vivat Microsoft compatibility made in India!) 

Pokud máte Windows 11, máte tedy možnost pouze multibootu a modlit se aby čip "TPM" v BIOSu nedělal problémy. **Záloha před operací nutná, možnost následné pitvy vysoká!**
