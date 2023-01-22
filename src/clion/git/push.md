## CLion/GIT Odeslání na server

Tuto funkci lze aktivovat z menu `Git`/`Push` nebo kliknutím na modrou ikonu šipky směřující k sobě:

![toolbar](toolbar.png)

Po aktivaci vyskočí okno

![push](push.png)

V levé části lze vidět vzdálené servery, do kterých je potřeba něco "natlačit", napravo jsou všechny odesílané změny.

Kliknutím na tlačítko Push se lokální data odešlou na server.

**POZOR** Tlačítko Push je rozbalovací a je možné "natlěčit" na server "hrubou silou" tj Force Push. Tuto funkci 
**nepoužívejte**, dochází při ní ke zničení stavu serveru a je velmi obtížné stav obnovit, aby mohl tým bez problému 
fungovat. Funkce se využívá ve velmi specifických případech, které během výuky nenastanou.

**POZOR** Může se stát, že obah serveru je novější, než ten který máte u sebe. Může být nutné nejprve [aktualizovat 
projekt](update.md) předtím než provedete Push (Program vás na to upozorní)!
