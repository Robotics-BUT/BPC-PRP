## Orientace v systému

Souborová struktura Linuxu se odvozuje od tzv. kořene (root), který značíme jako `/` (vzdáleny ekvivalent `C:/` na Windows).

V kořenovém adresáři pak nalezneme složky jako:

  - `bin/` - obsahuje binárky (spustitelné soubory operačního systému).
  - `home/` - adresář, který obsahuje domovské složky uživatelů, tj vaše soubory.
  - `dev/` - obsahuje soubory které mapují hardware počítače (interní a externí disky, sériovou linku, usb, síťové rozhraní, atd.).
  - `tmp/` - dočasná složka. Zde si programy odkládají svá dočasná data.
  - `media/` - místo kde se připojují externí disky.
  - `etc/` - složka obsahuje konfiguraci systému a všech nainstalovaných aplikací

V Linuxu neexistuje ekvivalent fyzického dělení na disky. Fyzické disky lze připojit do libovolné složky. Ze zadané cesty k souboru 
tedy nelze přímo usoudit na kterém fyzickém disku se data nacházejí. To je velmi odlišné od Windows kde cesta vždy začíná písmenem 
fyzického disku, a možnost připojit disk do složky přichází až od systému souborů NTFS.

Složka `tmp` vede na většině distribucí do virtuálního disku v paměťi RAM počítače. Je tedy extrémně rychlá, ale za cenu že se při 
vypnutí počítače smaže. Spousta nástrojů ji používá pro vytvoření mezivýsledků kompilace.

Po příhlášení do konzole se obvykle nacházíte v domovském adresáři, tj na místě `/home/<jmeno_uzivatele>/`