### Eskalace oprávnění

`sudo`

Meta příkaz. Operace specifikovaná za tímto příkazem bude provedena v režimu oprávnění administrátora operačního systému. 
Obvykle používáme, když zasahujeme do systémových souborů.


```shell
sudo mkdir /etc/config      ...      vytvoří složku "config" v systémovém adresáři "/etc".
sudo rm -r /                ...      příkaz rekurzivně smaže celý adresář "root" (v podstatě smaže celý disk včetně OS)
```
