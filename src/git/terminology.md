## Základní terminologie

Vymezme si několik základních pojmů, abychom si rozuměli.

### repozitář (repo)

Sada verzovaných souborů a záznamy o jejich historii. Pokud je repozitář uložen na našem počítači, nazýváme jej lokální repozitář (local repo). Jeli uložen na jiném stroji, hovoříme o vzdáleném repozitáři (remote repo).

### klonování (cloning)
Stažení repozitáře z remote repa. Klonujeme v okamžiku, kdy na lokálním počítači repozitář neexistuje.

### snapshot
Stav repozitáře v konkrétním bodě v historii.

### diff
Rozdíl mezi dvěmi snapshoty. Tedy rozdíl stavu verzovaných souborů.

### commit
Záznam, který obsahuje referenci na předchozí, následujicí snapshot a diff mezi nimi. Zároveň každý commit má svůj unikátní dvaceti bytový hash, který jej jednoznačně identifikuje v rámci repozitáře.

### push
Nahrání nových comitů na remote repo.

### fetch
Stažení commitů z remote repo na lokál. Fetchujeme, pokud na lokále máme repozitář naklonovaný, ale nemáme stažené nejnovější commity.

### větev (branch)
Řetězec na sebe navazujicích commitů. Ze základu má každý repozitáž jednu větev ("master", někdy "main"). Probíha-li však vývoj několika funkcionalit vedle sebe, je možné tyto vývoje rozdělit do zvláštnich větví a připojit je spátky k hlavní větni, až je funkcionalita dokončená.

