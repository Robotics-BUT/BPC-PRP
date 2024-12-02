# Instalace a seznámení se s prostředím
Cvičící: Ing. Adam Ligocki

## Instalace Linux (cca 45min)

Nainstalujte si operační systém Linux Ubuntu 20.04 na fyzický stroj (preferovaná varianta), případně jako alternativu si vytvořte virtální stroj pomocí VirtualBoxu a operační systém si nainstalujte zde.

Návod naleznete zde [Příprava prostředí](../chap_1_software/text/vb.linux_installation.md)

✅ Senzamte se se GUI systému.

✅ Otevřete si příkazový řádek, vyzkoušejtesi pohyb v souborovém systému

✅ Vytovřte, smažte soubor/složku

✅ Pomocí balíčkovacího manažeru si nainstalujte git, midnight commander (mc). Pro odvážné také textový editor vim

<details>
    <summary>Nainstaloval jsem si vim a omylem jej zapnul. Co teď?</summary>

Vim vypnete touto sekvencí: stiskněte ESC, pak jej pusťte, přidržte LSHITF a dakrát klávesu 'Z'.
    
Pro zájemce, tutorilál práce s vim: [zde](https://www.openvim.com/)
</details>

## Instalace ROS (cca 45min)

Projděte si návod na [Robotic Operating System](../chap_1_software/text/ros.md). Zde si nastudujte přibližný princip fungování systému ROS aby jste později chápali jeho základní mechanizmy. 

Podle návodu si nainstalujte ROS na svůj čerstně nainstalovaný operační systém.

Detailní přednáška na práci s ROSem bude v druhé polovině semestru.

✅ Zavolejte z terminálu příkaz roscore. Měl by se objevit výpis o startu programu. Neměly by být přítomny žádné chybové hlášky

✅ Voláním přžíkazu rviz ověřte, že Vám nastartuje vizualizařní program RVIZ (paralelně musí běžet roscore).

## Instalace CLion (cca 30 min)

Nainstalujte si CLion pomocí balíčkovacího manageru Snap

```shell script
sudo snap install --classic clion
```

nebo z [oficiálního webu](https://www.jetbrains.com/clion/) si stáhněte IDE CLion a seznamte se s ním (viz [CLion IDE](../chap_1_software/text/clion.md)). Registrací pomocí školního emailu získáte licenci na užití softwaru zdarma.

✅ Pouze prací v konzole si vytvořte jednoduchý hello world program a zkompilujte jej pomocí volání g++ kompilátoru. Otestujte funkčnost programu.

✅ Tentýž hello world program realizujte ve vývojovém prostředí CLion. Zkompilujte jej a ověřte jeho funkčnost.

✅ Diskutujte výhody práce s IDE oproti kompilaci v příkazovém řádku.

### Záložní plán

Předinstalovaný obraz Ubuntu 20.04 s ROSem pro VB

https://drive.google.com/file/d/1_wCduSS30O7lHB2oYQVEErgrPAWc1ALR/view?usp=sharing


