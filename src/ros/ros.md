# Robotic Operating System 2

Název "Robot Operating System" poněkud klame svým zněním. Nejedná se o samostaný operační sýstém, nýbrž spíše o middle-ware, tedy softwarový nástroj (knihovnu), který pomáha propojit dílčí programy do komplexnejšího celku. V praxi si to můžeme představit tak, že máme jednoduchou aplikaci pro robot jezdící po čáre, kterou realizujeme pomocí 3 navzájem spolupracujících programů (příklad funguje jako ilustrační; takový robot samozřejmě můžeme naprogramovat pomocí jednoho programu; ilustrujeme tím ale komplexnější problém). První program vyčítá data ze snímače a provádí jednoduchou filtraci dat. Druhý program je mozkem celého řešení a rozhoduje o pohybu robotu. Třetí program pak přijímá řídicí pokyny a na jejich základě ovládá motory.

![Rviz](../images/robot_scheme.png)

Obr: Schéma fungování pomyslého line-tracking robotu na platformě Rasperry Pi s použitím ROSu.

V případě absence ROSu bychom museli vymyslet způsob jak spolu budou tyto tři programy komunikovat. Mohli bychom sdílet paměť, pipovat, posílat si IP zprávy, používat DBus, atd. Všechny tyto techniky fungují, ale vyžadují určitý programátorský um. My se těmito nízkouúrovňovými problémy nechceme zabývat a proto použijeme ROS.

V praxi si pak můžeme říct, že ROS komunikuje mezi procesy právě pomocí posílání UDP paketů. To umožňuje také komunikovat procesům, které běží na různých počítačích. Tomu říkáme distribuovaný systém.

Základ ROSu je postaven na 3 stavebních kamenech.

 - **ROS Node**
 - **ROS Topic**
 - **ROS Message**

**ROS Node** - Nodem je myšlený každý program do kterého přídáme klientskou knihovnu ROSu. Naučíme tedy program používat funkce ROSu. ROS Node je pak schopen "automaticky" objevit další instance (programy), které jsou spuštěny na stejné síťi a navázat s nimi komunikaci.

**ROS Topic** - Doména, ve které se posílá specifický okruh ROS Messagů.

**ROS Message** - Jedna instance odeslané zprávy. V rámci ROSu je možné posílat jenom zprávy, které jsou striktně zadefinovány a mají svůj jasně daný formát. Často obsahují také časovou značku, kdy byly odeslány.

Dále si zadefinujme dva typy postavení ROS Nodů při komunikaci.

**Subscriber** - ROS Node, který přijímá všechny zprávy v rámci daného ROS Topicu.

**Publisher** - ROS Node, který vytváří a odesíla zprávy v rámci daného ROS Topicu.

Náš robot-sledující-čáru příklad si pak můžem ilustrovat takto:


![Rviz](../images/ros_com_scheme.png)

Napíšeme zmíněné 3 programy. Jeden pro čtení dat ze snímače, druhý pro rozhodování jak se pohybovat a třetí pro ovládání motorů. První program (Node) vystaví svůj topic "SensorData" jako publisher. Druhý se přihlásí k odebírání zpráv jako subscriber v témuž topicu. V tuto chvíli dojde k navázání spojení a všechny zprávy publikované na tomto topicu budou směrovány k subsriberovi. Když pak první program přečte data ze snímače, vyfiltruje je a vytvoří z nich message, kterou odešle. Obdobným způsobem se vymění data i mezi druhým a třetím programem, pouze pod hlavičkou jiného topicuu.


Nyní máme vytvořené všechny tři programy. Ty spolu komunikují, ale robot přesto nefunguje podle přestav. Tušíme, že chyba je v tom, jak druhý program převádí data ze snímače na pohyb kol. Proto si napíšeme 4. program, který bude poslouchat veškerou komunikaci a bude ji logovat do souboru. Náš nový program tedy bude subscriberem pro oba dříve zavedené topicy "SensorData" a "MotorControl". V okamžiku kdy tento program zapneme, tak se ohlásí publisherům (Nodům, které data publikují) a od tohoto okamžiku všechny zprávy odeslané v topicích "SensorData" a "MotorControl" budou posílány také našemu logovacímu programu. Ten zprávy přijme a jejich obsah včetně časové značky vytiskne do souboru. Když se pak do souboru podíváme, zjistíme, že plánovací program vytváří akční zásah vždy s opačným znaménkem, proto přídáme "-" do výpočtu akčního zásahu a vše začne fungovat.


## Instalace ROS2

Tento návod je pouze českým přepisem oficiální dokumentace (Instalace pro Ubuntu)[https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html]. Primárně prosím používejte oficiální verzi. Tento návod je pouze doprovodný.

Instalace je doporučená na distribuci Ubuntu 22.04 LTS (long term stable). Instalovat budeme verzi ROSu z roku 2022, Humble.

Povolíme přístup do Ubuntu Universe repository
```
sudo apt install software-properties-common
sudo add-apt-repository universe
```

Přidáme do Linuxu repozitáře (servery) ze kterých je možné stáhnout ROS.
```
sudo apt update && sudo apt install curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

Necháme baličkovací systém načíst nově přidaná data.
```
sudo apt update
sudo apt upgrade
```

Samotný ROS nainstalujeme tímto příkazem. Trvá cca 10 min.
```
sudo apt install ros-humble-desktop
```

V neposlední řadě nainstalujeme build system zvalý colcon
```
sudo apt install python3-colcon-common-extensions
```

A na závěr si do souboru ~/.bashrc přídáme záznam o načítání ROSu do proměnného prostředí, kdykoliv zapneme terminál.
```
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

ROS je nyní nainstalován. Pokud vše proběhlo v pořádku, jste nyní schopni provést příkaz 
```
ros2
```

## Tvorba vlastního nodu

Vytvoříme si jednoduchou aplikaci, kde jeden node bude odesílat zprávu s pořadovým číslem a časovou značkou a druhý node zprávu přijme, vypíše a zjistí, s jakým zpožděním zpráva došla.

Nejprve si vytvoříme tzv. workspace pro náš projekt. Workspacem se myslí speciálně uspořádaná složka.

```
cd ~/
mkdir -p ros_ws/src
cd ros_ws/src
```

### C++ Node

Dále si vygenerujeme nový balíček (package) - Nutno spouštět ve složce ~/ros_ws/src.
```
ros2 pkg create --build-type ament_cmake cpp_publisher
```
Příkaz nám říká, že budeme volat program pkg create a chceme po něm, aby nám vytvořil balíček cpp_publisher.

Pokud se Vám stane, že předchozí příkaz neprojde z důvodu nedostatečných práv, vraťte se o složku zpět (cd ~/ros_ws) a upravte přístupová práva - pak zkuste balíček vytvořit znovu
```
sudo chmod 777 -R .
```


Nyní se náš balíček skládá z několika následujícíh souborů
```
~/ros_ws/src/cpp_publisher/
    include/
    src/
    CMakeLists.txt
    package.xml
```

Do adresářů include a src budeme ukládat naše zdrojové kódy a soubory CMakeLists.txt a package.xml slouží ke kompilaci balíčku.

CMakeLists.txt a package.xml obsahují velké množství předpřipravených direktiv, které slouží složitějším příkladům. Pro naše potřeby si můžeme tyto dva soubory smazat.
```
cd ~/ros_ws/src/cpp_publisher/
rm CMakeLists.txt
rm package.xml
```

Pomocí programu nano nebo vim si oba soubory znovu vytvoříme a přidáme následující obsah.
```
nano CMakeLists.txt
```

```
cmake_minimum_required(VERSION 3.8)
project(cpp_publisher)

## Set CMAKE standard and flags
SET(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
set(CMAKE_CXX_FLAGS_DEBUG " ${CMAKE_CXX_FLAGS_DEBUG} -g")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")

## Find catkin and any catkin packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

## Build talker and listener
include_directories(
        include
        ${rclcpp_INCLUDE_DIRS}
        ${std_msgs_INCLUDE_DIRS}
        )

add_executable(publisher src/main.cpp)
ament_target_dependencies(publisher rclcpp std_msgs)

install(TARGETS publisher DESTINATION lib/${PROJECT_NAME})

ament_package()
```

a
```
nano package.xml
```

```
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>cpp_publisher</name>
  <version>0.0.0</version>
  <description>The cpp_publisher package</description>

  <maintainer email="my@email.todo">cpp_publisher</maintainer>

  <license>TODO</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>rclcpp</depend>
  <depend>std_msgs</depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

Nyní si můžeme vytvořit soubor main.cpp ve složce src a do něj napíšeme vlastní program
```
nano src/main.cpp
```

```
#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/header.hpp>

class TimestampPublisher : public rclcpp::Node {
public:
        TimestampPublisher(): Node("timestamp_publisher") {
                publisher_ = this->create_publisher<std_msgs::msg::Header>("timestamp_topic", 10);
                using namespace std::chrono_literals;
                timer_ = this->create_wall_timer(10ms, std::bind(&TimestampPublisher::timer_callback, this));
        }

private:
        void timer_callback() {
                auto message = std_msgs::msg::Header();

                message.stamp = this->get_clock()->now();
                message.frame_id = "origin";

                publisher_->publish(message);
        }

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr publisher_;
};

int main(int argc, char **argv) {
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<TimestampPublisher>());
        rclcpp::shutdown();
        
        return 0;
}
```

Nyní se vrátíme do kořene našeho workspacu a zavoláme příkaz pro build celého workspacu.
```
cd ~/ros_ws
colcon build
```

Pokud se nevypíše žádná chyba, máme hotový publisher, který je uložený v  ~/ros_ws/src/cpp_publisher/install.

Aby si Linux načetl nově zkompilované programy z našeho ros_ws přidámi si tento workspace do systémového prostředí (environmentu).
```
source ~/ros_ws/src/cpp_publisher/install/setup.bash
```

Abychom tuto akci již nemuseli opakovat přidáme si tento řádek také do ~/.bashrc
```
echo "source ~/ros_ws/src/cpp_publisher/install/setup.bash" >> ~/.bashrc
```

Nyní si dvě okna terminálu. V jednom spustíme námi vytvořený publisher
```
ros2 run cpp_publisher publisher
```

A ve druhém si poslechneme zprávy na topicu /timestamp_topic
```
ros2 topic echo /timestamp_topic
```

Pokud vidíte v terminále výpis zpráv, vše pracuje, jak má.

### Python Node

Vytvoříme si další balíček pomocí

```
cd ~/ros_ws/src/
ros2 pkg create --build-type ament_python python_subscriber
```

a upravíme si strukturu balíčku tak, aby vypadala následovně.

```
~/ros_ws/src/python_subscriber/
    python_subscriber/
        __init__.py
        python_subscriber.py
    resource/
        ...
    test/
        ...
    setup.cfg
    setup.py
    package.xml
```

Složka python_subscriber bude soužit k uložení hlavního skriptu python_subscriber.py. package.xml obdobně jako pro C++ příklad. setup.py a setup.cfg slouží k instalaci python balíčku do workspacu.

Zmíněné soubory si pak upravíme následovně.
```
nano package.xml
```

```
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>python_subscriber</name>
  <version>0.0.0</version>
  <description>TODO</description>
  <maintainer email="todo@todo.todo">python_subscriber</maintainer>
  <license>TODO</license>

  <exec_depend>rclpy</exec_depend>
  <exec_depend>std_msgs</exec_depend>

  <!-- These test dependencies are optional
  Their purpose is to make sure that the code passes the linters -->
  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

pak
```
nano setup.py
```

```
from setuptools import setup

package_name = 'python_subscriber'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TODO',
    maintainer_email='todo@todo.todo',
    description='TODO',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'python_subscriber = python_subscriber.python_subscriber:main',
        ],
    },
)
```

a finálně
```
nano python_subscriber/python_subscriber.py
```

```
import rclpy
from rclpy.node import Node

from std_msgs.msg import Header


class TimestampSubscriber(Node):

    def __init__(self):
        super().__init__('timestamp_subscriber')
        self.subscription = self.create_subscription(
            Header,
            'timestamp_topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.stamp.nanosec)


def main(args=None):
    rclpy.init(args=args)

    timestamp_subscriber = TimestampSubscriber()

    rclpy.spin(timestamp_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    timestamp_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Nyní se můžeme vrátit do kořene workspacu a vše zkompilovat.
```
cd ~/ros_ws/
colcon build
```

Zaktualizujeme si proměné prostředí.
```
echo "source ~/ros_ws/src/python_subscriber/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Pokud máme aktivní cpp_publisher, pak zapnene python_subscriber node v dalším okně pomocí
```
ros2 run python_subscriber python_subscriber
```

a vidíme výpis přijímaných zpráv.

Pomocí programu rqt_graph si můžeme prohlédnout aktuální stav propojení nodů.
```
ros2 run rqt_graph rqt_graph
```

![rqt_graph](../images/rqt_graph.png)

Obr: vizualizace komunikace mezi nody pomocí rqt_graph


## Rviz

Rviz je vizualizační nástroj, který je dodáván jako součást ROSu. Jedná se o aplikaci, která dokáže poslouchat širokou paletu předdefinovaných ROS zpráv a vizualizovat je v 3D grafickém prostředí.

Obvykle Rviz používáme pro vizualizaci pointcloudů (mračna bodů z LIDARu), obrázků z kamery, vykreslování geometrických primitiv v prostoru, vizualizace occupancy grid map, atd.

Rviz aktivujeme pomocí

```
rviz2
```

Vizualizaci konkrétního topicu pak aktivujeme pomocí

Add -> By topic -> [náš topic]

V sekci

Add -> By display type

vidíme všechny podporované typy zpráv (viz online dokumentace ROSu).

![rqt_graph](../images/rviz.png)
Obr: příklad vizualizace pointcloudu a kamery v Rvizu

Nyní si zkusme vytvořit vlastní Node, který bude vykreslovat geometrické primitivum do RVizu. Vyjděme z příkladu cpp_publisher a vytvoříme následujicí program.

```
cd ~/ros_ws/src
ros2 pkg create --build-type ament_cmake cpp_rviz_publisher
```

CMakeLists.txt

```
cmake_minimum_required(VERSION 3.8)
project(cpp_rviz_publisher)

## Set CMAKE standard and flags
SET(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
set(CMAKE_CXX_FLAGS_DEBUG " ${CMAKE_CXX_FLAGS_DEBUG} -g")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")

## Find catkin and any catkin packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

## Build talker and listener
include_directories(
        include
        ${rclcpp_INCLUDE_DIRS}
        ${std_msgs_INCLUDE_DIRS}
        ${visualization_msgs_INCLUDE_DIRS}
        )

add_executable(rviz_publisher src/main.cpp)
ament_target_dependencies(rviz_publisher rclcpp std_msgs visualization_msgs)

install(TARGETS rviz_publisher DESTINATION lib/${PROJECT_NAME})

ament_package()
```

package.xml

```
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>cpp_rviz_publisher</name>
  <version>0.0.0</version>
  <description>The cpp_rviz_publisher package</description>

  <maintainer email="my@email.todo">cpp_rviz_publisher</maintainer>

  <license>TODO</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>rclcpp</depend>
  <depend>std_msgs</depend>
  <depend>visualization_msgs</depend>
  
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

src/main.cpp

```
#include "rclcpp/rclcpp.hpp"
#include <visualization_msgs/msg/marker.hpp>

class CubePublisher : public rclcpp::Node {
public:
        CubePublisher(): Node("cube_publisher") {
                publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("cube_topic", 10);
                using namespace std::chrono_literals;
                timer_ = this->create_wall_timer(100ms, std::bind(&CubePublisher::timer_callback, this));
        }

private:
        void timer_callback() {
                auto marker = visualization_msgs::msg::Marker();
                marker.header.frame_id = "map";
                marker.header.stamp = this->get_clock()->now();
                marker.ns = "cube";

                marker.id = 0;
                marker.type = visualization_msgs::msg::Marker::CUBE;
                marker.action = visualization_msgs::msg::Marker::ADD;

                marker.pose.position.x = sin(pose_);
                marker.pose.position.y = cos(pose_);
                marker.pose.position.z = 0.1*sin(5*pose_);

                marker.pose.orientation.x = 0.0;
                marker.pose.orientation.y = 0.0;
                marker.pose.orientation.z = 0.0;
                marker.pose.orientation.w = 1.0;

                marker.scale.x = 0.1;
                marker.scale.y = 0.1;
                marker.scale.z = 0.1;

                marker.color.a = 1.0;
                marker.color.r = 0.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;

                pose_ += 0.01;

                publisher_->publish(marker);
        }

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
        float pose_ = 0;
};

int main(int argc, char **argv) {
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<CubePublisher>());
        rclcpp::shutdown();

        return 0;
}
```

V Rvizu si pak otevřeme topic /cube topic.

## Kam dál?

Tento tutoriál je popisuje pouze malý zlomek všech možných funkcionalit této obšírné platformy.

Oficiální web [1] - [http://www.ros.org/](https://ros.org/)

Oficiální tutoriály [2] - [http://wiki.ros.org/ROS/Tutorials](http://docs.ros.org/en/humble/index.html)

Naučit se používat ROS Services [6]

Seznamy několika předdefinovaných ROS Messagů - [3] [4]

Pro reálnou práci se zdrojovými kódy je vhodné použít nějaké IDE. V případě Linuxu vřele doporučuji programy od JetBrains, CLion pro vývoj C++ a Pycharm pro práci s pythonem. Oba programy jsou pro studenty VUT zdarma.

## Reference

[1] [http://www.ros.org/](https://ros.org/)

[2] [http://wiki.ros.org/ROS/Tutorials](http://docs.ros.org/en/humble/index.html)

[3] [http://wiki.ros.org/std_msgs](https://docs.ros2.org/galactic/api/std_msgs/index-msg.html)

[4] [http://wiki.ros.org/sensor_msgs](https://docs.ros2.org/foxy/api/sensor_msgs/index-msg.html)

[6] [http://wiki.ros.org/Services](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html)

[7] [http://wiki.ros.org/ROS/Installation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
