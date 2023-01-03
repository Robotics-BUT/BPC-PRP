## Build system CMake

CMake je soubor nástrojů, které zjednodušují kompilaci projektů a knihoven takovým
způsobem, aby byly nezávislé na operačním systému a kompilátoru. Funguje tak,
že pomocí jednotného konfiguračního souboru CMakeLists.txt vygeneruje Makefile
pro UNIX-like systémy a pro Windows generuje MSVC pracovní prostory. Velkou
výhodou CMake je správa závislostí - aplikace si mohou definovat na jakých knihovnách
jsou závislé, přičemž CMake kontroluje, jestli jsou tyto knihovny dostupné a navíc v
požadované verzi. Další velkou výhodou je možnost vytvářet jak spustitelné soubory
tak knihovny pomocí jedné jednoduché konfigurace umístěné v CMakeLists.txt.

Ukázkový soubor CMakeLists.txt pro aplikaci:
```cmake
cmake_minimum_required(VERSION 3.7)
project(MyCoolRobot)

set(CMAKE_CXX_STANDARD 17) 

add_executable(MyCoolRobot main.cpp)
```

Ukázkový soubor CMakeLists.txt pro knihovnu:
```cmake
cmake_minimum_required (VERSION 3.7)
project (MyCoolLibrary VERSION 0.1 LANGUAGES CXX )

include(GNUInstallDirs)
set (CMAKE_CXX_STANDARD 17)
file (GLOB SOURCES src/*.cpp )

file (GLOB HEADERS include/*.h)

add_library(libmycoollibrary ${SOURCES})

target_include_directories(libmycoollibrary PUBLIC
    $<BUILD_INTERFACE : ${CMAKE_CURRENT_SOURCE_DIR}/include> 
    $<INSTALL_INTERFACE : include>
    PRIVATE src)

install (TARGETS libmycoollibrary EXPORT MyCoolLibraryConfig
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

install(DIRECTORY include/DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(EXPORT RoboUtilsConfig DESTINATION share/MyCoolLibrary/cmake)

export(TARGETS libroboutils FILE MyCoolLibraryConfig.cmake)
```

