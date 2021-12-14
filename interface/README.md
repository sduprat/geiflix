# Compiler le projet

Pour compiler le projet, il faut disposer des dépendances suivantes :
  - `CMake: ^3.16`

Les autres dépendances peuvent être installées avec le script `tools/install_tools.sh`.

La génération du buildsystem doit se faire dans un dossier build. Il peut être généré de la manière suivante :
```bash
mkdir -p build
cd build
cmake ..
```

Le buildsystem peut être spécifié avec l'option `-G` de cmake :
```bash
# Génération d'un build avec un GNU make
cmake .. -G "Unix Makefiles"
```

L'ensemble des buildsystems peut être consulté sur cette [page](https://cmake.org/cmake/help/latest/manual/cmake-generators.7.html).

## Compilation sur Windows

La compilation sur Windows peut être faite avec les générateurs suivants :
  - `Ninja`
  - `MinGW Makefiles`
  - `Visual Studio *`

### Compilation avec MinGW

Pour utiliser `MinGW`, il faut télécharger MinGW [ici](https://sourceforge.net/projects/mingw-w64/files/Toolchains%20targetting%20Win64/Personal%20Builds/mingw-builds/8.1.0/threads-posix/seh/x86_64-8.1.0-release-posix-seh-rt_v6-rev0.7z) et ajouter le dossier `bin` au PATH.

La compilation se fait alors avec :
```bash
mingw32-make.exe
```
