# Function 2 - Jetson

Petit blabla sur la Jetson. [URL du site web de nividia](https://developer.nvidia.com/embedded/learn/get-started-jetson-nano-devkit)

## Creation de la carte SD

Recuperer l'image de la carte SD de base [ici](https://developer.nvidia.com/jetson-nano-sd-card-image) et ecrivez là sur une carte SD suffisamment grande et rapide (minimum: 32 GB UHS-1)

- Windows
Utilisez un utilitaire comme celui de la [sd association](https://www.sdcard.org/downloads/formatter_4/eula_windows/)

- Linux
Utilisez [Etcher](https://www.balena.io/etcher) pour ecrire l'image

## Compte et mot de passe

Le compte configuré sur la carte SD est:
- identifiant: admin-jetson
- mot de passe: 4dminJ3tson*

## Installation de ROS

Tutoriels suivis:
- [https://www.jetsonhacks.com/2019/10/23/install-ros-on-jetson-nano/] (https://www.jetsonhacks.com/2019/10/23/install-ros-on-jetson-nano/)
- [https://www.stereolabs.com/blog/ros-and-nvidia-jetson-nano/] (https://www.stereolabs.com/blog/ros-and-nvidia-jetson-nano/)

On commence par le premier tutoriel en appliquant les commandes suivantes:
- *git clone https://github.com/JetsonHacksNano/installROS.git*
- *cd installROS*
- *./installROS.sh -p ros-melodic-desktop*
- *cd ~*

Puis on enchaine sur la deuxieme partie du second tutoriel concernant l'initialisation du workspace:
- *sudo apt-get install cmake python-catkin-pkg python-empy python-nose python-setuptools libgtest-dev python-rosinstall python-rosinstall-generator python-wstool build-essential git*
- *mkdir -p ~/catkin_ws/src*
- *cd ~/catkin_ws/*
- *catkin_make*
- *echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc*

RQ: Il est interessant d'ouvrir ~/.bashrc et de retirer (commenter) la ligne *source /opt/ros/melodic/setup.bash* si elle existe et la remplacer par *source ~/catkin_ws/devel/setup.bash*

Puis on continu :
- *source ~/.bashrc*

ROS est installé, le WS est initialisé.

## Installation du SDK C++ RPLidar

On va recuperer le depot git de slamtec et le compiler :
- *cd ~*
- *git clone rplidar_sdk*
- *cd rplidar_sdk*

Il est interssant d'etre sur de la version que l'on compile, en faisant un checkout d'une version donnée :
- *git tag*   <-- pour recuperer la liste des versions du SDK
- *git checkout <version>*  <-- avec <version> une version de la liste precedente
- *cd sdk*
- *make*

**Il n'y a pas de make install !** Les resultats de la compilation sont dans *~/rplidar_sdk/sdk/output/Linux/Release/*. Si on veut installer pour tout le monde, on peut appliquer les commandes suivantes:
- recuperez les fichiers librplidar_sdk.la et librplidar_sdk.pc presents dans le repertoire scripts de ce depot avec scp
- Installez librplidar_sdk.la dans /usr/lib/aarch64-linux-gnu
- Installez librplidar_sdk.pc dans /usr/lib/aarch64-linux-gnu/pkgconfig
- *cd output/Linux/Release*
- *sudo cp -v *.a /usr/lib/aarch64-linux-gnu
- *sudo cp -v simple_grabber /usr/bin*
- *sudo cp -v ultra_simple /usr/bin*
- *cd ../../../sdk/*
- *sudo cp -v include/* /usr/include*
- *sudo cp -rv srv/hal /usr/include*
- *ranlib /usr/lib/aarch64-linux-gnu/librplidar_sdk.a*

A partir de là, pour compiler un programmer (rptest, apr exemple), on peut faire:
*g++ -o rptest rptest.cpp $(pkg-config --libs --cflags librplidar_sdk)*

## installation du SDK ROS RPLidar

Le support du RPLidar pour ROS se trouve dans le depot wwww, qu'il faut cloner dans le workspace de ROS:
- *cd ~/catkin_ws/*
- *git clone https://github.com/slamtec/rplidar_ros*

On va ensuite selectionner la version du SDK que l'on compte utiliser
- *cd rplidar_ros*
- *git tag*   <-- pour recuperer la liste des versions du SDK
- *git checkout <version>*  <-- avec <version> une version de la liste precedente
- *cd ..*
- *source ~/catkin_ws/devel/setup.bash*
- *rosdep install --from-paths src --ignore-src -r -y*
- *catkin_make -DCMAKE_BUILD_TYPE=Release*

Si tout se passe bien, on peut verifier l'installation avec:
- *roslaunch rplidar_ros view_rplidar.launch*
