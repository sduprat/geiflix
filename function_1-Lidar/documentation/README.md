# Function 1 - Lidar

Le lidar monté sur les voitures est un modele rplidar a2 de Slamtec. Le site de telechargement se trouve [ici] (https://www.slamtec.com/en/Support#rplidar-a-series)

Le SDK peut etre telechargé sous forme d'archive ou de depot git. On privilegiera le depot.

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

## Installation du SDK ROS RPLidar

Le support du RPLidar pour ROS se trouve dans le depot rplidar_ros, qu'il faut cloner dans le workspace de ROS:
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

