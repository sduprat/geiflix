# Image recognition

## Brief
Recognition of group of white pixels on a picture.
Author : Celestin RONGERE
celestinrongere@gmail.com

## Building the program
You need `CMake`,`make`,`gcc` and `git` installed.
#### APT
```sh
sudo apt update
sudo apt upgrade
sudo apt install cmake make gcc git
```

Then you need to clone the project, and navigate to image analysis folder (note that it will create a directory with the name of the project, so it is not necessary to create one):
```sh
git clone -b 2022-the-general https://github.com/INSA-GEI/geiflix.git
cd geiflix/function_2-Jetson/camera/analyseIMG
```

Once done, create a `build` folder at the same level than `analyseIMG.cpp`, go into it and run `Cmake`:
```sh
mkdir build
cd build
cmake ..
```

Finally, you can launch make to build it:
```sh
make
```

*Hooray! You did it! You can now enjoy running the program...*

## Running the program

Still inside the `build` folder, you can launch the app by entering the following command:
```sh
./analyseIMG
```

## Additional informations

The program uses the first video source found on the computer.
You just have to run the program and adjust the different parameters according to the type of image that you want to analyse.
