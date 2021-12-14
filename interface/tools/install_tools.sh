#!/bin/bash

#
# Détection de l'OS
#
ISWINDOWS=false
ISLINUX=false
ISMAC=false
if [[ "$OSTYPE" == "cygwin" ]] || [[ "$OSTYPE" == "msys" ]];then
  ISWINDOWS=true
elif [[ "$OSTYPE" == "linux-gnu"* ]]; then
  ISLINUX=true
elif [[ "$OSTYPE" == "darwin"* ]]; then
  ISMAC=true
fi

if [[ $ISWINDOWS == false ]] && [[ $ISLINUX == false ]] && [[ $ISMAC == false ]]; then
  echo "Unsupported OS: ${OSTYPE}"
fi



#
# Installation de Qt
#

QTFILE_LINUX=qt-opensource-linux-x64-5.12.9
QTFILE_MACOS=qt-opensource-mac-x64-5.12.9
QTFILE_WINDOWS=qt-opensource-windows-x86-5.12.9

if $ISLINUX; then
  if [[ ! -f /tmp/${QTFILE_LINUX}.run ]]; then
    echo "Téléchargement de Qt"
    curl -k --fail -L https://download.qt.io/official_releases/qt/5.12/5.12.9/${QTFILE_LINUX}.run > /tmp/${QTFILE_LINUX}.run
  fi
elif $ISMAC; then
  if [[ ! -f /tmp/${QTFILE_MACOS}.dmg ]]; then
    echo "Téléchargement de Qt"
    curl -k -L https://download.qt.io/official_releases/qt/5.12/5.12.9/${QTFILE_MACOS}.dmg > /tmp/${QTFILE_MACOS}.dmg
  fi
elif $ISWINDOWS; then
  if [[ ! -f /tmp/${QTFILE_WINDOWS}.exe ]]; then
    echo "Téléchargement de Qt"
    curl -k --fail -L https://download.qt.io/official_releases/qt/5.12/5.12.9/${QTFILE_WINDOWS}.exe > /tmp/${QTFILE_WINDOWS}.exe
  fi
fi
if [ $? -ne 0 ];then
  echo "Erreur de téléchargement de Qt"
  exit 1
fi

echo "Voulez-vous installer Qt5 ? [y]es, [n]o"
read INSTALL_QT
if [[ $INSTALL_QT == "y" ]]; then
  if $ISLINUX; then
    echo "Installation de Qt"
    echo "Veuillez suivre les instructions de l'installeur Qt"
  
    chmod +x /tmp/${QTFILE_LINUX}.run
    sudo /tmp/${QTFILE_LINUX}.run
    if [[ $? -ne 0 ]]; then
      echo "L'installation de Qt a échoué"
      exit 1
    fi
  elif $ISMAC; then
    echo "Montage du drive de Qt"
  
    open /tmp/${QTFILE_MACOS}.dmg
    while [ ! -d /Volumes/${QTFILE_MACOS} ]; do
      sleep 1
    done

    echo "Installation de Qt"
    echo "Veuillez suivre les instructions de l'installeur Qt"
    open /Volumes/${QTFILE_MACOS}/${QTFILE_MACOS}.app
  elif $ISWINDOWS; then
    echo "Installation de Qt"
    echo "Veuillez suivre les instructions de l'installeur Qt"
  
    start /tmp/${QTFILE_WINDOWS}.exe

    echo "Définition des variables d'environement necessaires"
    # Les DLLs doivent être dans le path pour lancer l'executable
    echo "export PATH=\$PATH:C:/Qt/Qt5.12.9/5.12.9/mingw73_64/bin" >> ~/.bashrc
    # La variable QT_PLUGIN_PATH doit pointer sur l'installation de Qt
    echo "export QT_PLUGIN_PATH=C:/Qt/Qt5.12.9/5.12.9/mingw73_64/plugins" >> ~/.bashrc
  fi
fi
