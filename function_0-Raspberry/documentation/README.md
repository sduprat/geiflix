# Function 2 - Jetson

Le systeme central est basé sur une raspberry pi 3B, sur laquelle est fixée un "HAT" pican2 de chez [skpang](https://www.skpang.co.uk/collections/hats/products/pican2-can-bus-board-for-raspberry-pi-2-3)

## Creation de la carte SD

Recuperer l'image de la carte SD de base [ici](https://www.raspberrypi.org/software/operating-systems/) et ecrivez là sur une carte SD suffisamment grande et rapide (minimum: 8 GB). Recuperez de preference la version Lite.

- Windows
Utilisez un utilitaire comme celui de la [sd association](https://www.sdcard.org/downloads/formatter_4/eula_windows/) pour ecrire l'image

- Linux
Utilisez [Etcher](https://www.balena.io/etcher) pour ecrire l'image.

## Configuration de base du systeme

Apres le premier demarrage, lancez (*sudo raspi-config*) et configurez les points suivants:

- Interfaces: Activez SSH
- Localisation : Passez le clavier en francais, regler le WIFi sur France, timezone sur Paris

Ensuite, modifiez /etc/hosts et /etc/hostname pour changer le nom *raspberrypi* du systeme en *geicar*

Faites une mise à jour (*sudo apt update && sudo apt full-upgrade -y && sudo apt autoremove -y*)

## Compte et mot de passe

Le seul compte present est celui initialement fourni avec raspberryOS à savoir:
- identifiant: pi
- mot de passe: raspberry

## Configuration de la carte PiCan2

Pour activer la carte, rajoutez à la fin du fichier /boot/config.txt les lignes suivantes:
*dtparam=spi=on*
*dtoverlay=mcp2515-can0,oscillator=16000000,interrupt=25*
*dtoverlay=spi-bcm2835-overlay*

Ensuite copiez le service present dans le depot sous ./systemd-system/pican2.service dans /lib/systemd/system

Activez le service (*sudo systemctl enable pican2*)

Rebootez (*sudo reboot*)

Au reboot, verifiez que le service est demarré (*sudo systemctl status pican2*)

Installez ensuite le support python et cli (*sudo apt install python-can can-utils*)

L'image ainsi créée est telechargeble directement depuis le serveur du GEI [2021-05-07-raspios-buster-armhf-lite-pican2.img.7z] (http://srv-geitp.insa-toulouse.fr/geiflix/rpi-geicar/2021-05-07-raspios-buster-armhf-lite-pican2.img.7z) (RQ: pensez au VPN depuis chez vous)

## Installation de ROS

La version de ROS installée sur la carte SD est Kinetic Kame. Pour l'installer, la procedure suivante a été scrupuleusement suivie : [Installing ROS Kinetic on the Raspberry Pi] (http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Kinetic%20on%20the%20Raspberry%20Pi)

L'image ainsi créée est telechargeble directement depuis le serveur du GEI [2021-05-07-raspios-buster-armhf-lite-pican2-ros_Kinetic.img.7z] (http://srv-geitp.insa-toulouse.fr/geiflix/rpi-geicar/2021-05-07-raspios-buster-armhf-lite-pican2-ros_Kinetic.img.7z) (RQ: pensez au VPN depuis chez vous)