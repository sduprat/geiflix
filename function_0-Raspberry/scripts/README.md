># Server For the GeiCar
>
>This server is developed in Python using the Pican 2 libraries. To install the libraries, follow the instructions on [Elektor's website](http://skpang.co.uk/catalog/images/raspberrypi/pi_2/PICAN2UG13.pdf).
>
>The server is launched with
>
>````
>python3 server_old.py
>````
>The default port of the server is `6666`. To change it you have to modifiy the code.

# Serveur de la Raspberry Pi

## Topologie (TD)

| Device    | Ethernet (@IP)   | CAN    | Produit | Consomme |
|------------ | ---------- | ---------- | --- | --- |
|Jetson    | ✔️ 192.168.1.10 |   | `cam_angle` : angle de la personne sur la caméra  `lidar_distance` : distance de la personne à la voiture||
|
| Raspi    | ✔️ 192.168.1.2    |     |
| Nucleo   |    |    |

## Fonctionnement

Un seul thread tourne (*TBD*), de type `EthReceiver`. Il écoute sur le lien Ethernet les données en provenance de la Jetson, lit l'en-tête du message pour déterminer quel est son contenu (distance du LIDAR : header=`DST`, angle de la caméra : header=`ANG`), et le traite en fonction :
- Si la distance est supérieure à 2.1m, avancer;
- Si la distance est inférieure à 1.9m, reculer;
- Sinon s'arrêter.
On lit également les messages en provenance de la Discovery sur le CAN, pour récupérer les données des ultrasons et, si un objet est détecté à moins de 50cm (devant en marche avant, derrière en marche arrière), on arrête les moteurs.  
Enfin , l'ordre est envoyé à la Nucleo sur le CAN (marche av/arr, stop).
