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

## Topologie

| Device    | Ethernet (@IP)   | CAN    | Produit | Consomme |
|------------ | ---------- | ---------- | --- | --- |
|Jetson    | ✔️ 192.168.1.10 |   | `cam_angle` : angle de la personne sur la caméra  `lidar_distance` : distance de la personne à la voiture||
|
| Raspi    | ✔️ 192.168.1.2    |     |
| Nucleo   |    |    |

