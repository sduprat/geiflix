# Function 8 - Camera

High definition camera embedded on geibike is [LI-USB30-AR023ZWDRB](https://www.leopardimaging.com/product/usb30-cameras/usb30-box-cameras/li-usb30-ar023zwdrb/).


## Neural network setup

We are using jetson-inference detectnet

Steps :
- Clone the repo : git clone --recursive https://github.com/dusty-nv/jetson-inference.git
- Build the project from source : [Guide](https://github.com/dusty-nv/jetson-inference/blob/master/docs/building-repo-2.md)


## Buzzer setup

Put electronic schema

Activate the PWM on pin 33 :
Bash : sudo /opt/nvidia/jetson-io/jetson-io.py

## Recognition

Launch reco.py 
