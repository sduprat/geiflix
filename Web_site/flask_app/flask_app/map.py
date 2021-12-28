from flask import Flask
import folium
import os
from folium import IFrame
import base64

app = Flask(__name__)

@app.route("/map_fire")
def base():
    map = folium.Map(
    	location=[43.57076, 1.46605],
        zoom_start = 12
    )
    
    Filename ='images/fire.jpeg'
    encoded = base64.b64encode(open(Filename, 'rb').read())
    html='<img src="data:image/jpeg;base64,{}">'.format 
    resolution, width, height = 75, 50, 5
    iframe = IFrame(html(encoded.decode('UTF-8')), width=(width*resolution) + 20, height=(height*resolution) + 20)
    popup = folium.Popup(iframe, max_width= 500)

    folium.Marker(
	location=[43.57076, 1.46605],
    zoom_start = 12,
	#popup= '<img src="fire.jpeg"/>',
    popup=popup,
	tooltip="Fire Alert",
	icon=folium.Icon(color="red", icon="fire")
    ).add_to(map)

    return map._repr_html_()

if __name__ == "__main__":
	app.run(debug=True)
