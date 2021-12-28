import requests, random, time, threading

def client():
    data_from_pi = {"lat" : 1.46604, "lng" : 43.57076}
    response = requests.post("http://127.0.0.1:5000/pi", json = data_from_pi)
    if response.ok:
        print(response.json())
        #threading.Timer(2, client).start()

client()
