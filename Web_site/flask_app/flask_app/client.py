import requests, random, time, threading

def client():
    data_from_pi = {"longitude" : random.randint(0,100), "latitude" : random.randint(0,100)}
    response = requests.post("http://127.0.0.1:5000/pi", json = data_from_pi)
    if response.ok:
        print(response.json())
        threading.Timer(2, client).start()

client()
