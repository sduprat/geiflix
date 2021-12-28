from flask import Flask, request, jsonify, json, redirect, url_for, render_template
#from turbo_flask import Turbo
import time, threading

app = Flask(__name__)

#turbo = Turbo(app)

@app.route("/<coor>")
def coord(coor):
   return render_template("map.html", coordinates = coor)

@app.route("/pi", methods = ["POST", "GET"]) 
def pi():
    if request.method == "POST":
        pi_data = request.json
        print(f"Value on the server {pi_data}")
        with open("data.txt", "a") as data_file:
            json.dump(pi_data, data_file)
        return jsonify(pi_data)
    else:
       # threading.Timer(2, pi_refresh).start()
        pi_data = None
        with open("data.txt", "r") as data_file:
             pi_data = data_file.read()
             print(pi_data[-1])
             temp1 = pi_data.split("{")
             temp2 = temp1[-1].split("}")
             last_value = temp2[0] 
             print(f"lassssst value : {last_value}")
        return redirect(url_for("coord", coor = "{" + last_value + "}"))

#@app.before_first_request
#def before_first_request():
#    threading.Thread(target=update_load).start()

#def update_load():
#    with app.app_context():
#       while True:
#            time.sleep(5)
#            turbo.push(turbo.replace(render_template("index.html"), "load"))

if __name__ == "__main__":
    app.run(debug=True)
