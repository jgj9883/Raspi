#!/usr/bin/python3
from flask import Flask, Response, request, render_template
import cv2
from controlServer import *

cam=cv2.VideoCapture(1)

if cam.isOpened()==False:
    print("cant open cam")
    exit()
    
cam.set(cv2.CAP_PROP_FRAME_WIDTH,320)
cam.set(cv2.CAP_PROP_FRAME_HEIGHT,240)
app = Flask(__name__, template_folder=".")

def generate():
    while True:
        ret,image = cam.read()
        jpegdata=cv2.imencode(".jpeg",image)[1].tobytes()
        string = "--MjpgBound\r\n"
        string += "Content-Type: image/jpeg\r\n"
        string += "Content-length: "+str(len(jpegdata))+"\r\n\r\n"
        yield (string.encode("utf-8") + jpegdata + "\r\n\r\n".encode("utf-8"))


# stream camera
@app.route('/stream')
def do_stream():
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=--MjpgBound')



#control rccar
@app.route('/motor', methods=['POST'])
def control_rccar():
    command=request.forms.get('command')
    print(command)
    if command == "GO":
        forward()
    elif command == "LEFT":
        left()
    elif command == "STOP":
        stop()
    elif command == "RIGHT":
        right()
    elif command == "BACK":
        backward()
    return ''


@app.route('/')
def do_route():
    return render_template("index.html")

if __name__ == '__main__':
    initMotors()
    app.run(host='192.168.0.5', port=8020)
