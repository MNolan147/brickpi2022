from datetime import datetime
#from types import None
from xml.sax.xmlreader import Locator
from flask import Flask, render_template, session, request, redirect, flash, url_for, jsonify, Response, logging
from interfaces import databaseinterface, camerainterface, soundinterface
import robot #robot is class that extends the brickpi class
import global_vars as GLOBALS #load global variables
import logging, time

#Creates the Flask Server Object
app = Flask(__name__); app.debug = True
SECRET_KEY = 'my random key can be anything' #this is used for encrypting sessions
app.config.from_object(__name__) #Set app configuration using above SETTINGS
logging.basicConfig(filename='logs/flask.log', level=logging.INFO)
GLOBALS.DATABASE = databaseinterface.DatabaseInterface('databases/RobotDatabase.db', app.logger)

#Log messages
def log(message):
    app.logger.info(message)
    return

#create a login page
@app.route('/', methods=['GET','POST'])
def login():
    if 'userid' in session:
        return redirect('/dashboard')
    message = ""
    if request.method == "POST":
        email = request.form.get("email")
        userdetails = GLOBALS.DATABASE.ViewQuery("SELECT * FROM users WHERE email = ?", (email,))
        log(userdetails)
        if userdetails:
            user = userdetails[0] #get first row in results
            if user['password'] == request.form.get("password"):
                session['userid'] = user['userid']
                session['permission'] = user['permission']
                session['name'] = user['name']
                return redirect('/dashboard')
            else:
                message = "Login Unsuccessful"
        else:
            message = "Login Unsuccessful"
    return render_template('login.html', data = message)    

# Load the ROBOT
@app.route('/robotload', methods=['GET','POST'])
def robotload():
    sensordict = None
    if not GLOBALS.CAMERA:
        log("LOADING CAMERA")
        try:
            GLOBALS.CAMERA = camerainterface.CameraInterface()
        except Exception as error:
            log("FLASK APP: CAMERA NOT WORKING")
            GLOBALS.CAMERA = None
        if GLOBALS.CAMERA:
            GLOBALS.CAMERA.start()
    if not GLOBALS.ROBOT: 
        log("FLASK APP: LOADING THE ROBOT")
        GLOBALS.ROBOT = robot.Robot(20, app.logger)
        GLOBALS.ROBOT.configure_sensors() #defaults have been provided but you can 
        GLOBALS.ROBOT.reconfig_IMU()
    if not GLOBALS.SOUND:
        log("FLASK APP: LOADING THE SOUND")
        GLOBALS.SOUND = soundinterface.SoundInterface()
        GLOBALS.SOUND.say("I am ready")
    sensordict = GLOBALS.ROBOT.get_all_sensors()
    return jsonify(sensordict)

# ---------------------------------------------------------------------------------------
# Dashboard
@app.route('/dashboard', methods=['GET','POST'])
def robotdashboard():
    if not 'userid' in session:
        return redirect('/')
    if not "missionId" in session:
        #return redirect("/mission")
        session["missionId"] = 1
    enabled = int(GLOBALS.ROBOT != None)
    return render_template('dashboard.html', robot_enabled = enabled )

#Used for reconfiguring IMU
@app.route('/reconfig_IMU', methods=['GET','POST'])
def reconfig_IMU():
    if GLOBALS.ROBOT:
        GLOBALS.ROBOT.reconfig_IMU()
        sensorconfig = GLOBALS.ROBOT.get_all_sensors()
        return jsonify(sensorconfig)
    return jsonify({'message':'ROBOT not loaded'})

#calibrates the compass but takes about 10 seconds, rotate in a small 360 degrees rotation
@app.route('/compass', methods=['GET','POST'])
def compass():
    data = {}
    if GLOBALS.ROBOT:
        data['message'] = GLOBALS.ROBOT.calibrate_imu(10)
    return jsonify(data)

@app.route('/sensors', methods=['GET','POST'])
def sensors():
    data = {}
    if GLOBALS.ROBOT:
        data = GLOBALS.ROBOT.get_all_sensors()
    return jsonify(data)

# YOUR FLASK CODE------------------------------------------------------------------------
@app.route('/seedatabase')
def seedatabase():
    data = GLOBALS.DATABASE.ViewQuery('SELECT * FROM ')
    return jsonify(data)

@app.route('/admin', methods=['POST','GET']) # admin page
def admin():
    if 'permission' in session:
        if session['permission'] != 'admin':
            return redirect('/')
    else:
        return redirect('/')


    """if request.method == 'POST':
        listofusers = request.form.getlist('selecteduser')
        for userid in listofusers:
            GLOBALS.DATABASE.ModifyQuery("DELETE FROM tblUsers WHERE userid = ? AND accounttype != 'admin'", (userid,))
"""
    #displays data
    tables={}
    tables['users']=GLOBALS.DATABASE.ViewQuery('SELECT * FROM users')
    tables['missions']=GLOBALS.DATABASE.ViewQuery(("""SELECT *
                                                FROM tblExperts
                                                ORDER BY expertid"""))
    tables['movements']=GLOBALS.DATABASE.ViewQuery("SELECT * FROM movements")
    tables['events']=GLOBALS.DATABASE.ViewQuery(("SELECT * FROM events"))
    
    #primary_keys = {'tblUsers': 'userid', 'tblExperts': 'expertid', 'tblSubjects': 'subjectid', 'tblExpertSubjects': 'esid', 'tblCertList': 'certname', 'tblExpertCertifications': 'ecid', 'tblBookings': 'bookingid'}
    return render_template('admin.html', tables=tables)

@app.route('/shoot', methods=['GET','POST'])
def shoot():
    data = {}
    if GLOBALS.ROBOT:
        GLOBALS.ROBOT.spin_medium_motor(-1000)
    return jsonify(data)    

@app.route('/moveforward', methods=['GET','POST'])
def moveforward():
    data = {}
    if GLOBALS.ROBOT:
        data["startTime"] = time.time()
        data["type"] = "forward"
        data["heading"] = GLOBALS.ROBOT.get_compass_IMU()
        GLOBALS.ROBOT.move_encoder()
        GLOBALS.DATABASE.ModifyQuery("INSERT INTO movements (startTime,type,heading,missionId) VALUES (?,?,?,?)",(data["startTime"],data["type"],data["heading"],session["missionId"]))
    return jsonify(data)  

@app.route('/movebackward', methods=['GET','POST'])
def movebackward():
    data = {}
    if GLOBALS.ROBOT:
        data["startTime"] = time.time()
        data["type"] = "backward"
        data["heading"] = GLOBALS.ROBOT.get_compass_IMU()
        GLOBALS.ROBOT.move_encoder(-1)
        GLOBALS.DATABASE.ModifyQuery("INSERT INTO movements (startTime,type,heading,missionId) VALUES (?,?,?,?)",(data["startTime"],data["type"],data["heading"],session["missionId"]))
    return jsonify(data)

@app.route('/rotateright', methods=['GET','POST'])
def rotateright():
    data = {}
    if GLOBALS.ROBOT:
        GLOBALS.ROBOT.rotate_power(15)
    return jsonify(data)

@app.route('/rotateleft', methods=['GET','POST'])
def rotateleft():
    data = {}
    if GLOBALS.ROBOT:
        GLOBALS.ROBOT.rotate_power(-15)
    return jsonify(data)

@app.route('/stop', methods=['GET','POST'])
def stop():
    data = {}
    if GLOBALS.ROBOT:
        if GLOBALS.ROBOT.CurrentCommand == "move_encoder" or GLOBALS.ROBOT.CurrentCommand == "move_distance_encoder":
            data["endTime"] = time.time()
            action = GLOBALS.DATABASE.ViewQuery("SELECT movementId FROM actions WHERE missionId = ? ORDER BY movementId DESC Limit 1",(session["missionId"],))
            GLOBALS.DATABASE.ModifyQuery("UPDATE movements SET endTime = ? WHERE movementId = ?", (data['endTime'],action))
        GLOBALS.ROBOT.stop_all()
    return jsonify(data) 

# mission view page
# allows medic manager to create a mission and save data for that mission
@app.route('/create-mission', methods=['GET', 'POST'])
def createmission():
    data = {}
    # if request method is POST
    if request.method == "POST":
        # get form data
        userid = session['userid']
        notes = request.form.get('medical-notes')
        location = request.form.get('location')
        if request.form.get('start-time'):
            starttime = request.form.get('start-time')
        else:
            starttime = time.time().timestamp()
        # insert into mission
        GLOBALS.DATABASE.ModifyQuery('INSERT INTO mission (location, notes, starttime, userid) VALUES (?,?,?,?)', (location,notes,starttime,userid))
        # select the mission id and save to session data
        # send mission history to page
    return render_template('createmission.html', data=data)

@app.route('/mission', methods=['GET','POST'])
def mission():
    data = {}
    data["missions"] = GLOBALS.DATABASE.ViewQuery("SELECT * FROM")
    return render_template('mission.html',data=data)

# allows medic manager to test that all sensors are working
@app.route('/sensor-view', methods=['GET','POST'])
def sensorview():
    if GLOBALS.ROBOT:
        data = GLOBALS.ROBOT.get_all_sensors()
    else:
        return redirect('/dashboard')
    return render_template('sensorview.html', data=data)







# -----------------------------------------------------------------------------------
# CAMERA CODE-----------------------------------------------------------------------
# Continually gets the frame from the pi camera
def videostream():
    """Video streaming generator function."""
    while True:
        if GLOBALS.CAMERA:
            frame = GLOBALS.CAMERA.get_frame()
            if frame:
                yield (b'--frame\r\n'
                        b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n') 
            else:
                return '', 204
        else:
            return '', 204 

#embeds the videofeed by returning a continual stream as above
@app.route('/videofeed')
def videofeed():
    if GLOBALS.CAMERA:
        log("FLASK APP: READING CAMERA")
        """Video streaming route. Put this in the src attribute of an img tag."""
        return Response(videostream(), mimetype='multipart/x-mixed-replace; boundary=frame') 
    else:
        return '', 204
        
#----------------------------------------------------------------------------
#Shutdown the robot, camera and database
def shutdowneverything():
    log("FLASK APP: SHUTDOWN EVERYTHING")
    if GLOBALS.CAMERA:
        GLOBALS.CAMERA.stop()
    if GLOBALS.ROBOT:
        GLOBALS.ROBOT.safe_exit()
    GLOBALS.CAMERA = None; GLOBALS.ROBOT = None; GLOBALS.SOUND = None
    return

#Ajax handler for shutdown button
@app.route('/robotshutdown', methods=['GET','POST'])
def robotshutdown():
    shutdowneverything()
    return jsonify({'message':'robot shutdown'})

#Shut down the web server if necessary
@app.route('/shutdown', methods=['GET','POST'])
def shutdown():
    shutdowneverything()
    func = request.environ.get('werkzeug.server.shutdown')
    func()
    return jsonify({'message':'Shutting Down'})

@app.route('/logout')
def logout():
    shutdowneverything()
    session.clear()
    return redirect('/')

#---------------------------------------------------------------------------
#main method called web server application
if __name__ == '__main__':
    try:
        app.run(host='0.0.0.0', port=5000, debug=True, threaded=True) #runs a local server on port 5000
    finally:
        if GLOBALS.ROBOT:
            GLOBALS.ROBOT.safe_exit()