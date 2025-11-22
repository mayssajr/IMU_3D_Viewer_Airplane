from pyqtgraph.Qt import QtCore, QtWidgets
import pyqtgraph.opengl as gl
import numpy as np
import serial

# --- CONFIGURATION UART ---
ser = serial.Serial('COM5', 115200, timeout=1)   # adapte le port COM

# --- APPLICATION ET FENÊTRE 3D ---
app = QtWidgets.QApplication([])
w = gl.GLViewWidget()
w.show()
w.setWindowTitle('IMU 3D Viewer - Avion (External View)')
w.setCameraPosition(distance=20, elevation=20, azimuth=45)

# --- MODELE SIMPLIFIÉ D’AVION (fuselage + ailes) ---
verts = np.array([
    [0, 0, 1],       # Nez
    [-0.5,-0.2,-1],  # Queue bas gauche
    [0.5,-0.2,-1],   # Queue bas droite
    [-0.5, 0.2,-1],  # Queue haut gauche
    [0.5, 0.2,-1],   # Queue haut droite
    [0,-1.5,-0.5],   # Aile gauche
    [0, 1.5,-0.5]    # Aile droite
])

faces = np.array([
    [0,1,2],    # Fuselage bas
    [0,3,4],    # Fuselage haut
    [0,1,3],    # Coté gauche
    [0,2,4],    # Coté droit
    [1,2,5],    # Aile gauche
    [3,4,6]     # Aile droite
])

plane = gl.GLMeshItem(
    vertexes=verts,
    faces=faces,
    color=(1, 0, 0, 0.8),
    smooth=False,
    drawEdges=True,
    edgeColor=(1,1,1,1)
)
w.addItem(plane)

# --- AXES XYZ ---
axis = gl.GLAxisItem()
axis.setSize(5, 5, 5)
w.addItem(axis)

# --- VARIABLES DE LISSAGE ---
smoothed_roll  = 0.0
smoothed_pitch = 0.0
smoothed_yaw   = 0.0
alpha = 0.05   # 0 = très fluide, 1 = pas de lissage

# --- MISE À JOUR DE L’AVION ---
def update():
    global smoothed_roll, smoothed_pitch, smoothed_yaw

    line = ser.readline().decode('utf-8', errors='ignore').strip()

    if line:
        try:
            roll, pitch, yaw = map(float, line.split(","))

            # Lissage anti-vibration
            smoothed_roll  = alpha * roll  + (1 - alpha) * smoothed_roll
            smoothed_pitch = alpha * pitch + (1 - alpha) * smoothed_pitch
            smoothed_yaw   = alpha * yaw   + (1 - alpha) * smoothed_yaw

            # Conversion en radians
            r, p, y = np.radians([smoothed_roll, smoothed_pitch, smoothed_yaw])

            # Rotations Z → Y → X
            Rx = np.array([[1,0,0],
                           [0,np.cos(r),-np.sin(r)],
                           [0,np.sin(r), np.cos(r)]])
            
            Ry = np.array([[ np.cos(p),0,np.sin(p)],
                           [0,1,0],
                           [-np.sin(p),0,np.cos(p)]])
            
            Rz = np.array([[np.cos(y),-np.sin(y),0],
                           [np.sin(y), np.cos(y),0],
                           [0,0,1]])

            R = Rz @ Ry @ Rx

            rotated = verts @ R.T
            plane.setMeshData(vertexes=rotated, faces=faces)

        except:
            pass

# --- TIMER 50 Hz ---
timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(20)

# --- EXECUTION ---
QtWidgets.QApplication.instance().exec_()
