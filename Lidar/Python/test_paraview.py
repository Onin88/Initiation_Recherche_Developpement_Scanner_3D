from paraview.simple import smp
from lidarview.simple import lv

# Ouvrir le flux du capteur Velodyne
stream = lv.OpenSensorStream("VLP-16.xml", "Velodyne Meta Interpreter")

# Afficher le flux dans ParaView
smp.Show(stream)

# Démarrer la visualisation
stream.Start()