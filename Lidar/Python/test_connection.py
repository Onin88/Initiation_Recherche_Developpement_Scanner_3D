from pylsl import StreamInlet, resolve_stream

# Résolution du flux de données LSL provenant du lidar Velodyne
streams = resolve_stream('type', 'Velodyne')

# Création d'une prise d'entrée pour le flux de données
inlet = StreamInlet(streams[0])

# Lecture des données du flux pendant un certain temps (par exemple, 10 secondes)
duration = 10  # en secondes
start_time = time.time()

while (time.time() - start_time) < duration:
    # Lecture de l'échantillon du flux
    sample, timestamp = inlet.pull_sample()
    
    # Traitement de l'échantillon ici
    print(sample)  # Exemple : imprimez les données de l'échantillon
