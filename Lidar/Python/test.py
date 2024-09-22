import struct
import numpy as np
from scapy.all import *

def process_packet(packet):
    if IP in packet and UDP in packet:
        #if '51844' == str(packet[UDP].sport).strip():
        print("Paquet : ", packet, " -- Port : '", packet[UDP].sport, "'")
        if packet[UDP].sport == 2368:  # Port UDP utilisé par le lidar Velodyne
            # Les données du lidar Velodyne se trouvent dans le champ de charge utile du paquet UDP
            lidar_data = packet[UDP].payload

            # Décodez les données du paquet UDP pour obtenir les coordonnées XYZ des points du nuage de points
            points = []
            for i in range(0, len(lidar_data), 100):  # Chaque paquet UDP contient 100 blocs de données
                print("Itération : ", i , " sur ", len(lidar_data))
                block = lidar_data[i:i+100]
                block_data = struct.unpack('<IHHHBBI', block[:20])  # Informations de l'en-tête de chaque bloc
                azimuth = (block_data[0] & 0x0000FFFF) / 100.0  # Angle d'azimut
                for j in range(0, 12, 3):
                    distance = block[j] / 500.0  # Distance
                    intensity = block[j+2]  # Intensité du retour
                    # Conversion des coordonnées sphériques en coordonnées cartésiennes
                    x = distance * np.cos(np.radians(azimuth)) * np.sin(np.radians(2 * block_data[1] / 100.0))
                    y = distance * np.sin(np.radians(azimuth)) * np.sin(np.radians(2 * block_data[1] / 100.0))
                    z = distance * np.cos(np.radians(2 * block_data[1] / 100.0))
                    points.append((x, y, z, intensity))

            # Convertissez les points en un tableau numpy pour un traitement ultérieur
            points_np = np.array(points)

            # Enregistrez le nuage de points dans un fichier PLY
            save_ply(points_np, "lidar_point_cloud.ply")
            print("Nuage de points enregistré avec succès dans 'lidar_point_cloud.ply'.")

def save_ply(points, filename):
    with open(filename, 'w') as f:
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write("element vertex {}\n".format(len(points)))
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")
        f.write("property uchar intensity\n")
        f.write("end_header\n")
        for point in points:
            f.write("{} {} {} {}\n".format(point[0], point[1], point[2], point[3]))

# Capture les paquets UDP
sniff(filter="udp and port 51844", prn=process_packet)
#sniff(filter="port 2368", prn=process_packet)
#sniff(filter="tcp and port 2368", prn=process_packet)