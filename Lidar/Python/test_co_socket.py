import struct
import numpy as np
import socket

def process_packet(data):
    # Traitement des données du paquet UDP pour extraire les coordonnées XYZ des points du nuage de points
    points = []
    # Ajoutez votre logique de traitement ici

def main():
    # Configuration de la connexion UDP sur le port 2368
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.bind(('0.0.0.0', 2368))  # Écoute sur toutes les interfaces
    
    try:
        while True:
            # Réception des paquets UDP
            data, _ = udp_socket.recvfrom(2048)  # Taille du buffer à adapter selon vos besoins
            process_packet(data)
    except KeyboardInterrupt:
        print("Arrêt de la capture.")
        udp_socket.close()

if __name__ == "__main__":
    main()
