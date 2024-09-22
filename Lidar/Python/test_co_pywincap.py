import pyshark

def packet_handler(pkt):
    print(pkt)

# Créer un capturateur de paquets en utilisant l'interface par défaut
capture = pyshark.LiveCapture(interface='eth0')

# Définir le gestionnaire de paquets pour traiter chaque paquet capturé
capture.apply_on_packets(packet_handler)
