import math
import sys
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import open3d as o3d
import cv2
import pykitti
from datetime import datetime


# ========= SETUP =========
basedir = ''

# Données du dataset
date = '2011_09_26'
drive = '0048'

# Load the data
dataset = pykitti.raw(basedir, date, drive, frames=range(0, 21, 1))

# Récupération des données de calibration
calib_data = dataset.calib

# Récupération des données de temps pour chaque image
timestamps = dataset.timestamps

# On récupère les informations de déplacement
oxts = dataset.oxts


# ========= RECUPERATION DE MATRICES ========= #
P2 = np.array(calib_data.P_rect_20) # Matrice de projection de la caméra 2
P2 = np.insert(P2, 2, 0, axis=0)
P2[2,2] = 1
R0_rect = np.array(calib_data.R_rect_00) # Matrice de rectification
Tr_velo_to_cam = np.array(calib_data.T_cam0_velo_unrect) # Matrice de transformation du velodyne au repère de la caméra (non rectifiée)


# ========= VISUALISATION DU NUAGE DE POINTS ========= #
# TODO : transposer les points par rapport au deplacement du lidar dans le temps
# TODO : Z-buffer, verification de cohérence entre les 2 caméras
# TODO : probleme, la distance parcourue entre les images est pas linear, il faut la calculer dans une direction bien précise

def measure(lat1, lon1, lat2, lon2):
    R = 6378.137  # Radius of earth in KM
    dLat = math.radians(lat2 - lat1)
    dLon = math.radians(lon2 - lon1)
    a = math.sin(dLat / 2) * math.sin(dLat / 2) + \
        math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * \
        math.sin(dLon / 2) * math.sin(dLon / 2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    d = R * c
    return d * 1000  # meters

# On sauvegarde le nuage de points normal
point_cloud = o3d.geometry.PointCloud()

# On sauvegarde le nuage de points coloré
point_cloud_colored = o3d.geometry.PointCloud()

# Obtenez le premier horodatage comme temps de départ
start_time = timestamps[0]

cam = None


# Calculez le temps écoulé en secondes pour chaque horodatage par rapport au temps de départ
time_seconds = [(timestamp - start_time).total_seconds() for timestamp in timestamps]

for i in range(1, 21) :
    # Récupération des données du velodyne (nuage de points)
    third_velo = dataset.get_velo(i)
    points = third_velo[:, :3]

    # Récupération de l'image de la caméra 2
    img = dataset.get_cam2(i)

    # On calcul la distance parcourue entre les images
    lat, lon, alt = oxts[i].packet.lat, oxts[i].packet.lon, oxts[i].packet.alt
    old_lat, old_lon, old_alt = oxts[i-1].packet.lat, oxts[i-1].packet.lon, oxts[i-1].packet.alt
    distance = measure(old_lat, old_lon, lat, lon)
    #print(f"Distance parcourue entre les images {i-1} et {i} : {displacement:.2f} mètres, Altitude : {alt - old_alt:.2f} mètres")
    old_lat, old_lon = lat, lon

    # Interpolation de la distance parcourue avec l'accélération dans la direction x
    delta_x = distance * (1 - oxts[i].packet.al) / 2  # Utilisez l'accélération linéaire en x
    
    # Interpolation de la distance parcourue avec l'accélération dans la direction z
    delta_z = distance * (1 + oxts[i].packet.af) / 2  # Utilisez l'accélération linéaire en z

    print(f"Distance parcourue entre les images {i-1} et {i} x : {delta_x:.2f} mètres, z : {delta_z:.2f}, Altitude : {alt - old_alt:.2f} mètres")

    # On incrémente les coordonnées z avec le déplacement vers l'avant, y avec le déplacement vertical
    #print("Points avant : ", points[0])
    #points[:, 0] += delta_z - 70 # Z -60 pour recentrer le nuage de points
    #points[:, 0] -= 70
    #points[:, 1] += alt - old_alt # Y
    #points[:, 2] += delta_x # X
    #print("Points après : ", points[0])

    points[:, 2] += delta_z # Z -60 pour recentrer le nuage de points
    points[:, 1] += alt - old_alt # Y
    points[:, 0] += delta_x # X


    # ======= POINTS CAMERA ======= #
    # On ajoute une colonne de 1 pour la multiplication matricielle
    velo = np.insert(points,3,1,axis=1).T
    velo = np.delete(velo,np.where(velo[0,:]<0),axis=1)

    # On multiplie les matrices pour obtenir les coordonnées dans le repère de la caméra
    cam = P2 @ R0_rect @ Tr_velo_to_cam 
    cam = cam @ velo
    cam = np.delete(cam,np.where(cam[2,:]<0),axis=1) # On retire les points trop eloignés

    # On récupère les coordonnées u, v
    cam[:2] /= cam[3,:]

    # ======= VISUALISATION DU NUAGE DE POINTS ======= #
    # on ajoute une colonne de 1 pour les coordonnées homogènes
    points_homogeneous = np.hstack((points, np.ones((points.shape[0], 1))))

    # On transforme les coordonnées des points du Velodyne dans le repère de la caméra
    cam_coords = P2 @ R0_rect @ Tr_velo_to_cam @ points_homogeneous.T

    # Obtenez les coordonnées u, v en divisant par la coordonnée z
    cam_coords[:2] /= cam_coords[3,:] # On divise par la coordonnée z
    u_v = cam_coords[:2]

    # Récupèration les dimensions de l'image de la caméra
    IMG_W, IMG_H = img.size

    # Filtrage des points à l'intérieur des dimensions de l'image de la caméra
    valid_indices = np.where((u_v[0] >= 0) & (u_v[0] < IMG_W) & (u_v[1] >= 0) & (u_v[1] < IMG_H))

    # On récupère uniquement les points qui sont visibles par la caméra
    visible_points = points[valid_indices]

    # On supprime les points derrière le lidar
    visible_points = visible_points[visible_points[:, 0] >= 0]


    # On filtre les points qui sont en dehors de l'image
    u,v,z = cam[:3]
    u_out = np.logical_or(u<0, u>IMG_W)
    v_out = np.logical_or(v<0, v>IMG_H)
    outlier = np.logical_or(u_out, v_out)
    cam = np.delete(cam,np.where(outlier),axis=1)


    # On traspose les points pour obtenir le bon format de données
    point_cloud_tmp = o3d.geometry.PointCloud()
    point_cloud_tmp.points = o3d.utility.Vector3dVector(visible_points)

    # On ajoute les points au nuage de point
    point_cloud.points.extend(point_cloud_tmp.points)


    #======= Coloration du nuage de point =========
    # On récupère les coordonnées u, v
    u = cam[0].astype(int)
    v = cam[1].astype(int)

    # On convertit les images img en array (Image en mpimg) 
    img_array = np.array(img)

    # Récupération des couleurs correspondantes dans l'image en vérifiant que les points sont dans l'image
    colors = [img_array[v[i], u[i]] / 255 for i in range(len(u)) if u[i] < IMG_W and v[i] < IMG_H] # Normalisation des couleurs entre 0 et 1

    # Récupérer les couleurs correspondantes dans l'image
    #colors = [img_array[v[i], u[i]] / 255 for i in range(len(u))] # Normaliser les couleurs entre 0 et 1

    point_cloud_tmp.colors = o3d.utility.Vector3dVector(np.array(colors))
    point_cloud_colored.colors.extend(point_cloud_tmp.colors)
    point_cloud_colored.points.extend(point_cloud_tmp.points)


# On enregistre le nuage de point
o3d.io.write_point_cloud("./files_result/point_cloud_github_multi.ply", point_cloud)


# On enregistre le nuage de point coloré
o3d.io.write_point_cloud("./files_result/point_cloud_github_multi_colored.ply", point_cloud_colored)