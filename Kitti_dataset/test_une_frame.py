import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d
import pykitti


# ========= SETUP ========= #
basedir = ''

# Données du dataset
date = '2011_09_26'
drive = '0048'
index_frame = 7

# On charge les datas des frames 0 à 21 avec un pas de 1
dataset = pykitti.raw(basedir, date, drive, frames=range(0, 21, 1))

# Récupération des données de calibration
calib_data = dataset.calib

# Récupération des données du velodyne
third_velo = dataset.get_velo(index_frame)

# Récupération de l'image de la caméra 2
img_cam2 = dataset.get_cam2(index_frame)

# Récupération des points du velodyne
points = third_velo[:, :3]


# ========= MATRICES ========= #
P2 = np.array(calib_data.P_rect_20) # Matrice de projection de la caméra 2
P2 = np.insert(P2, 2, 0, axis=0)
P2[2,2] = 1
R0_rect = np.array(calib_data.R_rect_00) # Matrice de rectification
Tr_velo_to_cam = np.array(calib_data.T_cam0_velo_unrect) # Matrice de transformation du velodyne au repère de la caméra (non rectifiée)



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
IMG_W, IMG_H = img_cam2.size

# Filtrage des points à l'intérieur des dimensions de l'image de la caméra
valid_indices = np.where((u_v[0] >= 0) & (u_v[0] < IMG_W) & (u_v[1] >= 0) & (u_v[1] < IMG_H))

# On récupère uniquement les points qui sont visibles par la caméra
visible_points = points[valid_indices]

# On supprime les points derrière le lidar
visible_points = visible_points[visible_points[:, 0] >= 0]


# ======= VISUALISATION ======= #
# On affiche l'image
plt.figure(figsize=(12,5),dpi=96,tight_layout=True)
plt.axis([0,IMG_W,IMG_H,0])
plt.imshow(img_cam2)

# On filtre les points qui sont en dehors de l'image
u,v,z = cam[:3]
u_out = np.logical_or(u<0, u>IMG_W)
v_out = np.logical_or(v<0, v>IMG_H)
outlier = np.logical_or(u_out, v_out)
cam = np.delete(cam,np.where(outlier),axis=1)

# On affiche les points sur l'image en fonction de leur profondeur
u,v,z = cam[:3]
plt.scatter([u],[v],c=[z],cmap='rainbow_r',alpha=0.5,s=2)
plt.title(f'img_profondeur_{index_frame}')
plt.savefig(f'./files_result/img_profondeur_{index_frame}.png',bbox_inches='tight')
plt.show()


#======= Enregistrement du nuage de points ======== #
# On sauvegarde le nuage de points normal
point_cloud = o3d.geometry.PointCloud()
point_cloud.points = o3d.utility.Vector3dVector(visible_points)

# On enregistre le nuage de point
o3d.io.write_point_cloud("./files_result/point_cloud_github.ply", point_cloud)


#======= Enregistrement du nuage de points coloré ========= #
# On récupère les coordonnées u, v 
u = cam[0].astype(int)
v = cam[1].astype(int)

# On convertit l'image img_cam2 en array (Image en mpimg) 
img_array = np.array(img_cam2)

# Récupération des couleurs correspondantes dans l'image en vérifiant que les points sont dans l'image
colors = [img_array[v[i], u[i]] / 255 for i in range(len(u)) if u[i] < IMG_W and v[i] < IMG_H] # Normalisation des couleurs entre 0 et 1
point_cloud.colors = o3d.utility.Vector3dVector(np.array(colors)) 

# On enregistre le nuage de point en couleur
o3d.io.write_point_cloud("./files_result/point_cloud_github_colored.ply", point_cloud)