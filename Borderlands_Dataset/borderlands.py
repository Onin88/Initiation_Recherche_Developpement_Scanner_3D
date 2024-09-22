import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d
import pykitti
from PIL import Image
import re


# ========= SETUP ========= #
# Données du dataset
dir = 'Data'
nb_frame = 7

# On charge les points du fichier .txt au format x y z
points = np.loadtxt(f'./{dir}/PointCloud/pointcloud.txt', delimiter=' ', dtype=np.float32)

# On ajoute une quatrième coordonnée à chaque point
points_homogeneous = np.concatenate([points, np.ones((points.shape[0], 1))], axis=1)


# On crée le nuage de points normal
point_cloud = o3d.geometry.PointCloud()
point_cloud.points = o3d.utility.Vector3dVector(points)

# On crée le nuage de points coloré
point_cloud_colored = o3d.geometry.PointCloud()
point_cloud_colored.points = o3d.utility.Vector3dVector(points)

# On initialise les couleurs
colors = np.zeros((points_homogeneous.T.shape[1], 3))


# Fonction pour lire les matrices de transformation
def read_matrices(filename):
    with open(filename, 'r') as f:
        content = f.read()

    matrices = {}
    for matrix in re.split(r'\n(?=\w)', content):
        lines = matrix.split('\n')
        name = lines[0].strip(': ')
        values = []
        for line in lines[1:]:
            # Supprimez les espaces supplémentaires et ignorez les caractères non numériques
            line_values = re.findall(r'[\d\.\-e]+', line)
            if line_values:
                values.append(list(map(float, line_values)))
        matrices[name] = np.array(values)

    return matrices['Model'].T, matrices['View'].T, matrices['Persp'].T


# On charge les datas de la frame 0 à 7 avec un pas de 1
for i in range(0, nb_frame) :
    # Récupération de l'image actuelle
    img = Image.open(f'./{dir}/Pictures/screenshot_00{i}.png')

    # Récupération des matrices Model, View et Persp
    model, camera, perspective = read_matrices(f'./{dir}/Matrix/screenshot_00{i}.txt')

    # On multiplie les matrices
    projection = perspective @ camera @ model @ points_homogeneous.T

    # Taille de l'image
    IMG_W,IMG_H = img.size

    # Projetez les points dans l'espace de l'image
    projection[0,:] = projection[0,:] * IMG_W / (2*projection[3,:]) + IMG_W / 2
    projection[1,:] = -(projection[1,:] * IMG_H / (2*projection[3,:])) + IMG_H / 2
    projection[2,:] *= IMG_W

    # On convertit les images img en array (Image en mpimg) 
    img_array = np.array(img)


    # ======== Z-BUFFER ======== #
    # Initialiser le Z-buffer et les points
    zbuffer = np.full((IMG_W, IMG_H), np.inf)
    valid_points = []
        
    # On parcours tous les points projetés
    for j in range(projection.shape[1]):
        u, v, _, _ = projection[:, j].astype(int)
        _, _, z, _ = projection[:, j]

        # Limites de l'image
        if 0 <= u < IMG_W and 0 <= v < IMG_H:
            # On vérifie si la profondeur du point est inférieure à la valeur actuelle dans le Z-buffer
            if z < zbuffer[u, v]:
                zbuffer[u, v] = z

                # on ajoute le point à la liste des points valides
                valid_points.append(projection[:, j])
                colors[j] = img_array[v, u] / 255.0

    # on convertit les points valides en array
    valid_points = np.array(valid_points).T
    # ======== Z-BUFFER ======== #


    # On récupère les coordonnées u, v
    u, v, z = valid_points[:3].astype(int)

    # On génère une carte de profondeur qu'on affiche par dessus l'image
    plt.clf()  # efface le graphique précédent
    plt.axis([0,IMG_W,IMG_H,0])
    plt.imshow(img)
    plt.scatter([u],[v],c=[z],cmap='rainbow_r',alpha=0.5,s=2)
    plt.title(f'img_profondeur_{i}')
    plt.savefig(f'./Results/img_profondeur_{i}.png',bbox_inches='tight')
    '''if i == 0:
        plt.show()''' # Affiche l'image si c'est la première frame

    
    # On affiche la progression en console de 0 à 100 % sous la forme [======     ] en effacaant la ligne précédente
    progress = int(i/nb_frame*100)
    print(f'Progression : [{"="*(int(progress/10)*2)}{" "*(20-int(progress/10)*2)}] {progress}%', end='\r')
    


# On enregistre le nuage de point
o3d.io.write_point_cloud("./Results/point_cloud.ply", point_cloud)

# On convertit les couleurs en open3d
colors_o3d = o3d.utility.Vector3dVector(np.array(colors))
point_cloud_colored.colors = colors_o3d

# On enregistre le nuage de point coloré
o3d.io.write_point_cloud("./Results/point_cloud_colored.ply", point_cloud_colored)

# On affiche la progression en console de 100 %
print(f'Progression : [{"="*20}] 100%')