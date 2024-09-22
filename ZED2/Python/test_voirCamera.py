import pyzed.sl as sl
import cv2
import numpy as np

def main():
    zed = sl.Camera()

    # Initialiser la caméra ZED2
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720  # Vous pouvez ajuster la résolution selon vos besoins

    err = zed.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS:
        print("Camera Open : "+repr(err)+". Exit program.")
        exit()
    
    

    # Configurer les paramètres de la caméra
    runtime_params = sl.RuntimeParameters()
    mat = sl.Mat()

    obj_param = sl.ObjectDetectionParameters()
    obj_param.enable_tracking=True
    obj_param.enable_segmentation=True
    obj_param.detection_model = sl.OBJECT_DETECTION_MODEL.MULTI_CLASS_BOX_MEDIUM

    if obj_param.enable_tracking :
        positional_tracking_param = sl.PositionalTrackingParameters()
        #positional_tracking_param.set_as_static = True
        zed.enable_positional_tracking(positional_tracking_param)

    print("Object Detection: Loading Module...")


    err = zed.enable_object_detection(obj_param)
    if err != sl.ERROR_CODE.SUCCESS :
        print("Enable object detection : "+repr(err)+". Exit program.")
        zed.close()
        exit()

    objects = sl.Objects()
    obj_runtime_param = sl.ObjectDetectionRuntimeParameters()
    obj_runtime_param.detection_confidence_threshold = 40


    # Boucle principale
    while True:
        # Capturer une image
        if zed.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:
            # Obtenir l'image gauche
            zed.retrieve_image(mat, sl.VIEW.LEFT)
            image = mat.get_data()

            # Obtenir les informations sur l'objet 3D
            zed.retrieve_objects(objects, obj_runtime_param)

            # Afficher l'image
            cv2.imshow("Image", image)

            # Afficher les objets 3D
            for obj in objects.object_list:
                print(f"ID: {obj.id}, Label: {obj.label}, Confidence: {obj.confidence}")

            # Attendre une touche pour quitter
            key = cv2.waitKey(30)
            if key == 27:  # Appuyez sur la touche 'Esc' pour quitter
                break

    # Fermer la caméra ZED2
    zed.close()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()