import pyzed.sl as sl
import cv2
import numpy as np

def main():

    zed = sl.Camera()

    # Initialiser la caméra ZED2
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD1080

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
            # Obtenir l'image couleur
            zed.retrieve_image(mat, sl.VIEW.LEFT)
            image = mat.get_data()

            # Obtenir les informations sur l'objet 3D avec les couleurs
            zed.retrieve_objects(objects, obj_runtime_param)

            # Afficher l'image couleur
            cv2.imshow("Image", image)

            # Afficher l'objet 3D avec les couleurs
            for obj in objects.object_list:
                print(f"ID: {obj.id}, Label: {obj.label}, Confidence: {obj.confidence}")

                # Dessiner le rectangle englobant sur l'image
                left_top = (int(obj.bounding_box_2d[0, 0]), int(obj.bounding_box_2d[0, 1]))
                right_bottom = (int(obj.bounding_box_2d[2, 0]), int(obj.bounding_box_2d[2, 1]))
                cv2.rectangle(image, left_top, right_bottom, (0, 255, 0), 2)

            # Attendre une touche pour quitter
            key = cv2.waitKey(30)
            if key == 27:  # Appuyez sur la touche 'Esc' pour quitter
                break

    # Fermer la caméra ZED2
    zed.close()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()