import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import os
import math
from datetime import datetime
from ultralytics import YOLO
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

NAME = "CHRISTOPHER SAN JUAN FLORES"

CLASS_MAP = {
    "bottle"       : "coke_can",
    "book"         : "cheezit_big_original",
    "box"          : "cheezit_big_original",
    "cup"          : "trash_can",
}

OBJECTS_NEEDED = ["coke_can", "cheezit_big_original", "trash_can"]


class VisionNode(Node):

    def __init__(self):
        super().__init__('vision_node')
        print("Cargando YOLO...")
        self.model = YOLO("yolo11n.pt")
        self.found_objects = [False, False, False]   
        self.confidence_threshold = 0.20
        self.br = CvBridge()

        self.sub_img = self.create_subscription(
            Image, '/camera/image_raw', self.callback_img, 1)
        self.pub_stop = self.create_publisher(Bool, '/stop_exploration', 1)

        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.output_folder = 'salida_evidencia'
        os.makedirs(self.output_folder, exist_ok=True)

        self.txt_path = os.path.join(self.output_folder, 'hallazgos.txt')
        with open(self.txt_path, 'w') as f:
            f.write("=== Hallazgos Proyecto Final 2026-2 ===\n\n")

        print("Buscando:", OBJECTS_NEEDED)

    def get_pose_string(self):
        try:
            t = self.tf_buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time())
            x = t.transform.translation.x
            y = t.transform.translation.y
            return f"x={x:.2f}, y={y:.2f}"
        except TransformException:
            return "posicion desconocida"

    def callback_img(self, msg):
        if all(self.found_objects):
            return

        try:
            img_bgr = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"cv_bridge error: {e}")
            return

        results = self.model(img_bgr, verbose=False)
        annotated = results[0].plot()

        for box in results[0].boxes:
            cls_id = int(box.cls[0])
            conf   = float(box.conf[0])
            name   = self.model.names[cls_id]

            if conf < self.confidence_threshold:
                continue

            obj_name = CLASS_MAP.get(name.lower(), None)
            if obj_name is None:
                continue

            idx = OBJECTS_NEEDED.index(obj_name)
            if self.found_objects[idx]:
                continue   # ya lo encontramos antes

            self.found_objects[idx] = True
            pose_str  = self.get_pose_string()
            timestamp = datetime.now().strftime("%H-%M-%S")

            print(f"[ENCONTRADO] {obj_name} (clase YOLO: {name}, conf={conf:.2f}) | {pose_str}")

            x1, y1, x2, y2 = map(int, box.xyxy[0])
            label = f"{obj_name}  {conf:.2f}"
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 4)
            cv2.rectangle(annotated,
                          (x1, y1 - 35), (x1 + len(label) * 13, y1),
                          (0, 255, 0), -1)
            cv2.putText(annotated, label,
                        (x1 + 4, y1 - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 2)

            img_name = f"{obj_name}_{timestamp}.png"
            cv2.imwrite(os.path.join(self.output_folder, img_name), annotated)

            with open(self.txt_path, "a") as f:
                f.write(f"Objeto   : {obj_name}\n")
                f.write(f"Clase    : {name}  (conf={conf:.2f})\n")
                f.write(f"Posicion : {pose_str}\n")
                f.write(f"Hora     : {timestamp}\n\n")

            if all(self.found_objects):
                print("Todos los objetos encontrados. Deteniendo robot.")
                self.pub_stop.publish(Bool(data=True))
                break   # sale del loop de boxes, NO del proceso

        cv2.imshow("Camara Robot", annotated)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()