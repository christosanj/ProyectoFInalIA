from ultralytics import YOLO
import cv2

model = YOLO("yolo11n.pt")

results = model("https://ultralytics.com/images/bus.jpg")

plot = results[0].plot() 
print (type(plot))
cv2.imshow("results",plot)
cv2.waitKey(0)

success = model.export(format="onnx")
