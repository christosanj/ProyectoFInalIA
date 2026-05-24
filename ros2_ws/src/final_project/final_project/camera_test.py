from ultralytics import YOLO


model = YOLO("yolo11n.pt")

#results = model("https://ultralytics.com/images/bus.jpg")
results = model(source="/home/juan/Imágenes/PruebaSalida.png")

# Visualize the results
for result in results:
    result.show()
