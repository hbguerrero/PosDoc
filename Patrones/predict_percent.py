import tensorflow as tf
import tensorflow_hub as hub
from PIL import Image
import cv2
import numpy as np

# Cargar el modelo con capa personalizada
modelo = tf.keras.models.load_model(
    'bananaLSD.h5',
    custom_objects={'KerasLayer': hub.KerasLayer}
)

CLASES = ['cordana', 'healthy', 'pestalotiopsis', 'sigatoka']

def categorizar(path):
    img = Image.open(path).convert('RGB')  # Asegura 3 canales
    img = np.array(img).astype(float) / 255.0
    img = cv2.resize(img, (224, 224))
    
    predicciones = modelo.predict(img.reshape(-1, 224, 224, 3))[0]  # shape: (4,)
    
    # Mapear clase a probabilidad
    resultado = {CLASES[i]: float(predicciones[i]) for i in range(len(CLASES))}
    
    return resultado

if __name__ == "__main__":
    resultado = categorizar("content/field_images/balon.jpg")
    print("Probabilidades por clase:")
    for clase, prob in resultado.items():
        print(f"{clase}: {prob*100:.1f}%")
