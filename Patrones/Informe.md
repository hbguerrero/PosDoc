## Informe de Investigación

**Clasificación de enfermedades foliares del plátano mediante reentrenamiento de MobileNetV3**
*Enfoque práctico de transferencia de aprendizaje con modelos preentrenados*

---

### **1. Contexto y Motivación**

La producción de plátano en regiones tropicales enfrenta desafíos significativos debido a enfermedades foliares que afectan la salud del cultivo, reducen la productividad y requieren una atención fitosanitaria constante. Entre las enfermedades más frecuentes están la *Sigatoka negra*, la *Cordana* y la *Pestalotiopsis*, además de la necesidad de diferenciar entre hojas afectadas y hojas sanas.

En este trabajo se explora el uso de modelos de visión por computador preentrenados para asistir en la clasificación automática de imágenes de hojas de plátano, como apoyo al diagnóstico en campo. La investigación se orienta a validar si un modelo liviano como **MobileNetV3**, reentrenado con imágenes específicas del dominio agrícola, puede ser adaptado eficientemente sin necesidad de grandes recursos computacionales.

---

## **2. Conjunto de Datos y Procesamiento Previo**

### 2.1. Descripción del Dataset

El conjunto de datos empleado para esta investigación proviene de la plataforma [Mendeley Data](https://data.mendeley.com/datasets/9tb7k297ff/1), titulado *Banana Leaf Disease Image Dataset*, el cual fue desarrollado con el propósito de facilitar la investigación en detección automatizada de enfermedades en hojas de plátano. El dataset incluye un total de **7.001 imágenes** de hojas categorizadas en cuatro clases:

* **Sigatoka negra** (*Mycosphaerella fijiensis*): Enfermedad fúngica severa que produce manchas negras alargadas en las hojas y puede ocasionar necrosis parcial o total del tejido foliar.
* **Cordana** (*Cordana musae*): Se manifiesta como manchas circulares o elípticas de color marrón oscuro con bordes bien definidos.
* **Pestalotiopsis** (*Pestalotiopsis spp.*): Infección que genera lesiones irregulares con bordes necróticos y anillos oscuros concéntricos.
* **Sana**: Imágenes de hojas en buen estado fitosanitario, sin síntomas visibles de enfermedades.

Cada clase cuenta con un número equilibrado de imágenes, tomadas en su mayoría en condiciones de campo, bajo iluminación natural. Las imágenes presentan variabilidad en términos de fondo, ángulo, escala y condiciones ambientales, lo cual las hace adecuadas para entrenar modelos robustos capaces de generalizar.

### 2.2. Formato y Resolución Original

Las imágenes originales se encuentran en formato JPEG, con dimensiones variadas (por ejemplo, desde 400×300 px hasta 1200×800 px). Para garantizar la compatibilidad con el modelo MobileNetV3, que requiere una entrada de tamaño fijo, fue necesario realizar un proceso de **redimensionamiento sistemático**.

---

### 2.3. Procesamiento y Acondicionamiento de Datos

Con el objetivo de adaptar las imágenes al pipeline de entrenamiento y maximizar la eficiencia del modelo, se aplicaron los siguientes pasos de preprocesamiento utilizando herramientas de procesamiento de imágenes con **OpenCV** y funciones estándar de `tf.keras.preprocessing.image`:

#### 2.3.1. Redimensionamiento

Todas las imágenes fueron convertidas al tamaño estándar de entrada esperado por MobileNetV3, es decir:

* **Dimensiones finales**: `224 x 224 píxeles`
* **Interpolación usada**: Bilineal
* **Formato de color**: RGB

Este paso asegura una entrada homogénea y aprovecha las capas convolucionales preentrenadas del modelo.

#### 2.3.2. Normalización

Cada imagen fue escalada para que sus valores de píxeles estén en el rango `[0, 1]`, mediante la división de cada canal por 255. Esto mejora la estabilidad numérica del entrenamiento.

```python
img = img.astype('float32') / 255.0
```

#### 2.3.3. Aumento de Datos (Data Augmentation)

Dado el tamaño relativamente modesto del dataset y la variabilidad natural en las condiciones de captura (ángulo, sombra, fondo), se implementaron técnicas de aumento de datos durante la fase de entrenamiento. Estas técnicas tienen como objetivo **incrementar la capacidad de generalización del modelo** sin necesidad de recolectar imágenes adicionales.

Las transformaciones aplicadas incluyen:

* **Rotaciones aleatorias** (hasta ±20°)
* **Volteo horizontal** (probabilidad 0.5)
* **Zoom aleatorio** (hasta un 20%)
* **Ajustes de brillo** (rango 0.8 a 1.2)
* **Desplazamientos verticales y horizontales menores** (hasta 10%)

Estas transformaciones se aplicaron de forma dinámica (en tiempo real) durante el entrenamiento mediante la clase `ImageDataGenerator` de Keras, lo que permite al modelo ver variaciones distintas de las imágenes en cada época sin modificar los datos originales.

---

### 2.4. División del Conjunto de Datos

Para asegurar una evaluación confiable del modelo, el dataset fue segmentado en dos subconjuntos:

* **80%** para entrenamiento (`train`)
* **20%** para validación (`validation`)

La división se realizó de forma estratificada, garantizando que la distribución de clases se mantuviera balanceada en ambos subconjuntos. Esta estrategia permite evaluar la capacidad del modelo de generalizar a nuevas muestras sin sesgo de clase.

---

### 2.5. Codificación de Etiquetas

Dado que la salida del modelo consiste en una capa `Dense(4)` con activación `softmax`, las etiquetas fueron codificadas utilizando **one-hot encoding**, lo que convierte las clases categóricas en vectores binarios de dimensión 4. Por ejemplo:

```text
"Sigatoka"       → [1, 0, 0, 0]
"Cordana"        → [0, 1, 0, 0]
"Pestalotiopsis" → [0, 0, 1, 0]
"Sana"           → [0, 0, 0, 1]
```

Esta codificación es compatible con la función de pérdida `categorical_crossentropy` empleada durante la compilación del modelo.

---

### 2.6. Observaciones sobre la calidad del dataset

A pesar de la buena calidad general del dataset, se identificaron algunos desafíos:

* **Variabilidad en el fondo y la iluminación**: Algunas imágenes contienen fondos con elementos visuales no relacionados (como suelos, manos humanas o herramientas), lo que podría introducir ruido durante el aprendizaje.
* **Sobreposición de síntomas**: Existen casos donde una hoja muestra características de más de una enfermedad, aunque esté etiquetada con una sola clase.
* **Resolución desigual**: Aunque todas las imágenes fueron redimensionadas, la diferencia en calidad original puede afectar la nitidez de los síntomas más sutiles.

A pesar de estos retos, el dataset ofrece una base sólida para entrenar modelos de clasificación robustos y representa condiciones realistas que el modelo podría encontrar en aplicaciones prácticas en campo.

---

## **3. Arquitectura y Estrategia de Reentrenamiento**

### 3.1. Elección de la Arquitectura: MobileNetV3-Large

Para la presente investigación se seleccionó la arquitectura **MobileNetV3-Large (versión 1, multiplicador de anchura 0.75)** como base para la clasificación de enfermedades foliares. MobileNetV3 representa la evolución de la familia de arquitecturas MobileNet, diseñada específicamente para dispositivos con recursos limitados, ofreciendo una alta eficiencia computacional sin comprometer significativamente la precisión.

Esta arquitectura fue introducida por Google AI en 2019 ([Howard et al., 2019](https://arxiv.org/abs/1905.02244)) y combina múltiples técnicas de optimización:

* **Bloques invertidos de residuals móviles (Mobile Inverted Bottleneck)**
* **Squeeze-and-Excitation (SE)** para recalibración de canales
* **Activaciones Hard-Swish**, una alternativa eficiente a `ReLU6`
* **Búsqueda automática de arquitectura (NAS)** para balancear precisión y latencia

Se utilizó la variante **Large** con un factor de anchura de 0.75 (`alpha=0.75`) para reducir el número total de parámetros y operaciones, favoreciendo su uso futuro en entornos embebidos o móviles sin aceleración por GPU.

---

### 3.2. Estrategia: Aprendizaje por Transferencia (Transfer Learning)

Dado que el entrenamiento de redes convolucionales profundas desde cero requiere grandes cantidades de datos y capacidad de cómputo, se adoptó una estrategia de **transfer learning**, que consiste en reutilizar pesos previamente aprendidos en un dominio amplio (en este caso, el conjunto **ImageNet**) y adaptarlos a una tarea específica con menor cantidad de datos.

#### 3.2.1. Feature Extraction (Extracción de Características)

Se optó por la técnica de **feature extraction**, que implica **congelar todas las capas convolucionales** del modelo base (es decir, no se actualizan durante el entrenamiento), conservando sus pesos originales entrenados sobre ImageNet. Estas capas actúan como un extractor de características visuales de alto nivel, capturando bordes, texturas, patrones y formas relevantes.

De esta manera, se entrena únicamente la nueva **capa de clasificación final**, lo que reduce significativamente el tiempo de entrenamiento, los requerimientos computacionales y el riesgo de sobreajuste en datasets pequeños.

---

### 3.3. Modificación de la Capa de Salida

El modelo preentrenado original está diseñado para clasificar entre 1.000 clases de ImageNet. Para adaptar este modelo a nuestro problema de clasificación de 4 clases, se reemplazó la capa densa final por una nueva capa `Dense(4)` con activación **softmax**, que permite modelar una distribución de probabilidad sobre las clases objetivo:

```python
from tensorflow.keras import layers

# Capa personalizada de salida
x = base_model.output
x = layers.GlobalAveragePooling2D()(x)
x = layers.Dense(4, activation='softmax')(x)
```

Este diseño permite al modelo emitir una predicción probabilística para cada imagen, asignando una puntuación a cada clase.

---

### 3.4. Compilación del Modelo

El modelo fue compilado utilizando el optimizador **Adam**, ampliamente utilizado por su eficacia en la convergencia rápida y adaptativa. Se utilizó la función de pérdida `categorical_crossentropy`, adecuada para clasificación multiclase con etiquetas codificadas como one-hot, y se eligió **accuracy** como métrica principal de evaluación:

```python
model.compile(
    optimizer='adam',
    loss='categorical_crossentropy',
    metrics=['accuracy']
)
```

#### Justificación técnica:

* **Adam** combina las ventajas de *AdaGrad* y *RMSProp*, ajustando la tasa de aprendizaje de manera adaptativa para cada parámetro, lo que resulta especialmente útil en redes profundas.
* **Categorical Crossentropy** es idónea cuando las clases son mutuamente excluyentes y se expresan en vectores binarios.
* La **métrica de precisión** permite monitorear de forma intuitiva la evolución del modelo durante el entrenamiento.

---

### 3.5. Configuración de Entrenamiento

La fase de entrenamiento fue diseñada para ser eficiente y reproducible, con los siguientes parámetros clave:

* **Batch size**: 32 imágenes por lote
* **Número de épocas**: 15
* **Entrenamiento sobre CPU**:


La duración del entrenamiento fue deliberadamente limitada a 15 épocas para evitar sobreajuste, dado que solo se entrena la capa final y el objetivo principal es validar la viabilidad del modelo preentrenado en el nuevo dominio de aplicación.

---

### 3.6. Arquitectura Final del Modelo

La arquitectura final del modelo puede resumirse de la siguiente forma:

| Componente               | Descripción                                                        |
| ------------------------ | ------------------------------------------------------------------ |
| Entrada (`224x224x3`)    | Imagen RGB preprocesada                                            |
| MobileNetV3-Large (0.75) | Red convolucional base preentrenada en ImageNet (capas congeladas) |
| GlobalAveragePooling2D   | Capa de reducción espacial para resumen de activaciones            |
| Dense(4, softmax)        | Capa final personalizada para clasificación en 4 clases            |

Este diseño logra un equilibrio adecuado entre **eficiencia computacional** y **capacidad discriminativa**, permitiendo su entrenamiento en tiempos reducidos y facilitando futuras implementaciones móviles o embebidas.

---

## **4. Resultados y Observaciones**

### 4.1. Desempeño General

El modelo MobileNetV3-Large, adaptado mediante la reconfiguración de su capa de salida y entrenado exclusivamente con imágenes del dominio agrícola, mostró un comportamiento estable y eficaz. A pesar de que se realizó un reentrenamiento parcial —limitado únicamente a la capa de clasificación— el modelo fue capaz de aprender y generalizar con una precisión destacable desde las primeras épocas de entrenamiento.

Al cabo de 15 épocas, el modelo alcanzó una precisión considerablemente alta sobre el conjunto de validación, evidenciando que las representaciones internas aprendidas durante su preentrenamiento en ImageNet fueron **altamente transferibles** al dominio específico de imágenes foliares de plátano, a pesar de las diferencias visuales entre ambos dominios.


---

### 4.2. Observaciones Clave

#### a. Eficiencia del Transfer Learning

Uno de los resultados más notables fue comprobar que el modelo pudo adaptarse correctamente a la tarea con un número **relativamente bajo de muestras** y sin necesidad de entrenar todas sus capas internas. Esto valida el principio del **aprendizaje por transferencia**, permitiendo capitalizar el conocimiento ya embebido en redes convolucionales profundas, incluso cuando el nuevo problema de clasificación difiere sustancialmente del original.

#### b. Comportamiento por Clase

El modelo fue particularmente eficiente en identificar imágenes de **hojas sanas**, así como casos evidentes de **sigatoka**, en los que las manchas son prominentes y fáciles de distinguir. Por el contrario, se observó un **mayor solapamiento entre las clases “Cordana” y “Pestalotiopsis”**, cuyas manifestaciones visuales pueden ser similares en fases iniciales.

Este tipo de ambigüedad visual es esperable en problemas reales de diagnóstico agrícola, donde las diferencias entre enfermedades son sutiles y pueden requerir información contextual adicional (como edad de la planta o patrón de propagación).

#### c. Robustez frente a Variaciones

Gracias al uso de técnicas de aumento de datos, el modelo mostró robustez frente a condiciones diversas como **ángulos de toma, variaciones de iluminación y fondos heterogéneos**. Esto refuerza su potencial de ser desplegado en aplicaciones prácticas en campo, donde estas condiciones no pueden ser controladas.

---

## **5. Conclusiones**

Este trabajo demuestra que los modelos de clasificación de imágenes basados en **redes convolucionales preentrenadas**, como MobileNetV3, pueden ser reaprovechados con éxito para tareas especializadas, como la **identificación de enfermedades foliares en cultivos de plátano**, con un esfuerzo mínimo en reentrenamiento.

Entre las conclusiones más relevantes, se destacan:

* La arquitectura **MobileNetV3-Large (0.75)**, optimizada para entornos móviles, ofrece un **equilibrio notable entre precisión y eficiencia computacional**, haciéndola ideal para contextos agrícolas con recursos limitados.
* El uso de **feature extraction** permitió reducir el tiempo de entrenamiento, simplificar la implementación y minimizar el riesgo de sobreajuste, al mismo tiempo que se obtuvo un modelo funcional con buen rendimiento.
* El dataset empleado, a pesar de su tamaño y ciertas limitaciones inherentes a la captura en campo, fue suficiente para validar el enfoque gracias a la **potencia de las representaciones extraídas por el modelo base**.
* Los resultados respaldan el potencial de usar esta técnica como base para una **aplicación móvil de diagnóstico rápido**, especialmente útil en zonas rurales donde no se dispone de conectividad constante o infraestructura de cómputo avanzada.

---

### Perspectivas futuras

* Se recomienda evaluar un **ajuste fino (fine-tuning) parcial** de las capas superiores del modelo para aumentar su sensibilidad ante clases con síntomas visuales similares.
* También sería beneficioso expandir el dataset con imágenes tomadas en distintas regiones y bajo diferentes condiciones ambientales para reforzar la generalización del modelo.
* Finalmente, se sugiere el desarrollo de un prototipo funcional embebido o móvil, que integre el modelo entrenado en una interfaz accesible para técnicos agrícolas y productores.

## **6. Referencias**

[1] [Dataset de imagenes](https://data.mendeley.com/datasets/9tb7k297ff/1)

[2] [Modelo preentrenado MobileNetV3-Large 0.75 v1](https://www.kaggle.com/models/google/mobilenet-v3/TensorFlow2/large-075-224-feature-vector/1)
