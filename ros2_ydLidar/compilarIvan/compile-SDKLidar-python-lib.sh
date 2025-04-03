#!/bin/bash

# Guardar el directorio actual
CURRENT_DIR=$(pwd)

# Instalar dependencias necesarias
sudo apt update && sudo apt install -y \
    cmake \
    pkg-config \
    python3 \
    python3-dev \
    python3-pip \
    swig \
    git \
    build-essential \
    python3-setuptools \
    python3-wheel

# Clonar el repositorio en /tmp
cd /tmp || exit
rm -rf YDLidar-SDK  # Asegurar que no hay una versión previa

git clone https://github.com/YDLIDAR/YDLidar-SDK.git
mkdir -p YDLidar-SDK/build && cd YDLidar-SDK/build || exit

# Compilar el SDK
cmake ..
make -j$(nproc)

# Generar el paquete wheel de Python
cd /tmp/YDLidar-SDK || exit
python3 setup.py bdist_wheel --universal

# Copiar el wheel al directorio original
cp dist/*.whl "$CURRENT_DIR"

echo "Compilación y empaquetado completados. El archivo wheel ha sido copiado a: $CURRENT_DIR"
