# 🏎️ Controlador Reactivo F1TENTH - Algoritmo Follow the Gap (FTG)

Este repositorio contiene la implementación completa y funcional del controlador reactivo **Follow the Gap (FTG)**, optimizado para la competencia de vehículos F1TENTH en ROS 2.

---

## ⚙️ Prerrequisitos de Ejecución

Para utilizar este controlador, se asume que el usuario ya tiene instalado y configurado el entorno de simulación F1TENTH.

* **Sistema Operativo:** Ubuntu (recomendado 20.04 o 22.04).
* **ROS 2:** Humble.
* **Workspace F1TENTH:** El entorno debe seguir la estructura estándar de un *workspace* de ROS 2 (e.g., `~/f1tenth_ws`) y tener las dependencias básicas instaladas (como se detalla en el repositorio base de la asignatura: [https://github.com/widegonz/F1Tenth-Repository](https://github.com/widegonz/F1Tenth-Repository)).

---

## 1. 🚀 Instrucciones de Instalación y Compilación

### Paso 1: Clonar el Repositorio

Abre una terminal y navega al directorio `src` de tu *workspace* de F1TENTH.

```bash
cd ~/F1Tenth-Repository/src
git clone [https://github.com/JeremyD111/ftg_budapest_jeremy.git](https://github.com/JeremyD111/ftg_budapest_jeremy.git)
```

### Paso 2: Compilar el Paquete

Navega a la raíz de tu workspace y compila

```bash
cd ~/F1Tenth-Repository/
colcon build
source install/setup.bash
```










