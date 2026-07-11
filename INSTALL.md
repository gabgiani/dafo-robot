# Instalación

Esta guía cubre la instalación local del simulador y sus dependencias.

## Requisitos

- macOS o Linux.
- Python 3.
- Un entorno virtual en `.venv`.
- La carpeta `third_party/mujoco_menagerie` con los modelos Unitree.
- Opcional: `ffmpeg` para exportar video desde la demo de grasp.

## Dependencias Python

Crear el entorno e instalar dependencias:

```bash
cd /Users/gianig/dafo-human
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt
```

Dependencias instaladas desde [requirements.txt](/Users/gianig/dafo-human/requirements.txt):

- `mujoco==3.2.7`
- `glfw==2.10.0`
- `numpy==2.5.1`
- `PyOpenGL==3.1.10`

## Modelos MuJoCo

El launcher espera encontrar escenas Unitree en `third_party/mujoco_menagerie`.

Rutas esperadas:

- `third_party/mujoco_menagerie/unitree_h1/scene.xml`
- `third_party/mujoco_menagerie/unitree_g1/scene.xml`
- `third_party/mujoco_menagerie/unitree_g1/scene_with_hands.xml`

Si esa carpeta no existe, el simulador falla al resolver el modelo.

## Validación rápida

Comprobar que MuJoCo carga el modelo sin abrir el viewer:

```bash
cd /Users/gianig/dafo-human
.venv/bin/python simulate_unitree.py --robot g1-hands --mode headless --steps 300
```

Si eso funciona, la instalación base está lista.

## Viewer interactivo

Para abrir el viewer usa `mjpython`, no `python`:

```bash
cd /Users/gianig/dafo-human
.venv/bin/mjpython simulate_unitree.py --robot g1-hands --mode viewer
```

Eso evita fallos observados al abrir el viewer de MuJoCo con el intérprete estándar.

## Problemas comunes de instalación

### La escena no existe

Síntoma:

```text
No encontre el modelo en ...
```

Causa:

- Falta `third_party/mujoco_menagerie`.
- O la escena custom pasada con `--xml` no existe.

### El viewer no abre bien

Usa:

```bash
.venv/bin/mjpython simulate_unitree.py --robot g1-hands --mode viewer
```

No uses:

```bash
.venv/bin/python simulate_unitree.py --robot g1-hands --mode viewer
```

### Puerto UDP ocupado

El viewer escucha por defecto en `127.0.0.1:47001`.

Ver qué proceso lo está usando:

```bash
lsof -nP -iUDP:47001
```

## Siguiente paso

Después de instalar, sigue [RUNBOOK.md](/Users/gianig/dafo-human/RUNBOOK.md) para operar el simulador y [WALKING.md](/Users/gianig/dafo-human/WALKING.md) para el estado actual de la caminata.