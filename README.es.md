# dafo-human

*[English version](README.md)*

Base mínima para instalar, ejecutar y entender cómo se usa este simulador de MuJoCo con robots Unitree.

## Mapa de documentación

- [INSTALL.es.md](INSTALL.es.md): instalación local y validación base.
- [RUNBOOK.es.md](RUNBOOK.es.md): operación diaria del simulador, viewer, teleop y demo.
- [WALKING.es.md](WALKING.es.md): estado actual del controlador de caminata y cómo probarlo.
- [WORKSHOP.es.md](WORKSHOP.es.md): recorrido paso a paso — control a mano vs. Reinforcement Learning vs. objetos en el escenario.
- [REINFORCEMENT_LEARNING.es.md](REINFORCEMENT_LEARNING.es.md): cómo funciona por dentro la política de RL que mantiene al robot parado.
- [FULL_BODY_INTEGRATION.es.md](FULL_BODY_INTEGRATION.es.md): qué pasó al combinar las piernas RL con el modelo completo de brazos/manos, y cómo arreglarlo de verdad (reentrenar).

## Qué hay en este repositorio

Este proyecto está orientado a ejecución local. Hoy no incluye un pipeline de despliegue remoto, contenedores ni scripts de infraestructura. El flujo real es:

1. Instalar dependencias Python.
2. Tener disponible `third_party/mujoco_menagerie`.
3. Lanzar el simulador en modo `viewer` o `headless`.
4. Controlarlo desde otra terminal por UDP con `teleop_unitree.py`.

## Requisitos

- macOS o Linux con Python 3.
- Un virtualenv en `.venv`.
- MuJoCo Python `3.2.7`.
- La carpeta `third_party/mujoco_menagerie` con los modelos Unitree.
- Opcional: `ffmpeg` para exportar video en la demo de grasp.

## Instalación

La guía detallada está en [INSTALL.es.md](INSTALL.es.md).

Crear el entorno e instalar dependencias:

```bash
cd /ruta/al/repo/dafo-human
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt
```

Si falta la menagerie de MuJoCo, clónala dentro de `third_party` para que existan rutas como estas:

- `third_party/mujoco_menagerie/unitree_h1/scene.xml`
- `third_party/mujoco_menagerie/unitree_g1/scene.xml`
- `third_party/mujoco_menagerie/unitree_g1/scene_with_hands.xml`

## Arranque rápido

Prueba headless para validar que MuJoCo compila el modelo:

```bash
cd /ruta/al/repo/dafo-human
.venv/bin/python simulate_unitree.py --robot g1-hands --mode headless --steps 300
```

Abrir el simulador con viewer:

```bash
cd /ruta/al/repo/dafo-human
.venv/bin/mjpython simulate_unitree.py --robot g1-hands --mode viewer
```

Modelos soportados por el launcher:

- `h1`
- `g1`
- `g1-hands`

## Cómo se “despliega” hoy

En este repo, despliegue significa ejecución local del simulador. No existe una etapa separada de build/deploy a servidor.

El punto de entrada es [simulate_unitree.py](simulate_unitree.py), que:

- Resuelve la escena del robot.
- Compila el modelo MuJoCo.
- Aplica el keyframe inicial.
- Abre viewer o corre una prueba headless.
- Puede escuchar control externo por UDP en `127.0.0.1:47001`.

## Control del robot

La operación completa está documentada en [RUNBOOK.es.md](RUNBOOK.es.md).

El viewer corre la física y escucha comandos externos por UDP. El teleop se ejecuta en otra terminal:

```bash
cd /ruta/al/repo/dafo-human
.venv/bin/python teleop_unitree.py --host 127.0.0.1 --port 47001
```

También existe el wrapper:

```bash
cd /ruta/al/repo/dafo-human
.venv/bin/python teleop_unitree.pyw --host 127.0.0.1 --port 47001
```

Controles del teleop:

- `W/S`: avance y retroceso
- `A/D`: giro
- `Espacio`: centrar joystick
- `R`: reset
- `P/O`: pausar y reanudar
- `J/K`: bajar/subir amplitud
- `N/M`: bajar/subir frecuencia
- `Q`: salir

## Demo disponible

Hay una demo de reach-and-grasp para G1 con manos en [reach_grasp_demo.py](reach_grasp_demo.py).

Ejecutar sin video:

```bash
cd /ruta/al/repo/dafo-human
.venv/bin/python reach_grasp_demo.py --no-video
```

Exportar video:

```bash
cd /ruta/al/repo/dafo-human
.venv/bin/python reach_grasp_demo.py
```

El video se escribe por defecto en `artifacts/g1_reach_grasp.mp4`.

## Archivos principales

- [simulate_unitree.py](simulate_unitree.py): launcher principal.
- [interactive_unitree.py](interactive_unitree.py): loop del viewer y controlador interactivo.
- [external_control.py](external_control.py): transporte UDP.
- [teleop_unitree.py](teleop_unitree.py): teclado en raw mode para mandar comandos.
- [send_unitree_command.py](send_unitree_command.py): envío one-shot de comandos UDP.
- [reach_grasp_demo.py](reach_grasp_demo.py): demo de alcance y agarre.

## Problemas comunes

Si el viewer no abre correctamente:

- Usa `.venv/bin/mjpython` en vez de `.venv/bin/python` para el modo `viewer`.
- Verifica que el puerto `47001` no esté ocupado por otra instancia.

Si aparece un error de puerto ocupado:

```bash
lsof -nP -iUDP:47001
```

Si la escena no existe:

- Revisa que `third_party/mujoco_menagerie` esté presente.
- O usa `--xml /ruta/a/scene.xml` para pasar una escena personalizada.

## Comandos útiles

Viewer con control UDP desactivado:

```bash
cd /ruta/al/repo/dafo-human
.venv/bin/mjpython simulate_unitree.py --robot g1-hands --mode viewer --control-port 0
```

Viewer con cierre automático para validar arranque:

```bash
cd /ruta/al/repo/dafo-human
.venv/bin/mjpython simulate_unitree.py --robot g1-hands --mode viewer --max-seconds 10
```

Prueba headless con otro robot:

```bash
cd /ruta/al/repo/dafo-human
.venv/bin/python simulate_unitree.py --robot h1 --mode headless --steps 300
```