# Runbook

Esta guía explica cómo ejecutar el simulador y cómo operarlo en el flujo normal de trabajo.

## Punto de entrada

El launcher principal es [simulate_unitree.py](/Users/gianig/dafo-human/simulate_unitree.py).

Funciones principales:

- Carga escenas de Unitree.
- Aplica el keyframe inicial.
- Corre en modo `headless` o `viewer`.
- Expone control UDP externo si `--control-port` no es `0`.

## Arranque rápido

### Headless

```bash
cd /Users/gianig/dafo-human
.venv/bin/python simulate_unitree.py --robot g1-hands --mode headless --steps 300
```

### Viewer con control UDP

```bash
cd /Users/gianig/dafo-human
.venv/bin/mjpython simulate_unitree.py --robot g1-hands --mode viewer
```

Por defecto escucha en `127.0.0.1:47001`.

### Viewer sin control UDP

```bash
cd /Users/gianig/dafo-human
.venv/bin/mjpython simulate_unitree.py --robot g1-hands --mode viewer --control-port 0
```

## Robots soportados

- `h1`
- `g1`
- `g1-hands`

Ejemplo:

```bash
cd /Users/gianig/dafo-human
.venv/bin/python simulate_unitree.py --robot h1 --mode headless --steps 300
```

## Keyframes

Por defecto el launcher usa el keyframe `home`.

Cambiar keyframe:

```bash
cd /Users/gianig/dafo-human
.venv/bin/mjpython simulate_unitree.py --robot g1-hands --mode viewer --keyframe home
```

Desactivar keyframe:

```bash
cd /Users/gianig/dafo-human
.venv/bin/python simulate_unitree.py --robot g1-hands --mode headless --keyframe ''
```

## Teleoperación

El flujo esperado es:

1. Abrir el viewer.
2. En otra terminal, lanzar el teleop.

```bash
cd /Users/gianig/dafo-human
.venv/bin/python teleop_unitree.py --host 127.0.0.1 --port 47001
```

Wrapper equivalente:

```bash
cd /Users/gianig/dafo-human
.venv/bin/python teleop_unitree.pyw --host 127.0.0.1 --port 47001
```

Controles:

- `W/S`: avance y retroceso
- `A/D`: giro
- `Espacio`: centrar avance y giro
- `R`: reset
- `P`: pausar
- `O`: reanudar
- `J/K`: bajar/subir amplitud
- `N/M`: bajar/subir frecuencia
- `Q`: salir

## Envío one-shot de comandos

Para pruebas puntuales, usa [send_unitree_command.py](/Users/gianig/dafo-human/send_unitree_command.py).

Ejemplos:

```bash
cd /Users/gianig/dafo-human
.venv/bin/python send_unitree_command.py --host 127.0.0.1 --port 47001 --advance 0.8
```

```bash
cd /Users/gianig/dafo-human
.venv/bin/python send_unitree_command.py --host 127.0.0.1 --port 47001 --turn 0.4
```

```bash
cd /Users/gianig/dafo-human
.venv/bin/python send_unitree_command.py --host 127.0.0.1 --port 47001 --center
```

```bash
cd /Users/gianig/dafo-human
.venv/bin/python send_unitree_command.py --host 127.0.0.1 --port 47001 --reset
```

## Demo de grasp

La demo disponible es [reach_grasp_demo.py](/Users/gianig/dafo-human/reach_grasp_demo.py).

Sin video:

```bash
cd /Users/gianig/dafo-human
.venv/bin/python reach_grasp_demo.py --no-video
```

Con video:

```bash
cd /Users/gianig/dafo-human
.venv/bin/python reach_grasp_demo.py
```

Salida por defecto:

- `artifacts/g1_reach_grasp.mp4`

## Operación diaria

Secuencia recomendada:

1. Validar modelo en `headless` si cambiaste física o controlador.
2. Abrir el `viewer` con `mjpython`.
3. Conectar `teleop_unitree.py` desde otra terminal.
4. Probar primero `advance=0.2` o `0.4` y subir de forma gradual.
5. Si el robot cae, usar `R` o reenviar `--reset`.

## Fallas habituales

### Puerto ocupado

```bash
lsof -nP -iUDP:47001
```

### Viewer correcto, pero sin mover el robot

Revisar:

- Que el viewer esté escuchando en `47001`.
- Que el teleop apunte al mismo host y puerto.
- Que no haya otra instancia anterior consumiendo el puerto.

### El viewer se reinicia por caída

Eso hoy forma parte de la lógica de recuperación del controlador en [interactive_unitree.py](/Users/gianig/dafo-human/interactive_unitree.py).

## Archivos operativos principales

- [simulate_unitree.py](/Users/gianig/dafo-human/simulate_unitree.py)
- [interactive_unitree.py](/Users/gianig/dafo-human/interactive_unitree.py)
- [external_control.py](/Users/gianig/dafo-human/external_control.py)
- [teleop_unitree.py](/Users/gianig/dafo-human/teleop_unitree.py)
- [send_unitree_command.py](/Users/gianig/dafo-human/send_unitree_command.py)