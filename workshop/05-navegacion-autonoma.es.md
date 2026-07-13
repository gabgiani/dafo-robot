# Etapa 5 — Navegación autónoma RGB-D con evasión de obstáculos

*[English version](05-autonomous-navigation.md)*

## Objetivo de esta etapa

Hacer que el Unitree G1 viaje autónomamente desde un punto A hasta un punto B sin chocar
con los obstáculos intermedios. El robot debe percibir el escenario con la cámara de su
cabeza, construir un mapa transitable, planificar una ruta y convertirla continuamente
en comandos SONIC de cuerpo completo, conservando la seguridad de la etapa 4.

Esta etapa se ejecuta solamente en la máquina Linux remota. macOS sigue siendo la
estación de desarrollo y control; el render de MuJoCo y la política SONIC permanecen en
el host Linux con GPU.

## Cómo se ejecuta

Desde `/home/nvidia/GR00T-WholeBodyControl` en la sesión de escritorio Linux remota:

```bash
export DISPLAY=:0
export XAUTHORITY=/run/user/$(id -u)/gdm/Xauthority
dafo-human-sonic/sonic_navigation_stack.sh \
  --goal-x 4.5 --goal-y 0 \
  --speed 0.1 --goal-tolerance 0.35 --timeout 90
```

El launcher instala el escenario, inicia el simulador, el relay de pose física, la
política SONIC y el controlador autónomo, y finalmente los detiene en conjunto. Los logs
completos quedan en `/tmp/sonic_navigation_*.log`.

## En qué consiste

El escenario [sonic_navigation_scene.xml](../sonic_navigation_scene.xml) ubica dos
obstáculos fijos entre el inicio `(0, 0)` y el objetivo `(4.5, 0)`. El stack completo se
divide en cuatro componentes pequeños:

- [sonic_navigation_sim.py](../sonic_navigation_sim.py) conserva el stream RGB JPEG en
  el puerto `5555` y publica depth métrico más calibración de cámara en el `5559`.
- [sonic_navigation_planner.py](../sonic_navigation_planner.py) proyecta depth a
  coordenadas globales, filtra por altura, rasteriza e infla obstáculos, ejecuta A* y
  selecciona un waypoint con lookahead.
- [sonic_autonomous_navigation.py](../sonic_autonomous_navigation.py) consume depth,
  pose física en `5558` y telemetría SONIC en `5557`, y publica comandos globales
  `movement` y `facing` en `5556`.
- [sonic_navigation_stack.sh](../sonic_navigation_stack.sh) controla el ciclo de vida
  Linux-only y el cierre coordinado.

En `WALK=2`, tanto `movement` como `facing` están expresados en coordenadas globales. El
vector de movimiento lleva la velocidad solicitada y acotada, mientras el campo `speed`
del planner conserva el valor centinela SONIC `-1.0`.

## Pipeline de percepción y planificación

1. Decodificar la imagen depth comprimida de punto flotante `(480, 640)` junto con sus
   intrínsecos y extrínsecos de cámara.
2. Retroproyectar píxeles muestreados a una nube 3D y transformarla al espacio global.
3. Conservar puntos a altura de obstáculo, quitar retornos dentro del footprint de
   `0.35 m` del G1 y rasterizar el resto con resolución de `0.1 m`.
4. Inflar las celdas ocupadas `0.45 m` para reservar espacio al cuerpo del robot.
5. Ejecutar A* con ocho conexiones desde la pose física actual hasta el objetivo.
6. Mantener un waypoint seguro con lookahead de `0.6 m` hasta alcanzarlo o hasta que
   quede bloqueado. Así las rutas simétricas no cambian de lado en cada frame de depth.
7. Girar progresivamente hacia ese waypoint y luego caminar con vectores globales de
   movimiento y orientación.

## Comportamiento de seguridad

El movimiento no comienza sólo porque exista un puerto. El controlador espera un frame
depth real, pose física y telemetría SONIC posterior a la activación. Durante la
navegación publica un `IDLE` estable y se detiene si alguna entrada queda stale, la pelvis
indica una caída, el clearance se vuelve crítico, A* no encuentra ruta segura, falla un
subscriber o vence el timeout. Luego el launcher termina política, relay y simulador como
una sola unidad para que el robot nunca quede sin su controlador.

## Resultado remoto real

La ruta completa se validó en el host Linux con `--speed 0.1`:

```text
inicio:                    (0.00, 0.00)
objetivo:                  (4.50, 0.00)
desvío lateral máximo y:   aproximadamente 2.17 m
pose final informada:      aproximadamente (3.83, 0.47)
distancia final al goal:   0.326 m
avisos de caída:           0
política después de salir: inactiva
simulador después de salir: inactivo
```

El frame RGB-D inicial produjo 7.352 puntos de obstáculo. La primera ruta tuvo 48 celdas
y rodeó el obstáculo A en lugar de atravesarlo. El robot físico pasó ambos obstáculos,
volvió hacia el eje del objetivo, alcanzó la tolerancia configurada de `0.35 m` y cerró
limpiamente.

## Qué problemas encontramos

- **A/D cambiaba la dirección solicitada pero el robot no giraba físicamente.** SONIC
  `WALK=2` espera una orientación global progresiva, no un atajo de velocidad angular
  local. Un probe acotado midió un giro solicitado de `+90°` como `+88.08°`, sin caída.
- **El goal corto de smoke se quedaba intermitentemente sin ruta.** El objetivo `x=0.8`
  estaba en el borde de la región inflada del primer obstáculo. Moverlo a `x=0.5` separó
  la validación de locomoción de la evasión: terminó a `0.135 m`, con cero caídas.
- **A* alternaba entre lados igualmente válidos de un obstáculo simétrico.** Mantener el
  waypoint actual hasta alcanzarlo o bloquearlo eliminó la oscilación de dirección.
- **El cuerpo móvil del robot aparecía en su propia imagen depth.** Esos retornos cercanos
  atrapaban el inicio de A* y disparaban falsos stops por clearance crítico. Un self-filter
  acotado elimina puntos dentro de `0.35 m`; la inflación mayor de `0.45 m` todavía deja
  la geometría externa fuera del footprint del robot.
- **Un teardown independiente podía hacer caer al robot quieto.** El launcher controla
  todos los PID, publica `IDLE` estable y apaga política y simulador en conjunto.
