# Workshop: de control manual a caminar con Reinforcement Learning

*[English version](WORKSHOP.md)*

## Objetivo del workshop

Lo que buscamos lograr, en una frase: que un robot humanoide bípedo (Unitree G1) se
**mantenga parado** y, a partir de ahí, **camine hacia adelante sin caerse** — primero
con un controlador escrito a mano y después con una política de Reinforcement Learning
(RL) ya entrenada.

Al terminar este workshop, cada participante va a poder:

- Explicar por qué un robot bípedo de pie es inestable y qué hace falta para mantenerlo
  erguido.
- Ejecutar un controlador "a mano" (fórmulas fijas por motor) y ver en qué condiciones
  se cae.
- Ejecutar una política de RL ya entrenada y comparar, con números reales, cuánto mejor
  sostiene el equilibrio que el controlador a mano frente a la misma tarea.
- Agregar objetos al escenario y comprobar si el robot sigue de pie al chocar con algo
  que el entrenamiento original no anticipó.
- Repetir, paso a paso, las mismas pruebas que documentamos acá (con sus resultados
  reales y capturas) para verificar por sí mismos lo que reportamos.

## El problema que resolvemos

Un robot humanoide de pie es una torre alta y angosta apoyada sobre dos pies chicos:
cualquier pequeño error de ángulo en una pierna, o cualquier empujón, lo puede tirar.
Mantenerlo de pie — y encima caminando — exige corregir el equilibrio constantemente,
muchas veces por segundo. Hay dos formas de resolver esto que vamos a comparar en este
workshop:

1. **A mano**: un humano escribe fórmulas fijas (amplitud de zancada, flexión de
   rodilla, fuerzas de corrección) y las ajusta por prueba y error. Funciona hasta
   cierto punto, pero es frágil.
2. **Con una política aprendida (RL)**: una red neuronal aprendió sola, en simulación y
   por prueba y error, qué torque mandarle a cada motor para no caerse, sin que un
   humano escriba las fórmulas a mano.

El detalle de qué es Reinforcement Learning y cómo funciona la política ya entrenada
está en [REINFORCEMENT_LEARNING.es.md](REINFORCEMENT_LEARNING.es.md); en este workshop nos
enfocamos en la experiencia práctica de correr y comparar ambos enfoques.

## Requisito previo

Seguir [INSTALL.es.md](INSTALL.es.md) para tener el entorno (`.venv`) y
`third_party/mujoco_menagerie` instalados.

## Las 4 etapas

Cada etapa tiene su propia página con: el objetivo puntual de esa etapa, cómo
ejecutarla, en qué consiste, qué mirar, cómo se resuelve el problema, las capturas
reales de las pruebas que hicimos, y qué problemas encontramos armándola — para que
puedas repetir cada prueba paso a paso y comprobar si te da el mismo resultado.

1. **[Control heurístico](workshop/01-control-heuristico.es.md)** — objetivo: hacer
   caminar al robot con fórmulas a mano por motor + control remoto, y encontrar en qué
   punto se cae.
2. **[Reinforcement Learning](workshop/02-reinforcement-learning.es.md)** — objetivo:
   la misma tarea (caminar sin caerse), reemplazando las fórmulas a mano por una
   política ya entrenada.
3. **[Objetos en el escenario](workshop/03-objetos-en-el-escenario.es.md)** — objetivo:
   comprobar si la política de RL sigue de pie cuando además tiene que lidiar con
   cajas y un estante en el camino.
4. **[Control de cuerpo completo con SONIC](workshop/04-control-cuerpo-completo-sonic.es.md)** —
  objetivo: caminar físicamente con una policy de 29 DOF y coordinar postura de carga
  o balanceo natural de brazos desde un telecomando gráfico seguro.

## Resumen rápido: qué ejecutar en cada etapa

Si solo querés los comandos para probar ya mismo, son estos (el detalle de cada uno está
en la página de su etapa).

**Etapa 1 — heurístico:**
```bash
./run_viewer.sh
```
```bash
.venv/bin/python send_unitree_command.py --advance 0.5
```

**Etapa 2 — RL:**
```bash
./run_viewer_rl.sh
```
```bash
.venv/bin/python send_unitree_command.py --advance 0.6
```
(o directamente `W/A/S/D` en la ventana del visor, sin necesitar la segunda terminal)

**Etapa 3 — RL + objetos:**
```bash
.venv/bin/mjpython simulate_g1_rl.py --scene third_party/unitree_rl_gym/resources/robots/g1_description/g1_warehouse_scene.xml
```
```bash
.venv/bin/python send_unitree_command.py --advance 0.6
```

**Etapa 4 — SONIC de cuerpo completo:**
```bash
# Ejecutar ambos comandos en la misma sesión del desktop Linux remoto:
export SONIC_ROOT="$PWD"
./dafo-human-sonic/sonic_desktop_control.sh sim
./dafo-human-sonic/sonic_desktop_control.sh control
```

Esperar `Policy: Listo`, pulsar **Activar** y no mover hasta que el teleop indique
`Control conectado`. La Etapa 4 tiene prerrequisitos adicionales y un patch upstream;
seguir su página completa antes de ejecutar estos comandos.

## Para seguir profundizando

- [WALKING.es.md](WALKING.es.md): detalle técnico del controlador heurístico de la etapa 1.
- [REINFORCEMENT_LEARNING.es.md](REINFORCEMENT_LEARNING.es.md): cómo funciona por dentro la política de la etapa 2 (qué es RL, cuántos motores, cuántos valores de observación, qué significan los paneles del visor).
- [RUNBOOK.es.md](RUNBOOK.es.md): operación diaria del simulador en general.
- [FULL_BODY_INTEGRATION.es.md](FULL_BODY_INTEGRATION.es.md): por qué el intento anterior
  de mezclar una policy de piernas con control PD de brazos no fue físicamente estable.
