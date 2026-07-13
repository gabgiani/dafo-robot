# Modelo completo (brazos + manos): qué pasó, por qué, y cómo resolverlo de verdad

*[English version](FULL_BODY_INTEGRATION.md)*

Este documento deja registrado un intento real de integración: combinar la política RL de
caminata (solo piernas) con el modelo completo del G1 (brazos + manos), qué se rompió, por qué se
rompió, y el camino concreto para arreglarlo de verdad (reentrenar con la nueva distribución de
masa).

## La idea

[simulate_g1_rl.py](simulate_g1_rl.py) corre una política RL pre-entrenada que solo controla 12
articulaciones de las piernas ([g1_12dof.xml](third_party/unitree_rl_gym/resources/robots/g1_description/g1_12dof.xml)).
Ese modelo no tiene articulaciones de brazos/manos — son mallas estáticas fusionadas. Para agarrar
objetos necesitamos un modelo con brazos/manos articulados de verdad.

La buena noticia: `unitree_rl_gym` también trae
[g1_29dof_with_hand.xml](third_party/unitree_rl_gym/resources/robots/g1_description/g1_29dof_with_hand.xml),
de la **misma fuente**, con:
- **Los mismos nombres de articulación de las piernas** (`left_hip_pitch_joint`, `left_knee_joint`,
  etc.) en el mismo orden.
- **El mismo tipo de actuador** para las piernas (`<motor>`, control por torque) — coincide
  exactamente con lo que espera la política RL + nuestro control PD.

Entonces, en principio, podríamos alimentar los 12 torques de piernas de la política RL a los
primeros 12 actuadores de este modelo más grande, y controlar los 31 actuadores restantes
(cintura, brazos, manos) por separado con nuestro propio PD sosteniendo una pose neutra.

## Qué implementamos

1. Escena nueva: [g1_warehouse_scene_full.xml](third_party/unitree_rl_gym/resources/robots/g1_description/g1_warehouse_scene_full.xml),
   incluyendo `g1_29dof_with_hand.xml` (reusando las cajas/estante/cámara de la escena de almacén
   ya existente).
2. Extendí [simulate_g1_rl.py](simulate_g1_rl.py):
   - `self._num_upper = self.model.nu - 12` — detecta cuántos actuadores hay más allá de las
     piernas.
   - Un segundo controlador PD sostiene la parte superior (cintura/brazos/manos) en un objetivo
     neutro (todo en cero), con ganancias por grupo de articulaciones:

     | Grupo | kp | kd |
     |---|---|---|
     | Cintura (3 articulaciones) | 80 | 2 |
     | Hombro/codo/muñeca (14 articulaciones) | 30 | 1 |
     | Dedos (14 articulaciones) | 3 | 0.05 |

   - `_step_physics()` ahora escribe `ctrl[:12]` desde la política RL y `ctrl[12:]` desde este
     segundo PD.

Todo esto **compila y corre** — el cableado es correcto.

## Qué se rompió, y por qué no es un problema de cableado

Comparación de masa total:

```
modelo solo-piernas (con el que se entrenó la política): 32.11 kg
modelo completo (con brazos y manos reales):              36.17 kg   (+4 kg, +12.6%)
```

Incluso parado perfectamente quieto (`cmd = (0, 0, 0)`, sin ningún comando de caminar), el robot
**se cae en menos de un segundo**:

```
altura de la pelvis en el tiempo: [0.793, 0.769, 0.508, 0.091, 0.141, 0.062, 0.161, ...]
```
(0.793 m es la altura parado; cualquier valor debajo de ~0.4 m significa que está en el piso.)

**Causa raíz:** una política RL de locomoción aprende una estrategia de equilibrio que está
íntimamente ligada a la distribución exacta de masa e inercia del modelo con el que se entrenó. En
el modelo solo-piernas, los brazos son mallas estáticas fusionadas a la pelvis (su masa, si la
tienen, está simplificada dentro de la inercia del torso). El modelo completo tiene brazos/manos
articulados de verdad, cada uno con su propia masa, colgando de los hombros. Esto corre el centro
de masa y cambia el momento de inercia lo suficiente como para que la estrategia de equilibrio
aprendida ya no aplique — no es que camine peor, es que ni siquiera puede quedarse parado.

Esto **no se arregla ajustando las ganancias del PD de la parte superior** — la política de
piernas en sí no sabe reaccionar a un cuerpo para el que nunca fue entrenada.

## Qué haría falta para arreglarlo de verdad: reentrenar

### Framework: Isaac Lab, no Isaac Gym

La propia página de NVIDIA para Isaac Gym dice literalmente **"Isaac Gym - Now Deprecated"** y
recomienda migrar a **Isaac Lab**. `legged_gym` (el framework en el que se basa el código de
entrenamiento de `unitree_rl_gym`) anunció la misma migración. Así que el camino moderno correcto
es Isaac Lab, no la herramienta descontinuada.

Isaac Lab ya trae **entornos nativos de locomoción para el G1** que podríamos usar como referencia
o punto de partida:
- `Isaac-Velocity-Flat-G1-v0`
- `Isaac-Velocity-Rough-G1-v0`

### Qué hace falta

1. **El asset**: ya lo tenemos — `g1_29dof_with_hand.urdf` está en este repo
   (`third_party/unitree_rl_gym/resources/robots/g1_description/`). Isaac Lab puede importar URDF
   directo; la masa/inercia correcta sale automáticamente del archivo, no hace falta ajustarla a
   mano.
2. **Una configuración de entrenamiento** (términos de recompensa), adaptada del entorno de
   referencia del G1 o de la config propia de `unitree_rl_gym`, para el asset completo en vez del
   de solo piernas. Términos típicos: seguir la velocidad lineal/angular pedida, mantenerse
   erguido, minimizar energía/sacudones, penalizar caídas o fuerzas de contacto excesivas.
3. **Cómputo**: el entrenamiento corre miles de entornos simulados en paralelo en GPU durante
   muchas iteraciones — normalmente son **horas** de reloj, no minutos, incluso en hardware
   capaz.
4. **Exportar**: convertir el checkpoint entrenado a TorchScript (mismo formato `.pt` que
   `motion.pt`) para poder cargarlo con un script como `simulate_g1_rl.py` sin cambiar el código
   de inferencia.

### Chequeo de realidad de hardware (verificado, no asumido)

Requisitos mínimos documentados por Isaac Sim/Isaac Lab:

```
GPU VRAM: 16 GB o más
RAM:      32 GB o más
```

Lo que realmente tenemos disponible:

| Máquina | VRAM | RAM | Disco libre | ¿Cumple el mínimo? |
|---|---|---|---|---|
| RTX 3060 (`192.168.0.60`) | 6 GB | 23 GB | 30 GB (94% usado) | ❌ No — muy por debajo en todos los ejes |
| Tesla T4 (`10.8.0.3`) | 16 GB | 27 GB | 305 GB | ⚠️ Cumple la VRAM justo (sin margen), y ya comparte esa GPU con un servicio de TTS en vivo (`orpheus_ollama`) que ya hizo caer a otro modelo una vez por contención de recursos |

**Conclusión**: la RTX 3060 no puede correr Isaac Lab en absoluto — no es un escenario "más
lento", está por debajo del mínimo documentado en todos los ejes. La T4 es viable al límite, pero
comparte su VRAM (ya ajustada) con un servicio de producción.

## Alternativa práctica (sin necesidad de reentrenar)

En vez de forzar a un solo modelo a hacer las dos tareas a la vez, cambiar de modelo
**secuencialmente**:
1. Caminar hasta el objetivo con el modelo liviano de solo piernas (RL, ya probado y estable).
2. Al llegar lo suficientemente cerca, cambiar al modelo completo, controlado con el enfoque
   heurístico/por posición (al estilo `interactive_unitree.py`), solo para la fase estática de
   alcanzar y agarrar.

Esto evita por completo el problema de la masa, porque nunca le pedimos a la política RL que
sostenga el equilibrio cargando también la masa de los brazos — solo corre sobre el modelo exacto
para el que fue entrenada.

## Resumen

| | Estado |
|---|---|
| Mismos nombres de articulación/tipo de actuador entre modelos | ✅ Confirmado, no es un problema |
| Cableado del PD de sostén de la parte superior | ✅ Implementado, compila y corre |
| Equilibrio parado con el modelo completo | ❌ Se cae en menos de 1 segundo — no coinciden masa/inercia |
| Arreglo correcto (reentrenar la política) | Requiere Isaac Lab + nueva config de recompensas + GPU con 16GB+ VRAM / 32GB+ RAM |
| Hardware disponible para eso | La RTX 3060 no cumple los mínimos; la T4 al límite pero compartida con un servicio en vivo |
| Alternativa funcional hoy | Cambio de modelo secuencial (caminar con RL, después cambiar de modelo para agarrar) |
