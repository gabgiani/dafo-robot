# Walking Guide

Esta guía resume el estado actual del controlador de caminata y cómo conviene probarlo.

## Qué controla hoy la marcha

El controlador está implementado en [interactive_unitree.py](/Users/gianig/dafo-human/interactive_unitree.py).

Variables principales de la marcha:

- `advance`: intención de avance en `[-1, 1]`
- `turn`: intención de giro en `[-1, 1]`
- `amplitude_scale`: amplitud de zancada
- `frequency_hz`: frecuencia del ciclo

La marcha actual combina:

- Movimiento de cadera, rodilla y tobillo.
- Balanceo lateral para soporte.
- Asistencia física aplicada al pelvis con `xfrc_applied`.

No usa traslación directa de `qpos` para mover la base.

## Estado actual

Objetivo de esta iteración:

- Quitar el patinaje por arrastre artificial de la base.
- Hacer que el avance salga del patrón de piernas más una asistencia física consistente.

Estado validado en headless:

- `advance=0.8` da avance neto y se mantiene de pie.
- `advance=1.0` da avance neto y se mantiene de pie en la prueba corta.

Estado observado en viewer:

- Mantener `+1.0` durante mucho tiempo todavía puede terminar en caída y reset.
- El comportamiento en viewer sigue siendo menos estable que el headless.

## Cómo probar la marcha

Primero abrir el viewer:

```bash
cd /Users/gianig/dafo-human
.venv/bin/mjpython simulate_unitree.py --robot g1-hands --mode viewer
```

Luego abrir el teleop:

```bash
cd /Users/gianig/dafo-human
.venv/bin/python teleop_unitree.py --host 127.0.0.1 --port 47001
```

## Rango recomendado de prueba

Para no saturar el controlador desde el primer segundo:

1. Empezar en `advance=0.2`.
2. Subir a `0.4`.
3. Probar `0.6` y `0.8`.
4. Usar `1.0` solo para pruebas cortas.

Recomendación práctica actual:

- Para avance controlado: `0.4` a `0.8`.
- Para stress test: `1.0`.

## Qué significan los controles de marcha

- `W/S`: cambia `advance`.
- `A/D`: cambia `turn`.
- `J/K`: cambia `amplitude_scale`.
- `N/M`: cambia `frequency_hz`.

En la práctica:

- Más amplitud: zancada más grande, pero más riesgo de perder estabilidad.
- Más frecuencia: pasos más rápidos, pero más sensibles a caídas.

## Validación rápida sin viewer

Si cambias el controlador, la validación mínima recomendada es:

```bash
cd /Users/gianig/dafo-human
.venv/bin/python simulate_unitree.py --robot g1-hands --mode headless --steps 300
```

Y luego una prueba focalizada similar a las usadas durante esta sesión:

- medir `pelvis_delta`
- medir `pelvis_min_height`
- verificar que no reaparezca patinaje

## Limitaciones actuales

- La caminata todavía no es una locomoción robusta de cuerpo completo.
- El viewer y el headless no se comportan exactamente igual bajo comandos largos.
- El controlador actual está pensado para iterar rápido, no para una política dinámica completa.

## Siguiente mejora técnica lógica

Si se sigue trabajando la marcha, lo siguiente debería ser:

1. separar control de soporte y swing por pie
2. medir deslizamiento real del pie en contacto
3. limitar el avance máximo cuando el soporte cae a un solo pie
4. desacoplar mejor avance y giro