# DAFO-ROBOT Research

*[English version](README.md)*

Este repositorio documenta el tramo inicial del research para enseñarle a un robot
humanoide a hacer trabajo físico útil. El hito de mediano plazo es deliberadamente
simple y concreto: que el robot pueda tomar un objeto de un lugar y moverlo a otro sin
caerse, sin perder el objeto y sin depender de que un humano escriba a mano cada
movimiento articular.

Ese objetivo de pick-and-place no es la meta final. Es la primera tarea medible que
obliga a resolver los problemas reales del trabajo físico: equilibrio, locomoción,
contacto, coordinación del tren superior, percepción del escenario y selección de
acciones en el tiempo. Si esa base se vuelve confiable, el mismo enfoque puede empujar
después otras tareas, incluyendo flujos de manipulación manual y tareas derivadas de
manuales de ensamblado.

## Objetivo del research

El proyecto intenta responder una pregunta práctica: cómo convertimos conocimiento
operativo que hoy existe en operadores humanos y procedimientos en comportamiento
robótico que pueda entrenarse, medirse, reproducirse y mejorarse.

La hipótesis de trabajo es que el camino correcto no es arrancar con una promesa
end-to-end gigante, sino descomponer el problema en capacidades progresivas:

1. mantener al robot estable
2. hacer que se mueva con intención
3. lograr que sobreviva al contacto con el mundo
4. coordinar el cuerpo completo para acciones útiles
5. conectar esas capacidades con ejecución de tareas simples

Este repositorio es la base de implementación y el cuaderno de research de ese camino.

## Qué tarea estamos persiguiendo

La tarea objetivo concreta es intencionalmente modesta al principio:

- detectar o recibir la ubicación de un objeto
- acercarse sin caerse
- alcanzarlo con el tren superior
- agarrarlo o transportarlo
- moverlo a una segunda ubicación
- repetir el proceso bajo variaciones controladas

Si el proyecto no puede hacer eso de forma confiable en simulación, no hay base técnica
para afirmar que puede manejar trabajo más rico como manipulación repetitiva o manuales
de ensamblado. La tarea simple, entonces, no es un juguete: es el banco de prueba.

## Por qué dividir el research en etapas

La manipulación humanoide falla por muchas razones distintas, y mezclarlas todas en un
solo experimento oculta la causa real de los errores. Un programa por etapas vuelve
observable el modo de falla.

- Un problema de estabilidad todavía no es un problema de grasping.
- Un problema de caminata todavía no es un problema de percepción.
- Un problema de coordinación de brazos todavía no es un problema de planificación.
- Un problema de control de cuerpo completo debería medirse antes de afirmar competencia
	sobre una tarea.

Por eso el repositorio está organizado como workshops y documentos focalizados en vez de
una sola demo monolítica. Cada etapa aísla una capacidad, una comparación o un problema
de integración y deja registro de qué funcionó, qué falló y por qué.

## Por qué no apoyarnos sólo en control determinístico

Los métodos determinísticos siguen siendo parte de la caja de herramientas. Los usamos
para baselines, instrumentación, envolventes de seguridad, telecomando, pruebas
repetibles y comportamientos simples guionados. Pero no alcanzan por sí solos para el
objetivo que nos importa.

En un robot humanoide, el control totalmente diseñado a mano se vuelve frágil rápido
porque:

- el espacio de estados es de alta dimensión
- las correcciones de equilibrio deben ser continuas y rápidas
- el contacto con el mundo es difícil de modelar con reglas fijas
- las decisiones del tren superior e inferior se acoplan de formas costosas de ajustar a mano
- el mismo script se rompe cuando cambian la fricción, el timing o la geometría

Dicho de otra forma: un controlador determinístico puede mostrar un movimiento acotado,
pero eso no equivale a una capacidad robusta. Por eso este repositorio compara control
explícito escrito a mano contra políticas aprendidas y luego estudia dónde el control
aprendido de cuerpo completo se vuelve necesario.

## Stack tecnológico

El stack actual de research combina estas piezas:

- **MuJoCo** como simulador físico principal para iteración rápida y pruebas repetibles.
- **Modelos humanoides Unitree** como embodiment bajo estudio, especialmente G1.
- **Tooling en Python** para launchers, teleoperación, experimentos y análisis.
- **Políticas de Reinforcement Learning** para equilibrio y locomoción cuando las fórmulas fijas se vuelven demasiado frágiles.
- **Control aprendido de cuerpo completo con NVIDIA GEAR-SONIC** para movimiento coordinado de 29 DOF, incluyendo locomoción más postura del tren superior.
- **Interfaces de telecomando** para mandar órdenes y observar el robot de forma segura mientras se validan policies.
- **Workshops basados en escenarios** para documentar cada etapa del research como caso reproducible.

La idea no es acumular herramientas. La idea es usar el stack mínimo que permita
responder una pregunta difícil con evidencia.

La justificación del stack es directa:

- MuJoCo permite resets rápidos, variaciones controladas y experimentos de contacto reproducibles.
- Unitree G1 ofrece una morfología humanoide realista para estudiar equilibrio más manipulación.
- Python mantiene barato el loop de experimentación mientras las superficies de control todavía están cambiando.
- RL sirve donde el controlador debe absorber variación continua en lugar de repetir un script fijo.
- SONIC pasa a ser relevante cuando la competencia en piernas ya no alcanza y la tarea exige coordinación del tren superior.

## Hoja de ruta actual del research

Hoy el proyecto está estructurado en cuatro etapas principales:

1. **Control heurístico**: medir hasta dónde llega una caminata basada en reglas escritas a mano.
2. **Locomoción con Reinforcement Learning**: comparar una policy entrenada contra la base manual.
3. **Objetos en el escenario**: verificar si la locomoción sobrevive a contacto y clutter.
4. **Control de cuerpo completo con SONIC**: pasar de competencia en piernas a control coordinado de cuerpo completo con comportamientos del tren superior como postura de carga y balanceo de brazos.

Esas etapas no significan que la tarea ya esté resuelta. Significan que la base empieza
a ser lo bastante creíble como para intentar conductas simples de mover objetos y, más
adelante, flujos manuales más estructurados.

## Cómo leer este repositorio

El README es la vista general del proyecto. La instalación, la operación y la ejecución
paso a paso están separadas a propósito.

- [INSTALL.es.md](INSTALL.es.md): instalación local y validación base.
- [RUNBOOK.es.md](RUNBOOK.es.md): operación diaria del simulador, viewer, teleop y demo.
- [WORKSHOP.es.md](WORKSHOP.es.md): camino de research por etapas y cómo se prueba cada capacidad.
- [WALKING.es.md](WALKING.es.md): estado actual del controlador de caminata escrito a mano.
- [REINFORCEMENT_LEARNING.es.md](REINFORCEMENT_LEARNING.es.md): qué hace la policy de RL y por qué ayuda.
- [FULL_BODY_INTEGRATION.es.md](FULL_BODY_INTEGRATION.es.md): por qué el intento anterior de integración parcial de cuerpo completo fue inestable.

## Alcance del repositorio

Las etapas 1 a 3 son locales a este repositorio. La Etapa 4 agrega interacción por SSH
con una instalación externa de NVIDIA GR00T Whole-Body Control para los experimentos
SONIC. Este repositorio documenta y orquesta esos experimentos; no redistribuye
checkpoints de NVIDIA ni pretende que todo el stack sea autocontenido en local.

## Cómo se ve el éxito

La dirección de research sólo sirve si conduce a capacidad reproducible, no sólo a demos
interesantes. En términos prácticos, éxito significa poder mostrar, medir y mejorar un
robot que pueda:

- permanecer estable quieto y en movimiento
- aceptar comandos de forma segura
- coordinar locomoción y tren superior
- interactuar con objetos sin colapsar de inmediato
- ejecutar una tarea simple de transporte de punta a punta
- volverse entrenable para tareas adicionales más allá del primer escenario guionado

Ese es el motivo del foco actual: no porque caminar sea el producto final, sino porque
caminata, coordinación de cuerpo completo e interacción con objetos son la base mínima
para cualquier tarea posterior de tipo manual o de ensamblado descripta en un procedimiento operativo o en un manual de ensamblado.