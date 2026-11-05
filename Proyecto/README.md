# MTRB — Robot móvil con ROS 2 Jazzy, Nav2 e interfaz web

## Resumen

**MTRB** es un robot móvil: un iRobot Create 3 (firmware I.0.0.CycloneDDS, namespace `/r2`,
dominio 21) con un LiDAR 2D
[LDROBOT LD19 (youyeetoo FHL-LD19)](https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2), que corre
**ROS 2 Jazzy con Nav2 y slam_toolbox** en una Raspberry Pi 5 (nativo, sin Docker). Se desarrolla en
macOS y se despliega en la Pi.

Tiene dos modos, que se eligen con `mode:=` al lanzar:

| Modo | Para qué sirve | Localización |
| --- | --- | --- |
| `mode:=mapping` | Crear un mapa nuevo, o ampliar uno guardado | slam_toolbox |
| `mode:=map` | Navegar de forma autónoma sobre un mapa ya guardado | AMCL + map_server |

Con `web:=true` el robot sirve además una **interfaz web** (sustituye a RViz) desde la propia Pi.
Se abre en cualquier navegador de la misma red, sin instalar nada, y permite:

- ver en tiempo real el **mapa, el robot, el LiDAR y la ruta**;
- enviar **objetivos de navegación** a Nav2, cancelarlos y **volver al dock**;
- **teleoperar** con joystick o teclado (con límites de velocidad y un watchdog de 300 ms);
- **parar** el robot con un botón siempre visible;
- **guardar y listar mapas**, y ver el estado de los nodos y de la batería.

> **Estado.** El código está implementado, revisado y probado **en macOS** (más de 1500 tests, análisis
> de seguridad y revisión final). **Todavía no se ha probado en el robot real.** Antes de moverlo con la
> web, sigue la primera prueba física de la sección 2.1.7, con las ruedas en el aire y la parada física
> del Create 3 al alcance. La web **no** es el freno de emergencia.

### Inicio rápido (ya instalado)

La primera vez hay que instalar y configurar todo: apartados **1.1–1.8** (ROS, Create 3, LiDAR),
**2.1.1** (dependencias Python de la web) y **2.1.2** (token). Después, en cada sesión:

```bash
cd ~/Robot-Lidar/Proyecto && source env.sh          # entorno (dominio 21, CycloneDDS)
export MTRB_WEB_TOKEN='<token de 20 o más caracteres>'   # o el archivo ~/.config/mtrb/web_token

# Mapear (el robot arranca en el dock) con la interfaz web:
ros2 launch mtrb_bringup mtrb.launch.py mode:=mapping web:=true

# Navegar sobre un mapa guardado (ruta absoluta):
ros2 launch mtrb_bringup mtrb.launch.py mode:=map map:=/home/<usuario>/mtrb_maps/<nombre>.yaml web:=true
```

Después abre `http://<ip-de-la-pi>:8080` e introduce el token. Sin `web:=true` el robot funciona igual,
solo que sin la interfaz.

### Seguridad en cinco líneas

1. **Solo red local y sin cifrado** (HTTP): usa una red dedicada y no expongas el puerto a internet.
2. **El token es obligatorio** (mínimo 20 caracteres): el nodo se niega a arrancar sin él. Nunca va en
   un YAML ni en git, ni es un parámetro de ROS.
3. **La web no sustituye a la parada física** del Create 3.
4. Tras 10 intentos fallidos de token, esa IP queda bloqueada 60 s.
5. El dominio ROS 2 (21) no tiene autenticación: quien esté en esa red puede saltarse la web.

### Qué leer según lo que quieras hacer

| Quiero… | Ver |
| --- | --- |
| Instalar todo desde cero | apartado 1 |
| Lanzar el robot y elegir el modo | apartado 2 |
| Usar la interfaz web, el token y los mapas | apartado 2.1 |
| Probar la web por primera vez en el robot | apartado 2.1.7 |
| Entender topics, namespaces y remapeos | apartado 3 |
| Ejecutar las pruebas o contribuir | apartados 5 y 6 |
| Saber qué falta por hacer | apartado 7 |
| Diseño, decisiones y especificación | No se publican en este repositorio (solo se incluye la carpeta `Proyecto/`) |

## Estructura del proyecto

```
src/
├── mtrb_description/            # modelo del robot (solo cinemática y visual, sin simulación)
│   ├── urdf/mtrb.urdf.xacro     # entrada: LiDAR (args lidar_*) + Create 3 opcional
│   ├── urdf/create3_body…, bumper…, wheel…, caster…, common_properties…
│   └── test/test_urdf.py
└── mtrb_bringup/
    ├── launch/mtrb.launch.py    # único launch, mode:=map|mapping; una función por subsistema
    ├── param/{nav2,ekf,slam_toolbox,lidar}.yaml
    ├── param/{web,web.example}.yaml             # puente web (token de ejemplo; el real va aparte)
    ├── map/
    ├── mtrb_bringup/utils.py                    # helpers puros (sin ROS), compartidos y testeados
    ├── mtrb_bringup/initial_pose_publisher.py   # publica la pose del dock en /r2/initialpose
    ├── mtrb_bringup/mtrb_web_bridge.py          # nodo ROS 2 del puente web (goals, teleop, mapas)
    ├── mtrb_bringup/web_server.py               # FastAPI: REST + WebSocket (sin imports de ROS)
    ├── mtrb_bringup/web_utils.py                # lógica pura del puente: validación, teleop, rutas
    ├── web/                     # interfaz: index.html, app.js, style.css (sin build ni CDN)
    │   └── dev/fake_server.py   # servidor falso para desarrollar la UI sin el robot
    └── test/                    # unitarias, integración y lint
requirements-web.txt             # dependencias Python del puente web (fastapi, uvicorn)
env.sh
scripts/patch_ldlidar.sh          # parche del driver ldlidar para GCC 13 (tras vcs import)
mtrb.repos                       # código de terceros (driver ldlidar_stl_ros2) → vcs import
```

Arquitectura del launch: `_launch_setup` lee los argumentos (`Settings`) y decide qué arrancar;
cada subsistema (`_state_publisher`, `_lidar`, `_ekf`, `_localization`, `_slam`, `_navigation`) devuelve
sus acciones. Todos los nodos se crean con `_node()`, que aplica namespace, remapeo de TF y
`use_sim_time`. Para añadir un subsistema: escribe una función nueva y añádela en `_launch_setup`.

## 1. Instalación y primera configuración

Sigue los pasos en orden la primera vez. Después, en el día a día, solo necesitas el paso 1.6
(`source env.sh`) y el apartado 2 (lanzar).

### 1.1 Requisitos

| Equipo | Qué necesita |
| --- | --- |
| Raspberry Pi 5 | Ubuntu 24.04 (64 bits) + [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) (`ros-jazzy-ros-base` basta), `git` |
| iRobot Create 3 | Firmware **I.0.0.CycloneDDS** (o cualquier `I.x` con CycloneDDS) |
| LiDAR LD19 | Conectado a la Pi por USB con su adaptador de puerto serie (CP210x, USB ↔ UART) |
| Red | Pi y Create 3 en la **misma red Wi-Fi**, con multicast permitido |

### 1.2 Clonar el repositorio

El workspace de ROS 2 es la carpeta **`Proyecto/`** del repositorio (contiene `src/`, `env.sh`,
`mtrb.repos` y `requirements-web.txt`); la raíz del repositorio solo contiene esa carpeta.
Todos los comandos de este README se ejecutan desde `Proyecto/`:

```bash
sudo apt install -y git
git clone https://github.com/MarLon-MV/Robot-Lidar.git ~/Robot-Lidar
cd ~/Robot-Lidar/Proyecto

# Driver del LD19 (ldlidar_stl_ros2), que no está en apt: se descarga dentro de src/
sudo apt install -y python3-vcstool
vcs import src < mtrb.repos

# Parche del driver para GCC 13 (Jazzy) y exclusión de multirobot_map_merge (no se usa)
scripts/patch_ldlidar.sh
```

[mtrb.repos](mtrb.repos) lista el código de terceros que se compila en el workspace. La carpeta
`src/ldlidar_stl_ros2/` está en `.gitignore`: no se sube a este repositorio.

Para actualizar más adelante: `cd ~/Robot-Lidar/Proyecto && git pull && vcs pull src && scripts/patch_ldlidar.sh` y vuelve a compilar
(paso 1.5).

### 1.3 Instalar las dependencias

```bash
source /opt/ros/jazzy/setup.bash

# Solo la primera vez en esta máquina (si ya lo hiciste, 'rosdep init' da un aviso: ignóralo)
sudo apt install -y python3-rosdep python3-colcon-common-extensions
sudo rosdep init
rosdep update

# Dependencias del proyecto (Nav2, slam_toolbox, robot_localization, xacro, CycloneDDS…)
cd ~/Robot-Lidar/Proyecto
rosdep install --from-paths src --ignore-src -r -y
```

`rosdep` lee los `package.xml` e instala todo lo que falta. Si prefieres hacerlo a mano:

```bash
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox \
  ros-jazzy-robot-localization ros-jazzy-rmw-cyclonedds-cpp ros-jazzy-xacro
```

### 1.4 Configurar el Create 3

El robot debe usar el **mismo namespace, dominio y middleware** que la Pi; si no, no se ven.

1. Conecta el robot a tu Wi-Fi (solo la primera vez): mantén pulsados los botones **1 y 2**
   hasta que el anillo de luz se ponga azul, conéctate a la red `Create-XXXX` y abre
   `http://192.168.10.1` → *Connect* → elige tu Wi-Fi.
2. Abre la web del robot en su IP de tu red (`http://<ip-del-robot>`) → *Application* →
   *Configuration* y pon:

   | Ajuste | Valor |
   | --- | --- |
   | ROS 2 Domain ID | `21` |
   | ROS 2 Namespace | `/r2` |
   | RMW Implementation | `rmw_cyclonedds_cpp` |

3. Guarda y reinicia la aplicación (*Application* → *Restart Application*).

Si cambias el namespace o el dominio, cámbialos también aquí: `namespace:=` en el launch y
`ROS_DOMAIN_ID` en [env.sh](env.sh).

### 1.5 Compilar

```bash
cd ~/Robot-Lidar/Proyecto
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

`--symlink-install` hace que los cambios en YAML, launch y Python se apliquen sin recompilar.
Recompila si añades archivos nuevos o tocas `setup.py`, `package.xml` o `CMakeLists.txt`.

### 1.6 Cargar el entorno (en cada terminal)

```bash
cd ~/Robot-Lidar/Proyecto
source env.sh
```

[env.sh](env.sh) fija `ROS_DOMAIN_ID=21` y `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` y carga ROS 2
y el workspace. Sin esto la Pi no ve al robot. Para no tener que escribirlo cada vez:

```bash
echo 'source ~/Robot-Lidar/Proyecto/env.sh' >> ~/.bashrc
```

### 1.7 Comprobar la conexión con el robot

```bash
ros2 topic list | grep /r2
```

Deben aparecer al menos `/r2/odom`, `/r2/imu`, `/r2/tf`, `/r2/tf_static` y `/r2/cmd_vel`.
Si no sale nada:

- Revisa que Pi y robot estén en la misma red y que `echo $ROS_DOMAIN_ID` dé `21`.
- Comprueba que el robot tenga la configuración del paso 1.4.
- Reinicia el daemon de ROS: `ros2 daemon stop && ros2 daemon start`.
- Si tu red bloquea multicast (redes de universidad o empresa), configura `CYCLONEDDS_URI`
  con la IP del robot como peer (ver apartado 4).

### 1.8 LiDAR (LD19)

El launch arranca el driver (`ldlidar_node`, de [ldlidar_stl_ros2](https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2))
y publica en `/r2/scan` con frame `laser`. Su configuración está en
[param/lidar.yaml](src/mtrb_bringup/param/lidar.yaml): 230400 baudios (obligatorio en el LD19),
10 Hz, producto `LDLiDAR_LD19`.

Datos del datasheet que usa el proyecto:

| Dato | Valor | Dónde se usa |
| --- | --- | --- |
| Rango | 0,02–12 m | `laser_*_range` (AMCL, slam_toolbox) |
| Frecuencia / muestras | 10 Hz, 4,5 kHz, ~0,8° | `lidar.yaml` |
| Puerto | UART 3,3 V, 230400 8n1, 5 V | `lidar.yaml`, `lidar_port:=` |
| Altura del plano de escaneo | ~24,5 mm sobre la base (aprox., medir) | `lidar_z` |
| Medidas | 54 × 46 × 35 mm, < 50 g | URDF ([lidar.urdf.xacro](src/mtrb_description/urdf/sensors/lidar.urdf.xacro)) |

**Primera conexión (una sola vez):**

```bash
# Permiso para usar el puerto serie (cierra sesión y vuelve a entrar después)
sudo usermod -aG dialout $USER

# Nombre fijo /dev/ldlidar (si no, puede ser ttyUSB0, ttyUSB1… según el orden de conexión).
# El adaptador es un CP210x (10c4:ea60); si tienes otro adaptador, cambia idVendor/idProduct.
echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0777", SYMLINK+="ldlidar"' \
  | sudo tee /etc/udev/rules.d/ldlidar.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
ls -l /dev/ldlidar      # debe apuntar a ttyUSBx
```

Si el puerto USB no da suficiente corriente, el motor no alcanza velocidad y el LiDAR se apaga
solo: usa un cable corto y, si hace falta, un hub con alimentación.

**Montaje:** pon el frente del LD19 (el lado opuesto al conector del cable) mirando al **frente** del robot, centrado si puedes.
Mide desde el centro del Create 3 (entre las dos ruedas, a nivel del suelo):

- `lidar_x`, `lidar_y`: posición del centro del LiDAR (x hacia delante, y hacia la izquierda).
- `lidar_z`: altura de la superficie donde lo atornillas **+ ~0,0245 m** (aprox.).
- `lidar_yaw`: déjalo en `0.0` (el valor por defecto) y comprueba la orientación con la prueba
  de abajo; si sale girado, ajústalo (`3.14159265` para 180°).

**Probar solo el LiDAR** (sin Nav2):

```bash
ros2 launch mtrb_bringup mtrb.launch.py mode:=mapping navigation:=false \
  lidar_port:=/dev/ldlidar lidar_x:=0.0 lidar_z:=0.13
ros2 topic hz /r2/scan          # ~10 Hz
```

Para comprobar la orientación, deja libre ~1 m alrededor, pon un objeto a ~0,3 m **delante**
del robot y mira dónde está la lectura más corta:

```bash
ros2 topic echo --once --full-length /r2/scan --field ranges | grep -oE '[0-9]+\.[0-9]+|inf|nan' \
  | LC_ALL=C awk '$1+0>0.05 {if(!m||$1+0<m){m=$1+0;i=NR}} END{print "min", m, "m en el índice", i, "de", NR}'
```

- Índice cerca de la **mitad** del array (ángulo 0 del frame `laser`): correcto, deja
  `lidar_yaw` por defecto.
- Índice cerca del **principio o del final** (ángulo ±π): el LiDAR está montado al revés, así
  que usa `lidar_yaw:=3.14159265`.

Si prefieres lanzar el driver por tu cuenta, pasa `lidar:=false`. Tu driver debe publicar en
`/r2/scan` con `frame_id` igual a `lidar_frame`.

### 1.9 Desarrollo en macOS (opcional)

En el Mac no hace falta ROS para editar y pasar las pruebas que no dependen de él:

```bash
git clone https://github.com/MarLon-MV/Robot-Lidar.git ~/Robot-Lidar
cd ~/Robot-Lidar/Proyecto          # el workspace es Proyecto/, no la raíz del repositorio
python3 -m venv .venv && source .venv/bin/activate
pip install pytest pyyaml flake8 flake8-import-order flake8-docstrings
pytest src/mtrb_bringup/test/test_utils.py src/mtrb_bringup/test/test_params.py
```

Para las pruebas del puente web (sin ROS) instala además `pip install -r requirements-web.txt`
(incluye `httpx`, que usan los tests). Las pruebas del nodo (`test_web_bridge.py`) y del launch
necesitan `rclpy`: en el Mac se saltan, y las demás pruebas (`colcon test`) se corren en la Pi.
Ejecútalas desde la carpeta del paquete (si no, falla el import y `flake8` ignora `setup.cfg`):

```bash
cd src/mtrb_bringup
python -m pytest -q test/ --ignore=test/test_flake8.py --ignore=test/test_pep257.py
python -m flake8 .
node --check web/app.js     # si tienes Node
```

## 2. Lanzar

Nunca AMCL y slam_toolbox a la vez; el launch lanza solo uno según `mode`.

```bash
# Localizar sobre un mapa existente (pose inicial = dock)
ros2 launch mtrb_bringup mtrb.launch.py mode:=map map:=<ruta/mapa.yaml> \
  dock_x:=0.0 dock_y:=0.0 dock_yaw:=0.0

# Mapa nuevo (el robot arranca en el dock; el origen del mapa es esa pose)
ros2 launch mtrb_bringup mtrb.launch.py mode:=mapping

# Ampliar un mapa guardado (base del pose-graph, sin extensión)
ros2 launch mtrb_bringup mtrb.launch.py mode:=mapping posegraph:=<ruta/base> \
  dock_x:=… dock_y:=… dock_yaw:=…
```

Argumentos útiles: `namespace` (r2), `navigation` (false = solo localizar/mapear), `use_ekf`,
`scan_topic`, `odom_topic`, `imu_topic`, `lidar` (false = sin driver), `lidar_port`, `lidar_frame`,
`lidar_x/y/z/yaw`, `explore` (apartado 2.0), `params_file`, `ekf_params_file`, `slam_params_file`, `lidar_params_file`, `use_sim_time`. Ver `ros2 launch mtrb_bringup mtrb.launch.py -s`.
Sin `map:=` se busca `map/map.yaml` del paquete instalado.

Guardar mapas (con `mode:=mapping` en marcha; el directorio debe existir):

```bash
ros2 service call /r2/slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '<ruta/base>'}}"
ros2 service call /r2/slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '<ruta/base>'}"
```

Guarda ambos: `.yaml/.pgm` para `mode:=map` y `.posegraph/.data` para ampliar. Si usas la
interfaz web (apartado 2.1), puedes guardar desde ahí; estos comandos quedan como alternativa.

Visualización y control: desde la interfaz web (apartado 2.1). No se usa RViz.

### 2.0 Exploración autónoma (explore_lite)

Con `explore:=true` (solo en `mode:=mapping`, con Nav2) el robot **se mueve solo**: busca la frontera
entre lo mapeado y lo desconocido, manda un objetivo a Nav2 y repite hasta cubrir el entorno. Al
terminar vuelve al punto de partida (el dock). Usa [explore_lite](https://github.com/robo-friends/m-explore-ros2),
que se baja con `vcs import src < mtrb.repos` (hay que compilar otra vez).

```bash
ros2 launch mtrb_bringup mtrb.launch.py mode:=mapping explore:=true web:=true
# guarda el mapa cuando acabe (botón de la web o los comandos de arriba)
```

- **Parar:** el botón **PARAR** de la web pausa la exploración (publica `false` en
  `/r2/explore/resume`) y cancela el objetivo. También: `ros2 topic pub --once /r2/explore/resume std_msgs/msg/Bool "{data: false}"`.
  Para reanudar, el mismo comando con `true`.
- **Estado:** `ros2 topic echo /r2/explore/status`. Las fronteras se ven en `/r2/explore/frontiers`.
- Ajustes (`param/explore.yaml`): `planner_frequency`, `min_frontier_size`, `progress_timeout`, `return_to_init`.
- **Seguridad:** va por Nav2, así que respeta sus límites de velocidad y evita obstáculos con el
  LiDAR, pero el LiDAR de 2D no ve escalones ni objetos bajos. Pruébalo con el robot vigilado,
  en una zona cerrada y sin escaleras, y con el botón PARAR a mano. Empieza explorando en cuanto
  arranca el launch.

### 2.1 Interfaz web (sustituye a RViz)

> **Estado:** implementada y probada en macOS (tests, análisis de seguridad y revisión de código),
> pero **todavía no probada en el robot**. Antes de mover el robot con ella, sigue el
> procedimiento de verificación física de la sección 2.1.7.

Un nodo (`mtrb_web_bridge`) sirve una página web desde la Pi y la conecta con ROS 2: muestra el
mapa, el robot, el LiDAR y la ruta en tiempo real, y permite enviar objetivos a Nav2, teleoperar,
parar el robot y guardar mapas. Funciona desde cualquier navegador de la misma red (móvil o
portátil), sin instalar nada.

#### 2.1.1 Dependencias Python

El puente usa FastAPI y uvicorn, que no vienen con ROS. La lista está en
[requirements-web.txt](requirements-web.txt). En Ubuntu 24.04 `pip` no deja instalar en el
Python del sistema, así que usa un entorno virtual que vea los paquetes de ROS:

```bash
python3 -m venv --system-site-packages ~/mtrb_venv
~/mtrb_venv/bin/pip install -r ~/Robot-Lidar/Proyecto/requirements-web.txt
```

`httpx` del mismo archivo solo hace falta para las pruebas. Para que `ros2 launch` encuentre
esos paquetes, añade el entorno virtual al `PYTHONPATH` antes de lanzar (comprueba la versión
de Python de la ruta con `ls ~/mtrb_venv/lib`):

```bash
export PYTHONPATH=~/mtrb_venv/lib/python3.12/site-packages:$PYTHONPATH
```

*Pendiente de verificar en la Pi:* esta combinación (entorno virtual + `ros2 launch`) no se ha
probado todavía.

#### 2.1.2 Token (obligatorio)

El acceso exige un token. **El nodo se niega a arrancar** si no hay token, si es el marcador de
ejemplo (`CHANGE-ME…`), si tiene **menos de 20 caracteres**, si está en blanco o si es un solo
carácter repetido. Genera uno (`token_urlsafe(24)` da 32 caracteres):

```bash
python3 -c "import secrets; print(secrets.token_urlsafe(24))"
```

El token **no es un parámetro de ROS** (cualquier equipo del dominio 21 lo leería con
`ros2 param get`) y no está en ningún YAML. Se lee de una de estas dos fuentes; gana la primera
que tenga valor:

1. La variable de entorno `MTRB_WEB_TOKEN`:

   ```bash
   export MTRB_WEB_TOKEN='<el-token-generado>'
   ```

2. Un archivo privado, por defecto `~/.config/mtrb/web_token` (cámbialo con el parámetro
   `auth.token_file`, ruta absoluta o que empiece por `~/`):

   ```bash
   mkdir -p ~/.config/mtrb
   install -m 600 /dev/null ~/.config/mtrb/web_token
   python3 -c "import secrets; print(secrets.token_urlsafe(24))" > ~/.config/mtrb/web_token
   ```

   El nodo exige que sea un archivo **normal** (no un enlace simbólico), **tuyo** y con permisos
   `600` (sin acceso para grupo ni otros); si no, no arranca. No supera 1 KB, se lee una vez y
   nunca se escribe en los logs.

`param/web.local.yaml` ya no sirve para el token: solo para sobrescribir otros parámetros, y hay
que recompilar para que el launch lo vea (lee la carpeta `param/` *instalada*).

#### 2.1.3 Lanzar y abrir

```bash
ros2 launch mtrb_bringup mtrb.launch.py mode:=mapping web:=true
# o, sobre un mapa existente:
ros2 launch mtrb_bringup mtrb.launch.py mode:=map map:=/home/<usuario>/mtrb_maps/<nombre>.yaml web:=true
```

Abre `http://<ip-de-la-pi>:8080` (cambia el puerto con `web_port:=`; rango 1024–65535) e introduce
el token. Sin `web:=true` el puente no arranca (es el valor por defecto).

#### 2.1.4 Qué puedes hacer

| Función | Cómo |
| --- | --- |
| Ver mapa, robot, LiDAR, ruta | Zoom con rueda o pellizco, arrastrar para desplazar, «Centrar en robot», capas conmutables |
| Enviar un objetivo (Nav2) | Toca el mapa, arrastra para elegir la orientación y pulsa «Confirmar objetivo». Nunca se envía sin confirmar |
| Cancelar / volver al dock | «Cancelar navegación» y «Volver al dock» (este pide confirmación) |
| Pose inicial | Solo con `mode:=map`: toca, orienta y confirma |
| Teleoperar | Activa «Activar teleoperación» (desactivada por defecto); joystick o teclas WASD / flechas. Límites: 0,2 m/s y 0,8 rad/s |
| **PARAR** | Barra roja fija abajo, sin confirmación. También la tecla **Escape**; **Espacio** solo si el foco no está en un control |
| Guardar mapa | Solo con `mode:=mapping`: escribe un nombre y pulsa «Guardar mapa» |
| Estado | Modo, batería, estado de los nodos de Nav2, edad de scan y odom, conexión |

Reglas que conviene conocer:

- La teleop se **rechaza** si hay un objetivo de Nav2 activo, y un solo navegador puede conducir a la vez.
- Si el navegador deja de enviar, a los 300 ms el robot recibe velocidad cero. Cerrar la pestaña o
  cortar la Wi-Fi también lo detiene.
- «Cancelar navegación» y PARAR cancelan **todos** los objetivos de `navigate_to_pose`, también los
  que hayas lanzado desde otra herramienta (RViz, línea de comandos).
- Tras un objetivo fallido, la web muestra «Fallido» y puede seguir bloqueando la teleop hasta que
  termine otro objetivo (limitación conocida, ver el apartado 7).

#### 2.1.5 Mapas guardados

Se guardan en `~/mtrb_maps` (cámbialo con el parámetro `map_dir` de `param/web.yaml`; solo se
admiten caracteres `A-Z a-z 0-9 _ . / -`). Cada guardado crea `<nombre>.yaml`, `<nombre>.pgm`,
`<nombre>.posegraph` y `<nombre>.data`.

- Nombres: letras, números, `_` y `-` (máx. 40), sin empezar por `-`. El nombre `map` está
  reservado (es el mapa por defecto de `mode:=map`).
- Si el nombre ya existe, la web pide confirmación antes de sobrescribir. Si responde
  `backup_pending`, hay un respaldo anterior sin resolver: revisa o borra `<nombre>.*.bak` en
  `~/mtrb_maps`.
- Límites: 200 mapas, 200 MB libres como mínimo y 5 s entre guardados.
- **Cargar** un mapa: `mode:=map map:=/home/<usuario>/mtrb_maps/<nombre>.yaml` (ruta absoluta).
- **Ampliar** un mapa (con el robot en el dock): `mode:=mapping posegraph:=/home/<usuario>/mtrb_maps/<nombre> dock_x:=… dock_y:=… dock_yaw:=…`
  (sin extensión). Si el guardado avisa de que no se guardó el posegraph, ese mapa no se podrá ampliar.
- Guardar antes de que `slam_toolbox` haya publicado un mapa falla (`500 save_failed`).

#### 2.1.6 Seguridad

- **Solo red local, sin cifrado (HTTP, sin TLS).** El token viaja en claro. Usa una red dedicada o
  aislada para el robot y **no expongas el puerto a internet**.
- **Intentos de token:** tras 10 fallos desde una misma IP en 60 s, esa IP queda bloqueada 60 s
  (error 429; el WebSocket no se abre). Una parada (`/api/stop`) con token válido siempre pasa.
  El bloqueo usa la IP del socket: detrás de un proxy inverso todos los clientes compartirían IP.
- **Conexiones:** el servidor limita las conexiones simultáneas (`limits.max_connections`), pero
  uvicorn no tiene un tiempo máximo para cabeceras enviadas con lentitud. Si la red no es de
  confianza, limita las conexiones por IP al puerto 8080 con un cortafuegos (por ejemplo,
  `nftables` con `ct count`) y sube `ulimit -n`.
- **Logs:** ROS escribe en `~/.ros/log` sin rotación; vigila el espacio de la tarjeta SD y limpia
  los logs antiguos de vez en cuando.
- **La web no es el freno de emergencia.** Ten siempre a mano la parada física del Create 3. Como
  respaldo independiente de la web:
  `ros2 service call /r2/navigate_to_pose/_action/cancel_goal action_msgs/srv/CancelGoal "{}"`.
- Al apagar con Ctrl-C no se garantiza que el puente alcance a publicar velocidad cero. Detén el
  robot antes de apagar el launch.
- El dominio ROS 2 (21) no tiene autenticación: cualquier equipo de esa red puede llamar a los
  servicios de ROS directamente, sin pasar por la web.

#### 2.1.7 Primera prueba en el robot

Hazla con el robot **con las ruedas en el aire**, en una zona despejada y con la parada física al
alcance. Orden recomendado:

1. Entorno: `timedatectl` con NTP sincronizado. Lanza con `navigation:=true` (el valor por
   defecto): la teleop exige el `velocity_smoother`, que es quien convierte `cmd_vel_nav` en
   `cmd_vel`; sin él nadie mueve el robot desde la web. Comprueba en **el suavizador** (es el nodo
   que importa para la parada):
   `ros2 param get /r2/velocity_smoother use_sim_time` → `False`,
   `ros2 param get /r2/velocity_smoother velocity_timeout` → `1.0` y
   `ros2 param get /r2/velocity_smoother enable_stamped_cmd_vel` → `False`.
   El puente (`/r2/mtrb_web_bridge`) y `/r2/bt_navigator` también deben dar `use_sim_time` `False`.
2. Grafo: `ros2 topic info -v /r2/cmd_vel_nav` (tipo `geometry_msgs/msg/Twist`) y
   `ros2 topic info -v /r2/cmd_vel` (su único publicador debe ser `velocity_smoother`).
3. Cancelación a mano: con el servicio de la sección 2.1.6, sin objetivos debe responder
   `return_code: 0`; con un objetivo lanzado por línea de comandos debe cancelarlo y `/r2/cmd_vel`
   debe caer a cero. Hazlo **antes** de fiarte del botón de la web.
4. Teleop: monitoriza `ros2 topic echo /r2/cmd_vel_nav` y `/r2/cmd_vel`. Al soltar el control debe
   llegar un cero y, a unos 300 ms, una ráfaga de ceros; después silencio.
5. PARAR, cerrar la pestaña y cortar la Wi-Fi mientras conduces: el robot debe detenerse. Mide
   cuánto tarda en parar (objetivo: menos de 500 ms).
6. Objetivo activo: la teleop debe rechazarse.
7. `kill -9` del puente con la teleop mantenida: el robot debe parar por el `velocity_timeout` del
   suavizador (1 s, con rampa de deceleración). Si no para, usa la parada física.
8. Solo después de todo lo anterior, prueba con las ruedas en el suelo.

#### 2.1.8 Parámetros

Todos en `param/web.yaml` (el token real, aparte; ver 2.1.2). Los valores de teleop están marcados
`INITIAL`: ajústalos en el robot.

| Parámetro | Por defecto | Notas |
| --- | --- | --- |
| `host`, `port` | `0.0.0.0`, `8080` | `web_port:=` en el launch tiene prioridad |
| `teleop.max_vx`, `teleop.max_wz` | `0.2`, `0.8` | Techos duros: 0,46 m/s y 1,9 rad/s |
| `teleop.timeout_ms` | `300` | Entre 50 y 1000 |
| `teleop.enabled` | `true` | `false` rechaza toda la teleop |
| `rates.pose/scan/status` | `10 / 5 / 1` Hz | Baja `scan` a 2 Hz si la Pi va justa |
| `limits.max_clients` | `5` | Navegadores simultáneos |
| `limits.max_connections` | `64` | Conexiones HTTP y WebSocket simultáneas (1–1024) |
| `map_dir` | `''` (`~/mtrb_maps`) | Ver 2.1.5 |
| `maps.min_free_mb`, `maps.min_interval_s` | `200`, `5.0` | `0` desactiva el intervalo |
| `auth.token_file` | `~/.config/mtrb/web_token` | Ruta del archivo del token (no el token); ver 2.1.2 |
| `auth.allow_example_token` | `false` | Solo desarrollo, y solo con host `127.0.0.1` o `localhost` |

## 3. Namespace y remapeos

Todos los nodos van en `/r2`. Los topics relativos se resuelven bajo `/r2`.

| Nodo | Remapeo | Efecto |
| --- | --- | --- |
| todos (Nav2, AMCL, map_server, EKF, slam_toolbox, robot_state_publisher) | `/tf → tf`, `/tf_static → tf_static` | usan `/r2/tf`, `/r2/tf_static` del Create 3 |
| robot_state_publisher (`mtrb_state_publisher`) | `robot_description → mtrb_description` | no choca con `/r2/robot_description` del robot |
| EKF | `odom → odom_topic`, `imu → imu_topic` | entradas `/r2/odom`, `/r2/imu`; salida `/r2/odometry/filtered` |
| controller_server, behavior_server | `cmd_vel → cmd_vel_nav` | |
| velocity_smoother | `cmd_vel → cmd_vel_nav`, `cmd_vel_smoothed → cmd_vel` | salida final `/r2/cmd_vel` (`Twist`) |
| Nav2 / AMCL / slam_toolbox | `scan_topic` (parámetro, no remap) | `/r2/scan` por defecto |
| mtrb_web_bridge (solo con `web:=true`) | solo los de TF | publica `cmd_vel_nav` e `initialpose`; lee `map`, `scan`, `plan`, `battery_state`, `odom` y TF; usa `navigate_to_pose`, `slam_toolbox/save_map` y `serialize_map`, `<nodo>/get_state` y `navigate_to_pose/_action/cancel_goal` |

- **Regla de la teleop:** la web publica velocidades **solo** en `/r2/cmd_vel_nav` (la entrada del
  `velocity_smoother`), nunca en `/r2/cmd_vel`, que solo publica `velocity_smoother`.
- `odom → base_link` lo publica el Create 3. El EKF corre con `publish_tf: false` (dos publicadores
  romperían el árbol TF); su `/r2/odometry/filtered` la usan controller, bt_navigator y velocity_smoother.
  AMCL y los costmaps usan el TF crudo del robot.
- El launch publica solo `base_link → <lidar_frame>`; el resto del TF del Create 3 viene del robot.
- Pose del dock: en `map` se publica una vez en `/r2/initialpose` cuando AMCL está activo. En `mapping`
  sin pose-graph, el origen del mapa es la pose del robot al arrancar (el dock); con pose-graph se
  pasa `dock_*` como `map_start_pose` de slam_toolbox.

## 4. Compatibilidad Iron (robot) / Jazzy (Pi)

- Los topics estándar (`odom`, `imu`, `tf`, `cmd_vel`, `scan`) no dependen de la distro.
- Los mensajes propios (`irobot_create_msgs`) tienen tipos de Iron: no se usan aquí, pero si los añades
  verifica con `ros2 interface show` que coinciden con los del firmware.
- Nav2 en Jazzy publica `cmd_vel` como `Twist` (sin stamp), que es lo que espera el Create 3. Si un
  cambio de versión lo pasara a `TwistStamped`, el robot dejaría de moverse sin error visible.
- Ambos lados deben usar `rmw_cyclonedds_cpp` y `ROS_DOMAIN_ID=21`; sin multicast, hay que configurar
  `CYCLONEDDS_URI` con peers explícitos.

## 5. Pruebas y estilo

```bash
colcon build --symlink-install && source env.sh
colcon test && colcon test-result --verbose
```

- `mtrb_bringup/test/`: `test_utils.py` y `test_initial_pose.py` (unitarias), `test_params.py`
  (cada nodo del launch tiene su sección en el YAML), `test_launch.py` (argumentos públicos),
  `test_flake8.py` y `test_pep257.py` (estilo).
- Puente web: `test_web_utils.py`, `test_teleop_gate.py`, `test_map_saver.py`, `test_web_hub.py`,
  `test_web_server.py` (sin ROS; necesitan `fastapi` y `httpx`), `test_web_bridge.py` (nodo; necesita
  `rclpy`, se salta en el Mac) y `test_web_frontend_static.py` (comprueba que la UI no usa
  `innerHTML`, `eval` ni scripts o estilos en línea, compatibles con la CSP del servidor).
  `bench_web.py` no es una prueba: mide el coste de la codificación del mapa y del reparto de
  mensajes (ver `test/BENCHMARKS.md`, con el procedimiento para medir la CPU en la Pi).
- `mtrb_description/test/test_urdf.py`: el xacro genera los frames esperados y nada de simulación.
- Estilo: el de ROS 2 (`ament_flake8`, líneas de 99, config en `mtrb_bringup/setup.cfg`).
  `test_utils.py` y `test_params.py` corren también sin ROS: `pytest src/mtrb_bringup/test`.

## 6. Cómo contribuir

1. Cambios de comportamiento → primero en los YAML de `param/`; en el launch solo si hace falta
   un nodo o argumento nuevo (y documenta el argumento arriba).
2. Lógica nueva sin dependencias de ROS → `mtrb_bringup/utils.py` con su prueba en `test_utils.py`.
3. Nodo nuevo → archivo propio en `mtrb_bringup/`, entrada en `setup.py`, dependencias en
   `package.xml`, y su sección en el YAML (añádela a `test_params.py`).
4. Antes de subir: `colcon test` sin fallos.
5. Web: la lógica de decisión (validación, teleop, rutas de mapas) va en `web_utils.py` /
   `web_server.py` sin imports de ROS y con su test; el nodo solo ejecuta. La UI (`web/`) no usa
   build ni CDN, ni `innerHTML`, `eval` ni scripts o estilos en línea (la CSP los bloquearía).
   Todo lo que mueva el robot o escriba en disco necesita tests y revisión de seguridad.

## 7. TODO

`grep -rn TODO src/`: medir la posición real del LiDAR (`lidar_x/y/z`) y ponerla por defecto en el
launch (la altura del plano de escaneo figura como `0.0298` en un comentario del launch y como
`0.0245` en `lidar.urdf.xacro`: unifícala al medirla; la posición x/y es la que importa para que el
scan encaje con el mapa), ajustar inflación y velocidades.

Web (pendiente de probar en el robot, ver 2.1.7):

- Medir la parada (objetivo: menos de 500 ms), la CPU de la Pi con el mapeo en marcha y la
  latencia de pose y mapa (`test/BENCHMARKS.md`).
- Confirmar los techos de velocidad del Create 3 (0,46 m/s y 1,9 rad/s son valores asumidos).
- Comprobar que guardar, cargar y ampliar mapas funciona de extremo a extremo.
- Limitación conocida: tras un objetivo fallido, el estado «Fallido» puede bloquear la teleop hasta
  que termine otro objetivo.
- Verificar la instalación de `fastapi`/`uvicorn` en un entorno virtual con `ros2 launch`.

## 8. Fuentes

- [ldlidar_stl_ros2 (driver oficial de LDROBOT)](https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2):
  parámetros del nodo, launch del LD19, regla udev, baudrate (230400) y recorte de ángulos.
