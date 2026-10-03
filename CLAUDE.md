# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

The computer-side half of a two-part system to fly a **DJI Mini SE** by hand gestures seen by the *computer's webcam* (not the drone's camera). The other half is the Android app **Drone Bridge** at `../drone-bridge` (`/home/jean/projetos/drone-bridge`), which is a dumb bridge: it has no vision logic, it just forwards ROS2 commands from here to the DJI SDK and streams the drone's own camera back for monitoring. See that project's `CLAUDE.md` for the Android/DJI side, and its `PROTOCOLO.md` / `MANUAL_DESENVOLVEDOR.md` for the contract and the mandatory safety rules for any computer-side program (this package follows them).

This is a ROS2 **Humble** `ament_python` workspace (`colcon build`), not a plain Python script collection — the entry points are ROS2 nodes, run via `ros2 launch`, not `python foo.py`.

## Why ROS2 + rosbridge instead of ROS2-native on both ends

Android has no `rclpy`/`rclcpp`. The phone talks to this computer over a WebSocket using the rosbridge JSON protocol (`rosbridge_suite`, `ros-humble-rosbridge-suite`), which bridges to genuine ROS2 topics. From this package's point of view the phone is just another ROS2 node — publish/subscribe normally, rosbridge handles the translation. Don't try to add a native Android ROS2 client; it doesn't exist for this SDK generation.

## Build / run

```bash
./setup_ambiente.sh          # once: installs ROS2 Humble + rosbridge-suite (apt, needs sudo) + a --system-site-packages venv with mediapipe/opencv/numpy/matplotlib
source /opt/ros/humble/setup.bash
source .venv/bin/activate
python3 -m colcon build --symlink-install
source install/setup.bash
ros2 launch ponte_visao_drone sistema.launch.py
```

- The venv **must** be created with `--system-site-packages` — `rclpy` comes from the apt-installed ROS2, not pip, and a normal venv would hide it.
- **Build with `python3 -m colcon build`, never bare `colcon build`.** The apt-installed `colcon` executable has its own hardcoded `#!/usr/bin/python3` shebang, so invoking it directly ignores whatever venv is active — the generated `install/.../lib/ponte_visao_drone/piloto_gestos` entry-point script ends up shebanged to the *system* Python too, which can't see `mediapipe` (pip-installed only in `.venv`), and the node dies with `ModuleNotFoundError: No module named 'mediapipe'` at launch. Running colcon as a module (`python3 -m colcon ...`) under the activated venv's `python3` makes `sys.executable` (and so the generated shebang) correctly point at `.venv/bin/python3`. If a build was ever done the wrong way, `rm -rf build install log` before rebuilding — stale generated scripts don't get their shebang fixed by a later `--symlink-install` rebuild alone.
- **`matplotlib` must be pip-installed into `.venv` itself, not left to the system package.** `mediapipe.tasks.python.vision` eagerly imports `drawing_utils`, which imports `matplotlib` — unrelated to anything this project actually plots, but the import happens regardless. Because the venv is `--system-site-packages`, `pip install -r requirements.txt` sees Ubuntu's `python3-matplotlib` (apt, compiled against numpy 1.x) as "already satisfied" and skips it — but `mediapipe`/`opencv-python` pull in numpy 2.x, and the numpy-1-ABI-compiled system matplotlib then crashes on import (`AttributeError: _ARRAY_API not found`). Downgrading numpy instead just moves the break to `opencv-python` (it hard-requires numpy>=2). The fix is `pip install --force-reinstall matplotlib` inside the venv: a fresh numpy-2-compatible wheel lands in `.venv/lib/python3.10/site-packages`, which comes before the system `dist-packages` on `sys.path`, so it shadows the incompatible one. `setup_ambiente.sh` does this automatically; if dependencies ever get reinstalled by hand, redo that step too.
- `setup_ambiente.sh` is idempotent: it skips the ROS2/apt install if `ros2` is already on `PATH`.
- Quick webcam-only sanity check without ROS2 or a drone: `.venv/bin/python -m ponte_visao_drone.visao_mao`.

## Architecture

- `visao_mao.py`: pure gesture detection (MediaPipe `HandLandmarker`), copied from the standalone prototype at `/home/jean/projetos/visao_movimentos_mao/`. **Detection only** — no ROS2, no drone semantics. Keep it that way; command mapping lives in `integracao_visao_drone.py`. See its own module docstring/the prototype's `CLAUDE.md` for the classification algorithm details (finger ratios, palm-size motion window, stabilization).
- `comando_voo.py`: **pure** logic (no ROS, no webcam, no MediaPipe — gestures come in by `Gesto.name`): gesture→`(x,y,z,yaw)` mapping, speed constants (`VELOCIDADE_*` = 0.2 of the app's limits), `Suavizador` (ramps: 0.3 s up, 0.15 s down, sign flips pass through zero) and `FiltroRetorno` (the opposite gesture within 0.8 s of a movement is the hand coming back, so it becomes "hover"). Unit-tested in `test/test_comando_voo.py` (`python3 -m pytest test/test_comando_voo.py`, no venv needed). Put anything that turns input into numbers here.
- `integracao_visao_drone.py`: the `piloto_gestos` ROS2 node. The webcam loop runs on the main thread (calls `DetectorGestos.processar()`, `definir_gesto`); a `SingleThreadedExecutor` spins the node in a background thread so the 20 Hz timer (`_passo`, publishes `/drone/cmd_vel`), the `/phone/telemetria` and `/phone/rpc/response` callbacks never depend on the webcam rate (state is guarded by an `RLock`). It follows the Drone Bridge safety rules: starts disarmed (`a` key); arming needs fresh telemetry, `drone_conectado` and a subscriber on `/drone/stick_mode`; a stale gesture (>0.3 s) publishes zeros; telemetry `stick_mode:false` after a 3 s grace or lost telemetry disarms and never re-arms; every exit path (`q`, Ctrl+C, SIGTERM, exception, webcam failure) goes through `desarmar()` (zero Twist, then `stick_mode:false`) in `main()`'s `finally`. `rclpy.init(signal_handler_options=SignalHandlerOptions.NO)` is required on Humble, otherwise Ctrl+C invalidates the context before the disarm can publish. Takeoff/land go over RPC (`flight.take_off`/`flight.land`, unique ids, answer shown in the window), not the fire-and-forget `/drone/takeoff`/`/drone/land` topics; `t` needs a second press within 3 s and takeoff refuses with no telemetry, no drone or when already flying. `camera_index` is a ROS parameter (launch arg `camera_index:=N`).
- `visor_camera_drone.py`: the `visor_camera_drone` ROS2 node. Subscribes to the drone video the phone relays and shows it in an OpenCV window. Pure monitoring — never touches control.
- Axis convention is **REP 103** (x=forward, y=left, z=up, yaw+=counter-clockwise) on the `Twist` messages published here. The phone is responsible for converting this into whatever sign/axis convention the DJI SDK's `FlightControlData` actually wants (that SDK has its own well-documented pitch/roll-swap quirk — see `drone-bridge`'s `CLAUDE.md`). Don't "fix" the convention here to match DJI directly; keep this side ROS-idiomatic so other ROS tooling/nodes added later aren't surprised.
- Topic contract (publishers here → phone subscribes; phone publishes → nodes here subscribe) is documented in `README.md`. It's deliberately plain topics with standard or `std_msgs/String`-JSON payloads (no custom `.msg` package) so extending it later (new command, new telemetry field) never requires a `colcon` message-generation round trip — just start publishing/subscribing the new topic from either side.

## Non-obvious pitfalls

- `rosbridge_server`'s ROS2 port works with arbitrary message types via `rosidl` introspection — it is not limited to a fixed allowlist, so new topic types "just work" through the bridge without extra configuration on either end.
- `sensor_msgs/CompressedImage.data` arrives in this process as a normal `bytes`-like array (`uint8[]`) — rosbridge already decodes the base64 the phone sends over the WebSocket before it reaches this ROS2 node. Don't re-base64-decode it here.
- The hand-tracking model (`ponte_visao_drone/modelos/hand_landmarker.task`) is bundled, but `DetectorGestos.__init__` also auto-downloads it if missing — don't remove that fallback, it's what makes a fresh checkout runnable without manually fetching the model.
- Ctrl+C inside `ros2 launch` delivers **two** SIGINTs to each node (the terminal signals the whole process group and launch forwards it again). `piloto_gestos` installs `_encerrar` for SIGINT/SIGTERM, which ignores later signals so a second one can't interrupt `desarmar()` in the `finally` (without it the node segfaulted on exit under launch). `visor_camera_drone` does the same around its cleanup.
- `rclpy` loggers reject different severities from the same call site (`getattr(logger, level)(...)` raises `ValueError`); keep distinct `log.info/warning/error` lines, as `_avisar` does.
- OpenCV `putText` cannot draw accented characters — HUD strings are ASCII on purpose.
- Hardware behavior (whether the chosen axis signs/speeds actually feel right on a real aircraft, Wi-Fi hotspot range/latency between phone and computer in the field) has not been verified — this has only been exercised against synthetic/webcam input so far.
