# Plan: WiFi Motor/Light Control via NodeMCU (ESP8266)

## Context

Pi GPIO fried. Goal: Pi handles camera only, ESP8266 NodeMCU handles motors + ring light.
Pi sends high-level commands over WiFi HTTP. ESP8266 generates step pulses locally.

**Critical constraint:** Step pulses require microsecond precision (10μs pulses, up to 5000 steps/sec). Pi cannot send pulses over WiFi — ESP8266 must receive `{steps, direction, max_speed, acceleration}` and run trapezoidal velocity profile locally using timer ISR.

**Scope:** Pi-side code changes + HTTP API contract. User writes ESP8266 firmware.

## Architecture

```
Pi (camera + OpenScan3 API)
  └─→ HTTP → ESP8266
                ├─ stepper driver A (rotor)
                ├─ stepper driver B (turntable)  
                ├─ ring light (PWM via analogWrite)
                └─ endstop switches (attachInterrupt)
```

---

## HTTP API Contract (ESP8266 must implement)

All responses JSON. Motor move commands return immediately — Pi polls `/motor/status`.

### Motor Endpoints

| Method | Path | Body | Response |
|--------|------|------|----------|
| POST | `/motor/move` | `{"motor": "rotor", "steps": int, "direction": 1/-1, "max_speed": int, "acceleration": int}` | `{"status": "ok"}` |
| POST | `/motor/home` | `{"motor": "rotor"}` | `{"status": "ok"}` |
| POST | `/motor/stop` | `{"motor": "rotor"}` | `{"status": "ok"}` |
| GET | `/motor/status?motor=rotor` | — | `{"busy": bool}` |
| POST | `/motor/enable` | `{"motor": "rotor", "enabled": bool}` | `{"status": "ok"}` |

Motor names match config: `"rotor"`, `"turntable"` (or whatever is in device JSON).

### Light Endpoints

| Method | Path | Body | Response |
|--------|------|------|----------|
| POST | `/light/on` | — | `{"status": "ok"}` |
| POST | `/light/off` | — | `{"status": "ok"}` |
| POST | `/light/intensity` | `{"value": float}` (0.0–1.0) | `{"status": "ok"}` |
| GET | `/light/status` | — | `{"is_on": bool, "value": float}` |
| GET | `/health` | — | `{"ok": true}` |

### ESP8266 implementation notes

- Use `ESP8266WebServer` or `ESPAsyncWebServer` + `ArduinoJson`
- Stepper: pre-compute step interval array from trapezoidal profile, fire from `timer1_attachInterrupt` ISR — no math in ISR
- Endstop: `attachInterrupt(pin, isr, FALLING)` — ISR sets `stop_requested = true`; stepper ISR checks flag each step
- Light PWM: `analogWrite(pin, 0-1023)`, `analogWriteFreq()` for custom frequency

---

## Pi-Side Changes

### 1. New config model

**New file:** `openscan_firmware/config/remote.py`

```python
from pydantic import BaseModel, Field

class NodeMCUConfig(BaseModel):
    url: str  # e.g. "http://192.168.1.50"
    timeout: float = 5.0
    poll_interval: float = 0.05  # seconds between /motor/status polls
```

### 2. `MotorConfig` — make pins optional

**File:** `openscan_firmware/config/motor.py` (lines 6-8)

```python
direction_pin: Optional[int] = None
enable_pin: Optional[int] = None
step_pin: Optional[int] = None
```

Add validator: all three set OR all None (remote mode). Add `from typing import Optional`.

### 3. `ScannerDeviceConfig` — add nodemcu field

**File:** `openscan_firmware/models/scanner.py`

```python
from openscan_firmware.config.remote import NodeMCUConfig
# Add field:
nodemcu: NodeMCUConfig | None = None
```

### 4. `RemoteMotorController`

**New file:** `openscan_firmware/controllers/hardware/remote_motors.py`

Drop-in for `MotorController`. Same interface (`StatefulHardware`, `SleepCapableHardware`), same public methods.

Key design:
- No `ThreadPoolExecutor` — uses `async/await` + `httpx.AsyncClient`
- `_execute_movement(step_count, degrees)`:
  1. POST `/motor/move` with `{motor: name, steps, direction, max_speed, acceleration}`
  2. Poll `/motor/status?motor=name` with `asyncio.sleep(poll_interval)` until `busy == false`
  3. Track `_stop_requested` flag — if set, POST `/motor/stop` and break
  4. Update `model.angle` from executed steps
- `stop()`: sets `_stop_requested = True` (polling loop picks it up within one poll interval)
- `move_to_endstop()`: large step count; NodeMCU endstop ISR stops it; Pi detects `busy == false`
- `refresh()`: `loop.create_task(self._post("/motor/enable", {...}))` — no GPIO calls
- `calibrate()`: unchanged logic (calls `move_to_endstop()` then `move_to(home_angle)`)
- Does NOT import `gpio.py`

### 5. `RemoteLightController`

**New file:** `openscan_firmware/controllers/hardware/remote_lights.py`

Drop-in for `LightController`. Same interface.

- `turn_on()`: POST `/light/on`
- `turn_off()`: POST `/light/off`
- `set_value(v)`: POST `/light/intensity` with `{"value": v/100.0}`
- `refresh()`: sync → `loop.create_task(...)` to push state
- Does NOT import `gpio.py`

### 6. `device.py` changes

**File:** `openscan_firmware/controllers/device.py`

After config load, detect remote mode:
```python
nodemcu_config = NodeMCUConfig(**config_dict["nodemcu"]) if "nodemcu" in config_dict else None
```

Motor init loop (lines 651-656) — branch:
```python
for name, motor in motor_objects.items():
    try:
        if nodemcu_config:
            controller = RemoteMotorController(motor, nodemcu_config)
            _motor_registry[name] = controller  # plug into existing registry
        else:
            controller = create_motor_controller(motor)
        controller.set_idle_callbacks(is_idle, handle_idle_event)
    except Exception as e:
        logger.error(...)
```

Import `_motor_registry` from `motors.py` alongside existing motor imports.

Endstop init block (lines 659-672) — skip when remote (NodeMCU handles it):
```python
if not nodemcu_config and "endstops" in config_dict:
    # existing endstop init code unchanged
```

Light init loop (lines 676-681) — same branch pattern using `_light_registry`.

### 7. Device config JSON

**New file:** `settings/device/example_remote_nodemcu.json`

```json
{
    "name": "OpenScan Mini (NodeMCU Remote)",
    "model": "mini",
    "nodemcu": {
        "url": "http://192.168.1.50",
        "timeout": 5.0,
        "poll_interval": 0.05
    },
    "cameras": { "camera_0": { "type": "picamera2" } },
    "motors": {
        "rotor": {
            "acceleration": 10000,
            "max_speed": 5000,
            "direction": -1,
            "steps_per_rotation": 10240,
            "min_angle": 0,
            "max_angle": 145,
            "home_angle": 90
        },
        "turntable": {
            "acceleration": 10000,
            "max_speed": 5000,
            "direction": 1,
            "steps_per_rotation": 800,
            "min_angle": 0,
            "max_angle": 360,
            "home_angle": 0
        }
    },
    "lights": {
        "Ringlight": {}
    }
}
```

---

## Files to Create/Modify

| File | Action |
|------|--------|
| `openscan_firmware/config/remote.py` | **Create** — `NodeMCUConfig` |
| `openscan_firmware/controllers/hardware/remote_motors.py` | **Create** — `RemoteMotorController` |
| `openscan_firmware/controllers/hardware/remote_lights.py` | **Create** — `RemoteLightController` |
| `settings/device/example_remote_nodemcu.json` | **Create** — example config |
| `openscan_firmware/config/motor.py` | **Modify** — pins optional |
| `openscan_firmware/models/scanner.py` | **Modify** — add nodemcu field |
| `openscan_firmware/controllers/device.py` | **Modify** — branch on nodemcu_config |

`httpx` already in main dependencies — no `pyproject.toml` change needed.

---

## Verification

1. Flash ESP8266, `curl http://<ip>/health` → `{"ok": true}`
2. `curl -X POST http://<ip>/motor/move -H 'Content-Type: application/json' -d '{"motor":"rotor","steps":200,"direction":1,"max_speed":1000,"acceleration":5000}'` then poll `/motor/status?motor=rotor` until `busy: false`
3. `curl -X POST http://<ip>/light/on`
4. Point Pi at remote config JSON, start OpenScan3, call `/motors/rotor/angle` → motor moves
5. Run full scan — motors move, ring light toggles, camera captures
