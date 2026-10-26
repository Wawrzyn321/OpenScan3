"""Remote Motor Controller

Drop-in replacement for MotorController when motors are driven by a NodeMCU over WiFi.
The NodeMCU receives high-level commands (steps, direction, speed, acceleration) and
generates step pulses locally. This controller sends HTTP requests and polls for completion.
"""

import asyncio
import logging
import math
from typing import Callable, Awaitable, Optional

import httpx

from openscan_firmware.config.motor import MotorConfig
from openscan_firmware.config.remote import NodeMCUConfig
from openscan_firmware.controllers.hardware.interfaces import (
    HardwareEvent,
    StatefulHardware,
    SleepCapableHardware,
)
from openscan_firmware.controllers.services.device_events import (
    notify_busy_change,
    schedule_device_status_broadcast,
)
from openscan_firmware.controllers.settings import Settings
from openscan_firmware.models.motor import Motor
from openscan_firmware.models.paths import PolarPoint3D
from openscan_firmware.utils.inactivity_timer import inactivity_timer, inactivity_timer_paused

logger = logging.getLogger(__name__)


class RemoteMotorController(StatefulHardware, SleepCapableHardware):
    """Controls a stepper motor via HTTP commands to a NodeMCU."""

    def __init__(self, motor: Motor, nodemcu: NodeMCUConfig):
        self.model = motor
        self._nodemcu = nodemcu
        self.settings = Settings(motor.settings, on_change=self._on_settings_change)
        self._current_steps = 0
        self._target_angle: Optional[float] = None
        self._stop_requested = False
        self._calibrated = False
        self.endstop = True  # sentinel: router checks truthiness; NodeMCU handles endstop logic
        self.is_idle = lambda: True
        self.send_event = None

        logger.debug(f"RemoteMotorController for '{self.model.name}' initialized (NodeMCU: {nodemcu.url}).")

    def _on_settings_change(self, settings: MotorConfig) -> None:
        self.model.settings = settings
        schedule_device_status_broadcast([f"motors.{self.model.name}.settings"])
        logger.info(f"Motor '{self.model.name}' settings updated.")

    async def _post(self, path: str, json: dict | None = None, retries: int = 3) -> dict:
        last_exc = None
        for attempt in range(retries):
            try:
                async with httpx.AsyncClient(timeout=self._nodemcu.timeout) as client:
                    r = await client.post(self._nodemcu.url + path, json=json or {})
                    r.raise_for_status()
                    return r.json()
            except (httpx.RemoteProtocolError, httpx.ConnectError, httpx.ReadError) as e:
                last_exc = e
                logger.warning(f"NodeMCU POST {path} attempt {attempt + 1}/{retries} failed: {e}")
                if attempt < retries - 1:
                    await asyncio.sleep(1.0)
        raise last_exc

    async def _get(self, path: str, params: dict | None = None, retries: int = 3) -> dict:
        last_exc = None
        for attempt in range(retries):
            try:
                async with httpx.AsyncClient(timeout=self._nodemcu.timeout) as client:
                    r = await client.get(self._nodemcu.url + path, params=params)
                    r.raise_for_status()
                    return r.json()
            except (httpx.RemoteProtocolError, httpx.ConnectError, httpx.ReadError) as e:
                last_exc = e
                logger.warning(f"NodeMCU GET {path} attempt {attempt + 1}/{retries} failed: {e}")
                if attempt < retries - 1:
                    await asyncio.sleep(1.0)
        raise last_exc

    def is_busy(self) -> bool:
        return self._current_steps > 0

    def get_status(self) -> dict:
        return {
            "name": self.model.name,
            "angle": self.model.angle,
            "busy": self.is_busy(),
            "target_angle": self._target_angle,
            "settings": self.get_config(),
            "calibrated": self.is_calibrated(),
            "endstop": None,
        }

    def get_config(self) -> MotorConfig:
        return self.settings.model

    def refresh(self):
        if self.is_idle():
            logger.info(f"Motor '{self.model.name}' idle.")
            self._calibrated = False
            self._schedule_enable(False)
        else:
            logger.info(f"Motor '{self.model.name}' active.")
            self._schedule_enable(True)

    def _schedule_enable(self, enabled: bool):
        try:
            loop = asyncio.get_running_loop()
            loop.create_task(
                self._post("/motor/enable", {"motor": self.model.name, "enabled": enabled})
            )
        except RuntimeError:
            pass

    def is_calibrated(self) -> bool:
        return self._calibrated

    def set_idle_callbacks(
        self,
        is_idle: Callable[[], bool],
        send_event: Callable[[HardwareEvent], Awaitable[None]],
    ) -> None:
        self.is_idle = is_idle
        self.send_event = send_event

    def stop(self) -> None:
        self._stop_requested = True
        logger.info(f"Motor '{self.model.name}' stop requested.")

    def _normalize_target_angle(self, desired_angle: float) -> float:
        min_limit = self.settings.min_angle
        max_limit = self.settings.max_angle

        if min_limit == 0.0 and max_limit == 360.0:
            return desired_angle % 360

        final_angle = desired_angle
        if final_angle < min_limit:
            logger.warning(
                f"Desired angle {desired_angle:.2f} below minimum {min_limit:.2f}. Clamping."
            )
            final_angle = min_limit
        elif final_angle > max_limit:
            logger.warning(
                f"Desired angle {desired_angle:.2f} above maximum {max_limit:.2f}. Clamping."
            )
            final_angle = max_limit

        return final_angle

    def estimate_movement_time(self, steps: int) -> float:
        steps = abs(steps)
        if steps == 0:
            return 0.0

        max_accel = self.settings.acceleration
        max_speed = self.settings.max_speed
        accel_time = max_speed / max_accel
        accel_steps = int(0.5 * max_accel * accel_time * accel_time)

        if 2 * accel_steps > steps:
            accel_steps = max(steps // 2, 1)
            peak_time = math.sqrt(2 * accel_steps / max_accel)
            return 2 * peak_time
        else:
            const_steps = steps - (2 * accel_steps)
            const_time = const_steps / max_speed if const_steps > 0 else 0
            return 2 * accel_time + const_time

    def estimate_movement_time_for_degrees(self, degrees: float) -> float:
        spr = self.settings.steps_per_rotation
        return self.estimate_movement_time(int(abs(degrees) * spr / 360))

    def estimate_move_to_time(self, target_degrees: float) -> float:
        target_angle = self._normalize_target_angle(target_degrees % 360)
        move_angles = target_angle - self.model.angle
        if move_angles > 180:
            move_angles -= 360
        elif move_angles < -180:
            move_angles += 360
        return self.estimate_movement_time_for_degrees(abs(move_angles))

    async def move_to(self, degrees: float) -> None:
        if self.is_busy():
            raise RuntimeError("Motor is busy")
        self._stop_requested = False
        target_angle = self._normalize_target_angle(degrees % 360)
        await self._move_to_target_angle(target_angle)

    async def move_degrees(self, degrees: float) -> None:
        if self.is_busy():
            raise RuntimeError("Motor is busy")
        self._stop_requested = False
        target_angle = self._normalize_target_angle(self.model.angle + degrees)
        await self._move_to_target_angle(target_angle)

    async def _move_to_target_angle(self, target_angle: float) -> None:
        spr = self.settings.steps_per_rotation
        direction = self.settings.direction
        degrees_to_move = target_angle - self.model.angle

        if self.settings.min_angle == 0.0 and self.settings.max_angle == 360.0:
            if degrees_to_move > 180:
                degrees_to_move -= 360
            elif degrees_to_move < -180:
                degrees_to_move += 360

        step_count = int(degrees_to_move * spr / 360) * direction
        logger.debug(f"Motor '{self.model.name}' moving {step_count} steps to {target_angle:.2f}°.")
        await self._execute_movement(step_count, target_angle)

    async def move_to_endstop(self) -> None:
        logger.debug(f"Motor '{self.model.name}' homing via NodeMCU.")
        if self.is_idle():
            await self.send_event(HardwareEvent.MOVE_EVENT)

        async with inactivity_timer_paused:
            self._current_steps = 1  # mark busy
            notify_busy_change("motors", self.model.name)
            try:
                await self._post("/motor/home", {"motor": self.model.name})
                while True:
                    if self._stop_requested:
                        await self._post("/motor/stop", {"motor": self.model.name})
                        break
                    await asyncio.sleep(self._nodemcu.poll_interval)
                    status = await self._get("/motor/status", {"motor": self.model.name})

                    if not status.get("busy", False):
                        break
            finally:
                self._current_steps = 0
                self._target_angle = None
                notify_busy_change("motors", self.model.name)

    async def move_to_home(self) -> None:
        inactivity_timer.reset()
        await self.send_event(HardwareEvent.HOME_EVENT)
        await self.move_to(self.settings.home_angle)

    async def calibrate(self, *, force: bool = False) -> None:
        if self._calibrated and not force:
            inactivity_timer.reset()
            return
        self._calibrated = False
        await self.move_to_endstop()
        await asyncio.sleep(3)
        await self.move_to(self.settings.home_angle)
        self._calibrated = True

    async def _execute_movement(
        self, step_count: int, requested_degrees: float, stop_on_endstop: bool = False
    ) -> None:
        if self.is_idle():
            logger.debug(f"Motor '{self.model.name}': device idle, waking before move.")
            await self.send_event(HardwareEvent.MOVE_EVENT)

        async with inactivity_timer_paused:
            self._current_steps = abs(step_count)
            self._target_angle = requested_degrees
            self._last_endstop_triggered = False
            notify_busy_change("motors", self.model.name)

            try:
                if step_count == 0:
                    return

                direction = 1 if step_count > 0 else -1
                await self._post("/motor/move", {
                    "motor": self.model.name,
                    "steps": abs(step_count),
                    "direction": direction,
                    "max_speed": self.settings.max_speed,
                    "acceleration": self.settings.acceleration,
                })

                # Poll until NodeMCU reports not busy
                while True:
                    if self._stop_requested:
                        logger.info(f"Motor '{self.model.name}': stop requested, sending stop to NodeMCU.")
                        await self._post("/motor/stop", {"motor": self.model.name})
                        break

                    await asyncio.sleep(self._nodemcu.poll_interval)

                    status = await self._get("/motor/status", {"motor": self.model.name})


                    if not status.get("busy", False):
                        break

                # Update angle based on what NodeMCU actually executed.
                # NodeMCU may report actual steps; fall back to requested count.
                executed_steps = abs(step_count)
                spr = self.settings.steps_per_rotation
                dir_multiplier = 1 if step_count >= 0 else -1
                executed_degrees = (executed_steps / spr * 360) * dir_multiplier * self.settings.direction
                self.model.angle = (self.model.angle + executed_degrees) % 360
                logger.debug(f"Motor '{self.model.name}' moved {executed_degrees:.2f}°, now at {self.model.angle:.2f}°.")

            except asyncio.CancelledError:
                logger.info(f"Motor '{self.model.name}' movement cancelled.")
                try:
                    await self._post("/motor/stop", {"motor": self.model.name})
                except Exception:
                    pass
                raise
            finally:
                self._current_steps = 0
                self._target_angle = None
                notify_busy_change("motors", self.model.name)
