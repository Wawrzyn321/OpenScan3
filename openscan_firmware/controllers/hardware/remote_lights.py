"""Remote Light Controller

Drop-in replacement for LightController when lights are driven by a NodeMCU over WiFi.
Sends HTTP commands to the NodeMCU for on/off/brightness control.
"""

import asyncio
import logging
from typing import Callable, Awaitable

import httpx

from openscan_firmware.config.light import LightConfig
from openscan_firmware.config.remote import NodeMCUConfig
from openscan_firmware.controllers.hardware.interfaces import (
    HardwareEvent,
    SwitchableHardware,
    SleepCapableHardware,
)
from openscan_firmware.controllers.services.device_events import schedule_device_status_broadcast
from openscan_firmware.controllers.settings import Settings
from openscan_firmware.models.light import Light
from openscan_firmware.utils.inactivity_timer import inactivity_timer

logger = logging.getLogger(__name__)


class RemoteLightController(SwitchableHardware, SleepCapableHardware):
    """Controls a ring light via HTTP commands to a NodeMCU."""

    def __init__(self, light: Light, nodemcu: NodeMCUConfig):
        self.model = light
        self._nodemcu = nodemcu
        self.settings = Settings(light.settings, on_change=self._on_settings_change)
        self._is_on = False
        self._value = 100.0

        self.is_idle = lambda: True
        self.send_event = None

        logger.debug(f"RemoteLightController for '{self.model.name}' initialized (NodeMCU: {nodemcu.url}).")

    def _on_settings_change(self, settings: LightConfig) -> None:
        self.model.settings = settings
        schedule_device_status_broadcast([f"lights.{self.model.name}.settings"])
        logger.info(f"Light '{self.model.name}' settings updated.")

    async def _post(self, path: str, json: dict | None = None) -> dict:
        async with httpx.AsyncClient(timeout=self._nodemcu.timeout) as client:
            r = await client.post(self._nodemcu.url + path, json=json or {})
            r.raise_for_status()
            return r.json()

    async def _get(self, path: str) -> dict:
        async with httpx.AsyncClient(timeout=self._nodemcu.timeout) as client:
            r = await client.get(self._nodemcu.url + path)
            r.raise_for_status()
            return r.json()

    def get_status(self) -> dict:
        return {
            "name": self.model.name,
            "is_on": self.is_on,
            "value": self._value,
            "settings": self.get_config().model_dump(),
        }

    def get_config(self) -> LightConfig:
        return self.settings.model

    def refresh(self):
        inactivity_timer.reset()
        if self.is_idle():
            logger.info(f"Light '{self.model.name}' idle.")
            self._schedule_post("/light/off")
        else:
            logger.info(f"Light '{self.model.name}' active.")
            if self._is_on:
                self._schedule_post("/light/on")
                self._schedule_post("/light/intensity", {"value": self._value / 100.0})
            else:
                self._schedule_post("/light/off")

    def _schedule_post(self, path: str, json: dict | None = None):
        try:
            loop = asyncio.get_running_loop()
            loop.create_task(self._post(path, json))
        except RuntimeError:
            pass

    def set_idle_callbacks(
        self,
        is_idle: Callable[[], bool],
        send_event: Callable[[HardwareEvent], Awaitable[None]],
    ) -> None:
        self.is_idle = is_idle
        self.send_event = send_event

    async def _wake_if_idle(self, event: HardwareEvent) -> None:
        if not self.is_idle():
            self.refresh()
            return
        logger.info(f"Light '{self.model.name}': device idle, waking.")
        if self.send_event is not None:
            await self.send_event(event)

    @property
    def is_on(self) -> bool:
        return self._is_on

    async def turn_on(self):
        self._is_on = True
        await self._wake_if_idle(HardwareEvent.LIGHT_EVENT)
        logger.info(f"Light '{self.model.name}' turned on.")
        schedule_device_status_broadcast([f"lights.{self.model.name}.is_on"])

    async def turn_off(self):
        self._is_on = False
        await self._wake_if_idle(HardwareEvent.LIGHT_EVENT)
        logger.info(f"Light '{self.model.name}' turned off.")
        schedule_device_status_broadcast([f"lights.{self.model.name}.is_on"])

    async def set_value(self, value: float):
        self._value = max(0.0, min(100.0, value))
        await self._wake_if_idle(HardwareEvent.LIGHT_EVENT)
        schedule_device_status_broadcast([f"lights.{self.model.name}.value"])
        logger.info(f"Light '{self.model.name}' value set to {self._value}.")
