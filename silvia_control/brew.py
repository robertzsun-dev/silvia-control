import asyncio
from nicegui import ui
import time

from devices import Boiler


class Brew:
    def __init__(self, boiler: Boiler):
        self._boiler = boiler
        return

    async def _brew(self, button: ui.button):
        start_time = time.time()
        for i in range(0, 500):
            await asyncio.sleep(0.01)
            txt = "{time:.2f}"
            button.set_text(txt.format(time=time.time() - start_time))

    async def brew(self, button: ui.button) -> None:
        button.disable()
        await self._brew(button)
        button.enable()
        button.set_text("Brew")
