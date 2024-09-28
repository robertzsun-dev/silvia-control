import asyncio
from typing import List
from enum import Enum
import time

import aiomqtt
from nicegui import ui

from devices import Boiler, Pump, FlowSensor, PressureSensor
from brew import Brew


class MQTT:
    def __init__(self):
        return

    async def mqtt_sync(self, brew_profile_selector: ui.select, boiler: Boiler, pump: Pump,
                        flow_sensor: FlowSensor, pressure_sensor: PressureSensor, brew: Brew):
        # monitor pump loop
        while True:
            try:
                async with aiomqtt.Client("127.0.0.1") as client:
                    start_time = time.monotonic()

                    while True:
                        await asyncio.gather(
                            client.publish(topic="espresso/current_profile", payload=brew_profile_selector.value),
                            client.publish(topic="espresso/current_pressure", payload=pressure_sensor.pressure),
                            client.publish(topic="espresso/current_pump_setpoint", payload=pump.setpoint),
                            client.publish(topic="espresso/current_brew_stage", payload=brew.current_brew_stage_name),
                            client.publish(topic="espresso/total_time_taken", payload=brew.total_time_taken),
                        )

                        computation_time = time.monotonic() - start_time
                        # print(computation_time)
                        await asyncio.sleep(0.05 - computation_time)
                        start_time = time.monotonic()
            except aiomqtt.MqttError:
                print(f"Connection lost; Reconnecting in {1} seconds ...")
                await asyncio.sleep(1)
