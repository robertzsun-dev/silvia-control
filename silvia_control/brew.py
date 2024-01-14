import asyncio
from typing import List
from enum import Enum

from nicegui import ui
import time

from devices import Boiler, Pump, FlowSensor
from brew_profiles import brew_profiles, TransitionType, TargetType, BrewStage


class BrewState(Enum):
    IDLE = 0
    BREWING = 2
    FINISHED = 3


class Brew:
    PERIOD = 1.0 / 100.0  # 100 Hz brew loop

    def __init__(self, boiler: Boiler, pump: Pump, flow_sensor: FlowSensor):
        self._boiler = boiler
        self._pump = pump
        self._flow_sensor = flow_sensor
        self._currently_brewing = False
        self._current_state = BrewState.IDLE

        self._period = self.PERIOD
        return

    @property
    def currently_brewing(self):
        return self._currently_brewing

    async def _brew(self, brew_profile: List[BrewStage], last_brew_data_table: ui.table, brew_data_rows: list):
        brew_start_time = time.monotonic()
        cur_brew_stage_start_time = time.monotonic()
        cur_brew_stage_starting_pressure = self._pump.current_pressure
        cur_brew_stage_starting_flow = self._flow_sensor.get_filtered_flow

        # Initialize states
        self._current_state = BrewState.BREWING
        self._pump.reset_integrator()
        self._flow_sensor.reset_ticks()
        brew_data_rows.clear()
        self.brew_stage_idx = 0
        self._boiler.set_turbo(True)

        # Brew loop
        while True:
            # Loop housekeeping
            start_time = time.monotonic()

            # Brew
            brew_stage = brew_profile[self.brew_stage_idx]
            self._boiler.set_target_temp(brew_stage.target_temperature)

            if brew_stage.target_type == TargetType.PRESSURE:
                time_elapsed = time.monotonic() - cur_brew_stage_start_time
                if brew_stage.ramp_time > 0.0:
                    ramp_up_pressure_slope = ((brew_stage.target - cur_brew_stage_starting_pressure) /
                                              brew_stage.ramp_time)
                    actual_des_pressure = (cur_brew_stage_starting_pressure +
                                           time_elapsed * ramp_up_pressure_slope)
                    if actual_des_pressure >= brew_stage.target:
                        actual_des_pressure = brew_stage.target
                else:
                    actual_des_pressure = brew_stage.target
                self._pump.set_flow_mode(False)
                self._pump.set_target_pressure(actual_des_pressure)
            elif brew_stage.target_type == TargetType.FLOW:
                time_elapsed = time.monotonic() - cur_brew_stage_start_time
                if brew_stage.ramp_time > 0.0:
                    ramp_up_flow_slope = ((brew_stage.target - cur_brew_stage_starting_flow) /
                                          brew_stage.ramp_time)
                    actual_des_flow = (cur_brew_stage_starting_flow +
                                       time_elapsed * ramp_up_flow_slope)
                    if actual_des_flow >= brew_stage.target:
                        actual_des_flow = brew_stage.target
                else:
                    actual_des_flow = brew_stage.target
                self._pump.set_flow_mode(True)
                self._pump.set_target_flow(actual_des_flow)
            elif brew_stage.target_type == TargetType.FILL:
                self._pump.set_flow_mode(False)
                self._pump.set_target_pressure(brew_stage.target)
            elif brew_stage.target_type == TargetType.FLOW_WITH_PRESSURE_LIMIT:
                self._pump.set_flow_mode(True, brew_stage.pressure_limit)
                self._pump.set_target_flow(brew_stage.target)

            # Transitions
            transition = False
            if time.monotonic() - cur_brew_stage_start_time >= brew_stage.maximum_time:
                transition = True
            elif (brew_stage.transition_type == TransitionType.PRESSURE_OVER
                  and self._pump.current_pressure > brew_stage.transition_parameter):
                transition = True
            elif (brew_stage.transition_type == TransitionType.PRESSURE_UNDER
                  and self._pump.current_pressure < brew_stage.transition_parameter):
                transition = True
            elif (brew_stage.transition_type == TransitionType.FLOW_OVER
                  and self._flow_sensor.get_filtered_flow > brew_stage.transition_parameter):
                transition = True
            elif (brew_stage.transition_type == TransitionType.FLOW_UNDER
                  and self._flow_sensor.get_filtered_flow < brew_stage.transition_parameter):
                transition = True
            if transition:
                # Transition Brew Stage
                cur_brew_stage_start_time = time.monotonic()
                cur_brew_stage_starting_pressure = self._pump.current_pressure
                cur_brew_stage_starting_flow = self._flow_sensor.get_filtered_flow
                self._pump.reset_integrator()
                self.brew_stage_idx += 1

            # Brew Logs Update Display
            txt = "{time:.2f}"
            if len(brew_data_rows) == 0:
                for brew_stage_row in brew_profile:
                    brew_data_rows.append(
                        {'stage': brew_stage_row.name, 'logs': "0"}
                    )
                brew_data_rows.append(
                    {'stage': 'Total Time', 'logs': txt.format(time=time.monotonic() - brew_start_time)})
            else:
                brew_data_rows[self.brew_stage_idx]['logs'] = txt.format(
                    time=time.monotonic() - cur_brew_stage_start_time)
                brew_data_rows[len(brew_profile)]['logs'] = txt.format(time=time.monotonic() - brew_start_time)
            last_brew_data_table.update()

            # Finish Brew
            if not self._pump.brew_state or self.brew_stage_idx >= len(brew_profile):
                break

            # Loop housekeeping
            computation_time = time.monotonic() - start_time
            await asyncio.sleep(self._period - computation_time)

        # Finish brew
        self._current_state = BrewState.FINISHED
        self._pump.set_target_pressure(Pump.PUMP_OFF_TARGET_PRESSURE)
        self._boiler.set_turbo(False)
        self._boiler.set_target_temp(93 + 5)

    async def brew(self, brew_profile: List[BrewStage], last_brew_data_table: ui.table, brew_data_rows: list) -> None:
        # sanity check pump state is ON
        if not self._pump.brew_state:
            return
        self._currently_brewing = True
        await self._brew(brew_profile, last_brew_data_table, brew_data_rows)
        self._currently_brewing = False

    async def monitor_brew_button(self, brew_profile_selector: ui.select, brew_status_label: ui.label,
                                  last_brew_data_table: ui.table, brew_data_rows: list, reset_chart_callback):
        # monitor pump loop
        while True:
            start_time = time.monotonic()

            brew_state = self._pump.brew_state
            if brew_state and self._current_state == BrewState.IDLE:
                brew_status_label.set_text("Brewing")
                reset_chart_callback()
                await self.brew(brew_profiles[brew_profile_selector.value], last_brew_data_table, brew_data_rows)
            elif not brew_state:
                self._current_state = BrewState.IDLE
                brew_status_label.set_text("Turn on Brew Switch to start brew")
            else:
                brew_status_label.set_text("Brew Complete, Turn off Brew Switch to reset brew")

            computation_time = time.monotonic() - start_time
            await asyncio.sleep(0.05 - computation_time)  # monitor every 0.05 seconds
