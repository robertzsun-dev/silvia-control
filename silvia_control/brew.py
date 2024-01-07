from enum import Enum
import asyncio
from nicegui import ui
import time

from devices import Boiler, Pump


class Brew:
    class BrewState(Enum):
        IDLE = 0
        FILL = 1
        PREINFUSE = 2
        EXTRACTION_RAMP = 3
        EXTRACTION_HOLD = 4
        EXTRACTION_RAMP_DOWN = 5
        FINISHED = 6

    PERIOD = 1.0 / 100.0  # 100 Hz brew loop

    def __init__(self, boiler: Boiler, pump: Pump):
        self._boiler = boiler
        self._pump = pump
        self._currently_brewing = False
        self._current_state = self.BrewState.IDLE

        self._period = self.PERIOD
        return

    @property
    def currently_brewing(self):
        return self._currently_brewing

    async def _brew(self, last_brew_data_table: ui.table, brew_data_rows: list):
        brew_start_time = time.time()
        fill_end_time = brew_start_time
        preinfuse_end_time = brew_start_time
        extraction_ramp_end_time = brew_start_time
        extraction_hold_end_time = brew_start_time
        extraction_ramp_down_end_time = brew_start_time

        # Initialize states
        self._current_state = self.BrewState.FILL
        preinfuse_start_time = None
        extraction_hold_start_time = None
        self._pump.reset_integrator()
        brew_data_rows.clear()

        # Brew Parameters (Cleaning)
        # target_temperature = 93
        #
        # fill_des_pressure = Pump.PUMP_FULL_ON
        # is_filled_pressure = 15.0
        #
        # preinfuse_pressure = 11.0
        # preinfuse_time = 0.0
        #
        # extraction_pressure = 11.0
        # extraction_pressure_ramp_time = 0.5
        #
        # extraction_hold_time = 30.0
        #
        # extraction_ramp_down_pressure = 11.0
        # extraction_ramp_down_time = 0.5

        # Brew Parameters (Londinium)
        target_temperature = 93

        fill_des_pressure = Pump.PUMP_FILL_TARGET_PRESSURE
        is_filled_pressure = 3.0

        preinfuse_pressure = 3.0
        preinfuse_time = 12.0

        extraction_pressure = 9.0
        extraction_pressure_ramp_time = 2.0

        extraction_hold_time = 3.0

        extraction_ramp_down_pressure = 5
        extraction_ramp_down_time = 20

        # Brew Parameters (Flat 9)
        # target_temperature = 93
        #
        # fill_des_pressure = Pump.PUMP_FILL_TARGET_PRESSURE
        # is_filled_pressure = 0.0
        #
        # preinfuse_pressure = 0.0
        # preinfuse_time = 0.0
        #
        # extraction_pressure = 9.0
        # extraction_pressure_ramp_time = 2.0
        #
        # extraction_hold_time = 120.0
        #
        # extraction_ramp_down_pressure = 9
        # extraction_ramp_down_time = 20

        # Brew Parameters (Blooming)
        # target_temperature = 94
        #
        # fill_des_pressure = Pump.PUMP_FILL_TARGET_PRESSURE
        # is_filled_pressure = 3.0
        #
        # preinfuse_pressure = 0.0
        # preinfuse_time = 20.0
        #
        # extraction_pressure = 9.0
        # extraction_pressure_ramp_time = 5.0
        #
        # extraction_hold_time = 5.0
        #
        # extraction_ramp_down_pressure = 5
        # extraction_ramp_down_time = 6

        # # # Brew Parameters (Turbo)
        # target_temperature = 94
        # fill_des_pressure = Pump.PUMP_FILL_TARGET_PRESSURE
        # is_filled_pressure = 3.0
        #
        # preinfuse_pressure = 6.0
        # preinfuse_time = 0.0
        #
        # self._current_state = self.BrewState.EXTRACTION_RAMP
        # extraction_pressure = 6.0
        # extraction_pressure_ramp_time = 0.5
        #
        # extraction_hold_time = 15.0
        #
        # extraction_ramp_down_pressure = 6.0
        # extraction_ramp_down_time = 0.5

        # Calculated Brew Parameters
        extraction_ramp_up_pressure_slope = ((extraction_pressure - preinfuse_pressure) /
                                             extraction_pressure_ramp_time)
        extraction_ramp_down_pressure_slope = ((extraction_pressure - extraction_ramp_down_pressure) /
                                               extraction_ramp_down_time)

        # Brew loop
        while True:
            # Loop housekeeping
            start_time = time.monotonic()

            # Brew Cycle
            if self._current_state == self.BrewState.FILL:
                self._pump.set_target_pressure(fill_des_pressure)
                if self._pump.current_pressure > is_filled_pressure:
                    self._current_state = self.BrewState.PREINFUSE
                    self._pump.reset_integrator()
                    self._boiler.set_target_temp(target_temperature)
                fill_end_time = time.time()
                # TODO: High temp basket fill, proper temp preinfuse and extraction
            elif self._current_state == self.BrewState.PREINFUSE:
                if preinfuse_start_time is None:
                    preinfuse_start_time = time.time()
                self._pump.set_target_pressure(preinfuse_pressure)
                if time.time() - preinfuse_start_time > preinfuse_time:
                    self._pump.reset_integrator()
                    self._current_state = self.BrewState.EXTRACTION_RAMP
                preinfuse_end_time = time.time()
            elif self._current_state == self.BrewState.EXTRACTION_RAMP:
                extraction_ramp_up_time_elapsed = time.time() - preinfuse_end_time
                actual_des_pressure = (preinfuse_pressure +
                                       extraction_ramp_up_time_elapsed * extraction_ramp_up_pressure_slope)
                if actual_des_pressure >= extraction_pressure:
                    actual_des_pressure = extraction_pressure
                    self._current_state = self.BrewState.EXTRACTION_HOLD
                self._pump.set_target_pressure(actual_des_pressure)
                extraction_ramp_end_time = time.time()
            elif self._current_state == self.BrewState.EXTRACTION_HOLD:
                if extraction_hold_start_time is None:
                    extraction_hold_start_time = time.time()
                self._pump.set_target_pressure(extraction_pressure)
                if time.time() - extraction_hold_start_time > extraction_hold_time:
                    self._current_state = self.BrewState.EXTRACTION_RAMP_DOWN
                extraction_hold_end_time = time.time()
            elif self._current_state == self.BrewState.EXTRACTION_RAMP_DOWN:
                extraction_ramp_down_time_elapsed = time.time() - extraction_hold_end_time
                actual_des_pressure = (extraction_pressure -
                                       extraction_ramp_down_time_elapsed * extraction_ramp_down_pressure_slope)
                if actual_des_pressure <= extraction_ramp_down_pressure:
                    actual_des_pressure = extraction_ramp_down_pressure
                self._pump.set_target_pressure(actual_des_pressure)
                extraction_ramp_down_end_time = time.time()

            # Brew Logs Update Display
            txt = "{time:.2f}"
            if len(brew_data_rows) == 0:
                brew_data_rows.append(
                    {'stage': 'Fill', 'logs': "0"}
                )
                brew_data_rows.append(
                    {'stage': 'PreInfuse', 'logs': "0"}
                )
                brew_data_rows.append(
                    {'stage': 'Extraction Ramp Up', 'logs': "0"}
                )
                brew_data_rows.append(
                    {'stage': 'Extraction Hold', 'logs': "0"}
                )
                brew_data_rows.append(
                    {'stage': 'Extraction Ramp Down', 'logs': "0"}
                )
                brew_data_rows.append(
                    {'stage': 'Total Time', 'logs': txt.format(time=time.time() - brew_start_time)}
                )
            else:
                brew_data_rows[0] = {'stage': 'Fill', 'logs': txt.format(time=fill_end_time - brew_start_time)}
                if preinfuse_end_time != brew_start_time:
                    brew_data_rows[1] = {'stage': 'PreInfuse',
                                         'logs': txt.format(time=preinfuse_end_time - fill_end_time)}
                if extraction_ramp_end_time != brew_start_time:
                    brew_data_rows[2] = {'stage': 'Extraction Ramp Up',
                                         'logs': txt.format(time=extraction_ramp_end_time - preinfuse_end_time)}
                if extraction_hold_end_time != brew_start_time:
                    brew_data_rows[3] = {'stage': 'Extraction Hold',
                                         'logs': txt.format(time=extraction_hold_end_time - extraction_ramp_end_time)}
                if extraction_ramp_down_end_time != brew_start_time:
                    brew_data_rows[4] = {'stage': 'Extraction Ramp Down',
                                         'logs': txt.format(time=extraction_ramp_down_end_time -
                                                                 extraction_hold_end_time)}
                brew_data_rows[5] = {'stage': 'Total Time', 'logs': txt.format(time=time.time() - brew_start_time)}

            last_brew_data_table.update()

            # Finish Brew
            if not self._pump.brew_state:
                break

            # Loop housekeeping
            computation_time = time.monotonic() - start_time
            await asyncio.sleep(self._period - computation_time)

        # Finish brew
        self._current_state = self.BrewState.FINISHED
        self._pump.set_target_pressure(Pump.PUMP_OFF_TARGET_PRESSURE)
        self._boiler.set_target_temp(93 + 5)

    async def brew(self, last_brew_data_table: ui.table, brew_data_rows: list) -> None:
        # sanity check pump state is ON
        if not self._pump.brew_state:
            return
        self._currently_brewing = True
        await self._brew(last_brew_data_table, brew_data_rows)
        self._currently_brewing = False

    async def monitor_brew_button(self, brew_status_label: ui.label, last_brew_data_table: ui.table,
                                  brew_data_rows: list, reset_chart_callback):
        # monitor pump loop
        while True:
            start_time = time.monotonic()

            brew_state = self._pump.brew_state
            if brew_state and self._current_state == self.BrewState.IDLE:
                brew_status_label.set_text("Brewing")
                reset_chart_callback()
                await self.brew(last_brew_data_table, brew_data_rows)
            elif not brew_state:
                self._current_state = self.BrewState.IDLE
                brew_status_label.set_text("Turn on Brew Switch to start brew")
            else:
                brew_status_label.set_text("Brew Complete, Turn off Brew Switch to reset brew")

            computation_time = time.monotonic() - start_time
            await asyncio.sleep(0.05 - computation_time)  # monitor every 0.05 seconds
