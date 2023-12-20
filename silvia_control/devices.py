import time
import asyncio
import numpy as np

import pigpio
import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

INPUT_VOLTAGE = 5.0
READ_FREQUENCY = 30
READ_PERIOD = 1.0 / READ_FREQUENCY

pi = pigpio.pi()
i2c = busio.I2C(board.SCL, board.SDA)
ads = ADS.ADS1115(i2c)
ads.gain = 2.0 / 3.0


class LowPassSinglePole:
    def __init__(self, decay):
        self.y = None
        self.b = 1 - decay
        self.reset()

    def reset(self):
        self.y = 0

    def filter(self, x):
        self.y += self.b * (x - self.y)
        return self.y


class Thermocouple:
    _last_voltage: float = 0.0
    _last_temp: float = 0.0
    _last_filtered_temp: float = 0.0
    _adc: AnalogIn = None
    _lp_filter: LowPassSinglePole = None

    def __init__(self, channel_number: int, offset: int = 0):
        if channel_number == 0:
            self._adc = AnalogIn(ads, ADS.P0)
        elif channel_number == 1:
            self._adc = AnalogIn(ads, ADS.P1)
        elif channel_number == 2:
            self._adc = AnalogIn(ads, ADS.P2)
        self._lp_filter = LowPassSinglePole(0.95)
        self._offset = offset

    def read_voltage(self) -> float:
        self._last_voltage = self._adc.voltage
        return self._last_voltage

    def read_temperature(self) -> float:
        """
        Returns temperature in Celsius (filtered), to the 2nd decimal
        Calls a read from the ADC and filters value, should only be called in one loop at stable freq
        :return: float Celsius
        """
        self._last_temp = self.voltage_to_temperature(self.read_voltage())
        self._last_filtered_temp = self._lp_filter.filter(self._last_temp)
        return round(self._last_filtered_temp, 2)

    @property
    def temperature(self) -> float:
        """
        Returns filtered temperature in Celsius, to the 2nd decimal
        :return: float Celsius
        """
        return round(self._last_filtered_temp, 2)

    @property
    def raw_temperature(self) -> float:
        """
        Returns raw temperature in Celsius, to the 2nd decimal
        :return: float Celsius
        """
        return round(self._last_temp, 2)

    def voltage_to_temperature(self, voltage: float) -> float:
        return (50.0 * (40.0 * voltage - 11.0 * INPUT_VOLTAGE) / (9 * INPUT_VOLTAGE)) + self._offset


class PressureSensor:
    _last_voltage: float = 0.0
    _last_pressure: float = 0.0
    _last_filtered_pressure: float = 0.0
    _adc: AnalogIn = None
    _lp_filter: LowPassSinglePole = None

    def __init__(self, channel_number: int):
        if channel_number == 0:
            self._adc = AnalogIn(ads, ADS.P0)
        elif channel_number == 1:
            self._adc = AnalogIn(ads, ADS.P1)
        elif channel_number == 2:
            self._adc = AnalogIn(ads, ADS.P2)
        self._lp_filter = LowPassSinglePole(0.4)

    def read_voltage(self) -> float:
        self._last_voltage = self._adc.voltage
        return self._last_voltage

    def read_pressure(self) -> float:
        """
        Returns Pressure in bars (filtered), to the 2nd decimal
        Calls a read from the ADC and filters value, should only be called in one loop at stable freq
        :return: float Bars
        """
        self._last_pressure = self.voltage_to_pressure(self.read_voltage())
        self._last_filtered_pressure = self._lp_filter.filter(self._last_pressure)
        return round(self._last_filtered_pressure, 2)

    @property
    def raw_pressure(self) -> float:
        """
        Returns Raw Pressure in bars
        :return: float Bars
        """
        return round(self._last_pressure, 2)

    @property
    def pressure(self) -> float:
        """
        Returns Filtered Pressure in bars
        :return: float Bars
        """
        return round(self._last_filtered_pressure, 2)

    @staticmethod
    def voltage_to_pressure(voltage: float) -> float:
        return 2.558 * (voltage - 0.6)


class Boiler:
    BOILER_PIN = 13
    PWM_FREQUENCY = 5
    PWM_RANGE = 1500

    def __init__(self, boiler_thermometer: Thermocouple):
        pi.set_mode(self.BOILER_PIN, pigpio.OUTPUT)
        pi.set_PWM_frequency(self.BOILER_PIN, self.PWM_FREQUENCY)
        assert pi.get_PWM_frequency(self.BOILER_PIN) == self.PWM_FREQUENCY
        pi.set_PWM_range(self.BOILER_PIN, self.PWM_RANGE)
        pi.set_PWM_dutycycle(self.BOILER_PIN, 0)

        self._boiler_thermometer = boiler_thermometer

        self._period = 1.0 / 5.0
        self._kp = 200
        self._kd = 2500
        self._ki = 0.05
        self._last_temp_error = 0
        self._lp_filter_derivative = LowPassSinglePole(0.9)
        self._integrated_temp_error = 0

        self._p_component = 0.0
        self._i_component = 0.0
        self._d_component = 0.0

        self._target_temperature = 98

    @property
    def setpoint(self):
        return self._target_temperature

    @property
    def p_i_d_components(self):
        return [int(self._p_component), int(self._i_component), int(self._d_component)]

    async def control_loop(self):
        while True:
            start_time = time.monotonic()

            # Compute control
            current_temp = self._boiler_thermometer.temperature
            temp_error = self._target_temperature - current_temp
            derivative = self._lp_filter_derivative.filter((temp_error - self._last_temp_error) / self._period)
            self._last_temp_error = temp_error
            self._integrated_temp_error += temp_error
            if self._integrated_temp_error < 0 or temp_error < 0:
                self._integrated_temp_error = 0
            if self._ki * self._integrated_temp_error > self.PWM_RANGE / 5.0:
                self._integrated_temp_error = self.PWM_RANGE / (self._ki * 5.0)

            self._p_component = self._kp * temp_error
            self._d_component = self._kd * derivative
            self._i_component = self._ki * self._integrated_temp_error
            u = self._p_component + self._i_component + self._d_component

            # Saturate
            if u > self.PWM_RANGE:
                u = self.PWM_RANGE
            if u < 0:
                u = 0

            # Set PWM
            pi.set_PWM_dutycycle(self.BOILER_PIN, int(u))

            computation_time = time.monotonic() - start_time
            await asyncio.sleep(self._period - computation_time)


class Pump:
    PUMP_PWM_PIN = 12  # Also PWM0
    BREW_PIN = 16  # Digital Input
    PWM_FREQUENCY = 1000
    PWM_RANGE = 1500

    FEED_FORWARD = [[800, 1.7], [850, 2.7], [900, 3.5], [950, 4.0], [1000, 5.5], [1050, 6.2], [1100, 7.7], [1150, 8.8],
                    [1200, 9.4], [1250, 10.0], [1300, 10.3], [1350, 10.7], [1500, 11.0]]

    def __init__(self, pressure_sensor: PressureSensor):
        self._pressure_sensor = pressure_sensor
        pi.set_mode(self.BREW_PIN, pigpio.INPUT)
        pi.set_pull_up_down(self.BREW_PIN, pigpio.PUD_UP)

        pi.set_mode(self.PUMP_PWM_PIN, pigpio.OUTPUT)
        pi.set_PWM_frequency(self.PUMP_PWM_PIN, self.PWM_FREQUENCY)
        assert pi.get_PWM_frequency(self.PUMP_PWM_PIN) == self.PWM_FREQUENCY
        pi.set_PWM_range(self.PUMP_PWM_PIN, self.PWM_RANGE)
        pi.set_PWM_dutycycle(self.PUMP_PWM_PIN, self.PWM_RANGE)

        self._feed_forward = np.array(self.FEED_FORWARD)

        self._period = 1.0 / 100.0
        self._kp = 50
        self._kd = 0  # 20
        self._ki = 20
        self._last_pressure_error = 0
        self._lp_filter_derivative = LowPassSinglePole(0.98)
        self._integrated_pressure_error = 0

        self._p_component = 0.0
        self._i_component = 0.0
        self._d_component = 0.0

        self._target_pressure = 9.0

    def read_pump_state(self):
        return 1 - pi.read(self.BREW_PIN)

    def set_pump_pwm(self, value):
        if value < 0:
            value = 0
        if value > self.PWM_RANGE:
            value = self.PWM_RANGE

        value = self.PWM_RANGE - value
        pi.set_PWM_dutycycle(self.PUMP_PWM_PIN, value)

    @property
    def setpoint(self):
        return self._target_pressure

    @property
    def p_i_d_components(self):
        return [int(self._p_component), int(self._i_component), int(self._d_component)]

    async def control_loop(self):
        while True:
            start_time = time.monotonic()

            # Compute control
            current_pressure = self._pressure_sensor.pressure
            pressure_error = self._target_pressure - current_pressure
            derivative = self._lp_filter_derivative.filter((pressure_error - self._last_pressure_error) / self._period)
            self._last_pressure_error = pressure_error
            self._integrated_pressure_error += pressure_error
            if self._ki * self._integrated_pressure_error < -600.0:
                self._integrated_pressure_error = -100.0 / self._ki
            if self._ki * self._integrated_pressure_error > 600.0:
                self._integrated_pressure_error = 100.0 / self._ki

            self._p_component = self._kp * pressure_error
            self._d_component = self._kd * derivative
            self._i_component = self._ki * self._integrated_pressure_error
            u_ff = np.interp(self._target_pressure, self._feed_forward[:, 1], self._feed_forward[:, 0])
            u = u_ff + self._p_component + self._i_component + self._d_component

            # Saturate
            if u > self.PWM_RANGE:
                u = self.PWM_RANGE
            if u < 0:
                u = 0

            # Set PWM
            self.set_pump_pwm(int(u))

            computation_time = time.monotonic() - start_time
            await asyncio.sleep(self._period - computation_time)
