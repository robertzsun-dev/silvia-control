# import pigpio
import random

INPUT_VOLTAGE = 5.0
READ_FREQUENCY = 30
READ_PERIOD = 1.0 / READ_FREQUENCY


class Thermocouple:
    _last_voltage: float = 0.0

    def __init__(self):
        return

    def read_voltage(self) -> float:
        self._last_voltage = random.gauss(3.5, 1.0)
        return self._last_voltage

    @property
    def temperature(self) -> float:
        """
        Returns temperature in Celsius
        :return: float Celsius
        """
        return round(self.voltage_to_temperature(self.read_voltage()), 2)

    @staticmethod
    def voltage_to_temperature(voltage: float) -> float:
        return 50.0 * (40.0 * voltage - 11.0 * INPUT_VOLTAGE) / (9 * INPUT_VOLTAGE)


class PressureSensor:
    _last_voltage: float = 0.0

    def __init__(self):
        return

    def read_voltage(self) -> float:
        self._last_voltage = random.gauss(2.5, 1.5)
        return self._last_voltage

    @property
    def pressure(self) -> float:
        """
        Returns Pressure in bars
        :return: float Bars
        """
        return round(self.voltage_to_pressure(self.read_voltage()), 2)

    @staticmethod
    def voltage_to_pressure(voltage: float) -> float:
        return 16.0 * (voltage - 0.5)
