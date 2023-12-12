import asyncio
from nicegui import ui
from devices import Thermocouple, PressureSensor, READ_PERIOD
from brew import Brew

# Instantiate Devices
boiler_thermo = Thermocouple()
grouphead_thermo = Thermocouple()
pressure_sensor = PressureSensor()

# Instantiate Brew Class
brew = Brew()

# Instantiate UI Elements
with ui.card():
    with ui.row():
        with ui.column():
            boiler_temp_circular = ui.circular_progress(min=0.0, max=150.0, size="6em")
            ui.label('Boiler Temp (C)')
        with ui.column():
            grouphead_temp_circular = ui.circular_progress(min=0.0, max=150.0, size="6em")
            ui.label('Grouphead Temp (C)')
        with ui.column():
            pressure_circular = ui.circular_progress(min=0.0, max=150.0, size="6em")
            ui.label('Pressure (Bars)')

with ui.card():
    with ui.row():
        brew_button = ui.button('Brew', on_click=lambda e: brew.brew(e.sender))
        brew_button.props("size=3em")
        switch = ui.switch('AutoBrew')
        brew_button.bind_enabled_from(switch, 'value', backward=lambda x: not x)

# Instantiate Read Timers with UI (lol)
ui.timer(READ_PERIOD, lambda: boiler_temp_circular.set_value(boiler_thermo.temperature))
ui.timer(READ_PERIOD, lambda: grouphead_temp_circular.set_value(grouphead_thermo.temperature))
ui.timer(READ_PERIOD, lambda: pressure_circular.set_value(pressure_sensor.pressure))

ui.run()
