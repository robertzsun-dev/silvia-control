import asyncio
import time
from nicegui import app, ui
from devices import Thermocouple, PressureSensor, Boiler, Pump, READ_PERIOD
from brew import Brew

# Instantiate Devices
boiler_thermo = Thermocouple(1, offset=-7)
grouphead_thermo = Thermocouple(0)
pressure_sensor = PressureSensor(2)
boiler_thermo.read_temperature()
grouphead_thermo.read_temperature()
pressure_sensor.read_pressure()


# Boiler
boiler = Boiler(boiler_thermo)

# Instantiate Brew Class
brew = Brew(boiler)

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
            pressure_circular = ui.circular_progress(min=0.0, max=13.0, size="6em")
            ui.label('Pressure (Bars)')

with ui.card():
    with ui.row():
        with ui.column():
            p_circular = ui.circular_progress(min=0.0, max=1500.0, size="6em")
            ui.label('p (0-1500)')
        with ui.column():
            i_circular = ui.circular_progress(min=0.0, max=1500.0, size="6em")
            ui.label('i (0-1500)')
        with ui.column():
            d_circular = ui.circular_progress(min=0.0, max=1500.0, size="6em")
            ui.label('d (0-1500)')


def set_pid_component_ui():
    [p, i, d] = boiler.p_i_d_components
    p_circular.set_value(p)
    i_circular.set_value(i)
    d_circular.set_value(-d)


ui.timer(0.1, lambda: set_pid_component_ui())

start_time = time.time()
# Graphs
temp_echart = ui.echart({
    'xAxis': {'type': 'value'},
    'yAxis': {'type': 'value'},
    'series': [
        {'type': 'line', 'data': [[time.time() - start_time, boiler_thermo.raw_temperature]], 'name': 'Boiler Raw Temp',
         'smooth': "true", "symbol": "none"},
        {'type': 'line', 'data': [[time.time() - start_time, boiler_thermo.temperature]],
         'name': 'Boiler Filtered Temp',
         'smooth': "true", "symbol": "none"},
        {'type': 'line', 'data': [[time.time() - start_time, grouphead_thermo.raw_temperature]],
         'name': 'Grouphead Raw Temp', 'smooth': "true", "symbol": "none"},
        {'type': 'line', 'data': [[time.time() - start_time, grouphead_thermo.temperature]],
         'name': 'Grouphead Filtered Temp', 'smooth': "true", "symbol": "none"},
        {'type': 'line', 'data': [[time.time() - start_time, boiler.setpoint]],
         'name': 'Target temp', 'smooth': "true", "symbol": "none"}
    ],
})

# Graphs
def set_echart_values():
    global start_time
    temp_echart.options['series'][0]['data'].append([time.time() - start_time, boiler_thermo.raw_temperature])
    temp_echart.options['series'][1]['data'].append([time.time() - start_time, boiler_thermo.temperature])
    temp_echart.options['series'][2]['data'].append([time.time() - start_time, grouphead_thermo.raw_temperature])
    temp_echart.options['series'][3]['data'].append([time.time() - start_time, grouphead_thermo.temperature])
    temp_echart.options['series'][4]['data'].append([time.time() - start_time, boiler.setpoint])

    # Reset every 100 ticks (1000 * 0.1 = 100 secs)
    if len(temp_echart.options['series'][1]['data']) > 1000:
        start_time = time.time()
        temp_echart.options['series'][0]['data'] = [[time.time() - start_time, boiler_thermo.raw_temperature]]
        temp_echart.options['series'][1]['data'] = [[time.time() - start_time, boiler_thermo.temperature]]
        temp_echart.options['series'][2]['data'] = [[time.time() - start_time, grouphead_thermo.raw_temperature]]
        temp_echart.options['series'][3]['data'] = [[time.time() - start_time, grouphead_thermo.temperature]]
        temp_echart.options['series'][4]['data'] = [[time.time() - start_time, boiler.setpoint]]

    temp_echart.update()


ui.timer(0.1, lambda: set_echart_values())

with ui.card():
    with ui.row():
        brew_button = ui.button('Brew', on_click=lambda e: brew.brew(e.sender))
        brew_button.props("size=3em")
        switch = ui.switch('AutoBrew')
        brew_button.bind_enabled_from(switch, 'value', backward=lambda x: not x)

# Instantiate Read Timers with UI (lol)
ui.timer(READ_PERIOD, lambda: boiler_temp_circular.set_value(boiler_thermo.read_temperature()))
ui.timer(READ_PERIOD, lambda: grouphead_temp_circular.set_value(grouphead_thermo.read_temperature()))
ui.timer(READ_PERIOD, lambda: pressure_circular.set_value(pressure_sensor.read_pressure()))
ui.timer(READ_PERIOD, lambda: )

# Control Loop
app.on_startup(boiler.control_loop())


ui.run(port=80, show=False)
