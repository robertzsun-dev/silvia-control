import asyncio
import time
from nicegui import app, ui
from devices import Thermocouple, PressureSensor, Boiler, Pump, READ_PERIOD
from brew import Brew

# Instantiate Devices
boiler_thermo = Thermocouple(1, offset=-7)
grouphead_thermo = Thermocouple(0)
pressure_sensor = PressureSensor(2)
pump = Pump(pressure_sensor)
boiler_thermo.read_temperature()
grouphead_thermo.read_temperature()
pressure_sensor.read_pressure()

# Boiler
boiler = Boiler(boiler_thermo)

# Instantiate Brew Class
brew = Brew(boiler)

# Main Gauge Displays
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

# Graphs
start_time = time.time()
with ui.tabs().classes('w-full') as tabs:
    one = ui.tab('Pressure')
    two = ui.tab('Temperature')
with ui.tab_panels(tabs, value=one).classes('w-full'):
    with ui.tab_panel(one):
        pressure_echart = ui.echart({
            'xAxis': {'type': 'value'},
            'yAxis': {'type': 'value'},
            'legend': {'textStyle': {'color': 'gray'}},
            'series': [
                {'type': 'line', 'data': [[time.time() - start_time, pressure_sensor.pressure]],
                 'name': 'Pressure', 'smooth': "true", "symbol": "none"},
                {'type': 'line', 'data': [[time.time() - start_time, pump.setpoint]],
                 'name': 'Target Pressure', 'smooth': "true", "symbol": "none"}
            ],
        })
    with ui.tab_panel(two):
        temp_echart = ui.echart({
            'xAxis': {'type': 'value'},
            'yAxis': {'type': 'value'},
            'legend': {'textStyle': {'color': 'gray'}},
            'series': [
                {'type': 'line', 'data': [[time.time() - start_time, boiler_thermo.temperature]],
                 'name': 'Boiler Temp', 'smooth': "true", "symbol": "none"},
                {'type': 'line', 'data': [[time.time() - start_time, grouphead_thermo.temperature]],
                 'name': 'Grouphead Temp', 'smooth': "true", "symbol": "none"},
                {'type': 'line', 'data': [[time.time() - start_time, boiler.setpoint]],
                 'name': 'Target Temp', 'smooth': "true", "symbol": "none"}
            ],
        })


# Graph Update Loop
def set_echart_values():
    global start_time
    temp_echart.options['series'][0]['data'].append([time.time() - start_time, boiler_thermo.temperature])
    temp_echart.options['series'][1]['data'].append([time.time() - start_time, grouphead_thermo.temperature])
    temp_echart.options['series'][2]['data'].append([time.time() - start_time, boiler.setpoint])

    pressure_echart.options['series'][0]['data'].append([time.time() - start_time, pressure_sensor.pressure])
    pressure_echart.options['series'][1]['data'].append([time.time() - start_time, pump.setpoint])

    # Reset every 100 ticks (1000 * 0.1 = 100 secs)
    if len(temp_echart.options['series'][0]['data']) > 1000:
        start_time = time.time()
        temp_echart.options['series'][0]['data'] = [[time.time() - start_time, boiler_thermo.temperature]]
        temp_echart.options['series'][1]['data'] = [[time.time() - start_time, grouphead_thermo.temperature]]
        temp_echart.options['series'][2]['data'] = [[time.time() - start_time, boiler.setpoint]]

        pressure_echart.options['series'][0]['data'] = [[time.time() - start_time, pressure_sensor.pressure]]
        pressure_echart.options['series'][1]['data'] = [[time.time() - start_time, pump.setpoint]]

    temp_echart.update()
    pressure_echart.update()


ui.timer(0.1, lambda: set_echart_values())

# PID Debug Panel
with ui.card():
    with ui.row():
        brew_button = ui.button('Brew', on_click=lambda e: brew.brew(e.sender))
        brew_button.props("size=3em")
        switch = ui.switch('AutoBrew')
        brew_button.bind_enabled_from(switch, 'value', backward=lambda x: not x)

with ui.expansion('PID Debug', icon='work').classes('w-full'):
    with ui.card():
        ui.label('Boiler PID')
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

    with ui.card():
        ui.label('Pump PID')
        with ui.row():
            with ui.column():
                pump_p_circular = ui.circular_progress(min=0.0, max=1500.0, size="6em")
                ui.label('p (0-1500)')
            with ui.column():
                pump_i_circular = ui.circular_progress(min=0.0, max=1500.0, size="6em")
                ui.label('i (0-1500)')
            with ui.column():
                pump_d_circular = ui.circular_progress(min=0.0, max=1500.0, size="6em")
                ui.label('d (0-1500)')
            with ui.column():
                pump_state_circular = ui.circular_progress(min=0.0, max=1.0, size="6em")
                ui.label('Pump State')


def set_pid_component_ui():
    [p, i, d] = boiler.p_i_d_components
    p_circular.set_value(p)
    i_circular.set_value(i)
    d_circular.set_value(-d)

    [pump_p, pump_i, pump_d] = pump.p_i_d_components
    pump_p_circular.set_value(pump_p)
    pump_i_circular.set_value(pump_i)
    pump_d_circular.set_value(-pump_d)


# PID Debug Display Update Loop
ui.timer(0.1, lambda: set_pid_component_ui())

# Instantiate Read Timers with UI (lol)
ui.timer(READ_PERIOD, lambda: boiler_temp_circular.set_value(boiler_thermo.read_temperature()))
ui.timer(READ_PERIOD, lambda: grouphead_temp_circular.set_value(grouphead_thermo.read_temperature()))
ui.timer(1.0 / 100.0, lambda: pressure_circular.set_value(pressure_sensor.read_pressure()))
ui.timer(READ_PERIOD, lambda: pump_state_circular.set_value(pump.read_pump_state()))

# Control Loop
app.on_startup(boiler.control_loop())
app.on_startup(pump.control_loop())

ui.run(port=80, show=False)
