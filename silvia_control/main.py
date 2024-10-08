import asyncio
import time
import json
import dataclasses
from nicegui import app, ui
from devices import Thermocouple, FlowSensor, PressureSensor, Boiler, Pump, READ_PERIOD
from brew import Brew
from brew_profiles import brew_profiles
from mqtt import MQTT

# Instantiate Devices
boiler_thermo = Thermocouple(1, offset=-7)
grouphead_thermo = Thermocouple(0)
pressure_sensor = PressureSensor(2)
flow_sensor = FlowSensor(23)

pump = Pump(pressure_sensor, flow_sensor)
boiler_thermo.read_temperature()
grouphead_thermo.read_temperature()
pressure_sensor.read_pressure()

# Boiler
boiler = Boiler(boiler_thermo)

# Instantiate Brew Class
brew = Brew(boiler, pump, flow_sensor)

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

# Brew Profile Selection

# Profile Information
profile_data_columns = [
    {'name': 'stage', 'label': 'Stage', 'field': 'stage', 'required': True, 'align': 'left'},
    {'name': 'stage_info', 'label': 'Info', 'field': 'stage_info', 'required': True},
]
profile_data_rows = []


def change_profile_data_table(profile_name: str, table: ui.table):
    profile = brew_profiles[profile_name]
    profile_data_rows.clear()
    for stage in profile:
        profile_data_rows.append(
            {'stage': stage.name, 'stage_info': json.dumps(dataclasses.asdict(stage), indent=1)[1:-1]}
        )
    table.update()


with ui.card():
    with ui.column().classes('w-full items-center'):
        with ui.row().classes('w-full items-center'):
            ui.label("Brew Profile:").classes('mr-2')
            brew_profile_selector = ui.select(list(brew_profiles.keys()), value="londinium").classes('ml-2')
        with ui.expansion('Profile Details', icon='work').classes('w-full'):
            brew_profile_view_table = ui.table(columns=profile_data_columns, rows=profile_data_rows, row_key='stage')
        ui.label("Manual Temperature Control")
        temp_slider = ui.slider(min=0, max=120, value=boiler.setpoint)
        ui.label().bind_text_from(temp_slider, 'value')

temp_slider.on('update:model-value', lambda e: boiler.set_target_temp(e.args), throttle=1.0)
brew_profile_selector.on('update:model-value', lambda e: change_profile_data_table(e.args['label'], brew_profile_view_table))
brew_profile_view_table.add_slot('body-cell', r'''
            <td :props="props" :style="{'white-space':'pre-line'}">{{ props.value }}</td>
        ''')
change_profile_data_table(brew_profile_selector.value, brew_profile_view_table)

# Brew Information
brew_data_columns = [
    {'name': 'stage', 'label': 'Stage', 'field': 'stage', 'required': True, 'align': 'left'},
    {'name': 'logs', 'label': 'Logs', 'field': 'logs'},
]
brew_data_rows = []

with ui.card():
    with ui.row():
        with ui.card():
            brew_status_label = ui.label("Turn on Brew Switch to start Brew")
    with ui.row():
        ui.label("Last brew data:")
    with ui.row():
        last_brew_data_table = ui.table(columns=brew_data_columns, rows=brew_data_rows, row_key='stage')

# Graphs
start_time = time.time()
ui.label("Last brew graphs:")
with ui.tabs().classes('w-full') as tabs:
    one = ui.tab('Pressure')
    two = ui.tab('Temperature')
    three = ui.tab('Combined')
with ui.tab_panels(tabs, value=one).classes('w-full'):
    with ui.tab_panel(one):
        pressure_echart = ui.echart({
            'tooltip': {'trigger': 'axis'},
            'xAxis': {'type': 'value', 'name': '', 'axisLabel': {'formatter': '{value} s'}},
            'yAxis': [{'type': 'value', 'name': 'Pressure (Bars)'}, {'type': 'value', 'name': 'Pump U'}],
            'legend': {'textStyle': {'color': 'gray'}},
            'series': [
                {'type': 'line', 'data': [[0, pressure_sensor.pressure]],
                 'name': 'Pressure', 'smooth': "true", "symbol": "none"},
                {'type': 'line', 'data': [[0, pump.setpoint]],
                 'name': 'Target Pressure', 'smooth': "true", "symbol": "none"},
                {'type': 'line', 'yAxisIndex': 1, 'data': [[0, pump.current_u]],
                 'name': 'Pump U', 'smooth': "true", "symbol": "none"}
            ],
        })
        flow_echart = ui.echart({
            'tooltip': {'trigger': 'axis'},
            'xAxis': {'type': 'value', 'name': '', 'axisLabel': {'formatter': '{value} s'}},
            'yAxis': [{'type': 'value', 'name': 'Total Volume'}, {'type': 'value', 'name': 'Flow Rate'}],
            'legend': {'textStyle': {'color': 'gray'}},
            'series': [
                {'type': 'line', 'yAxisIndex': 0, 'data': [[0, 0]],
                 'name': 'Total Volume', 'smooth': "true", "symbol": "none"},
                {'type': 'line', 'yAxisIndex': 1, 'data': [[0, 0]],
                 'name': 'Flow Rate', 'smooth': "true", "symbol": "none"},
                {'type': 'line', 'yAxisIndex': 1, 'data': [[0, 0]],
                 'name': 'Target Flow Rate', 'smooth': "true", "symbol": "none"}
            ],
        })
    with ui.tab_panel(two):
        temp_echart = ui.echart({
            'tooltip': {'trigger': 'axis'},
            'xAxis': {'type': 'value', 'name': 'Time (s)'},
            'yAxis': [{'type': 'value', 'name': 'Temperature (C)'}, {'type': 'value', 'name': 'Boiler U'}],
            'legend': {'textStyle': {'color': 'gray'}},
            'series': [
                {'type': 'line', 'data': [[0, boiler_thermo.temperature]],
                 'name': 'Boiler Temp', 'smooth': "true", "symbol": "none"},
                {'type': 'line', 'data': [[0, grouphead_thermo.temperature]],
                 'name': 'Grouphead Temp', 'smooth': "true", "symbol": "none"},
                {'type': 'line', 'data': [[0, boiler.setpoint]],
                 'name': 'Target Temp', 'smooth': "true", "symbol": "none"},
                {'type': 'line', 'yAxisIndex': 1, 'data': [[0, boiler.current_u]],
                 'name': 'Boiler U', 'smooth': "true", "symbol": "none"}
            ],
        })
    with ui.tab_panel(three):
        combined_echart = ui.echart({
            'tooltip': {'trigger': 'axis'},
            'xAxis': {'type': 'value', 'name': '', 'axisLabel': {'formatter': '{value} s'}},
            'yAxis': [{'type': 'value', 'name': 'Pressure (Bars)'}, {'type': 'value', 'name': 'Pump U'},
                      {'type': 'value', 'name': 'Total Volume'}, {'type': 'value', 'name': 'Flow Rate'}],
            'legend': {'textStyle': {'color': 'gray'}},
            'series': [
                {'type': 'line', 'data': [[0, pressure_sensor.pressure]],
                 'name': 'Pressure', 'smooth': "true", "symbol": "none"},
                {'type': 'line', 'data': [[0, pump.setpoint]],
                 'name': 'Target Pressure', 'smooth': "true", "symbol": "none"},
                {'type': 'line', 'yAxisIndex': 1, 'data': [[0, pump.current_u]],
                 'name': 'Pump U', 'smooth': "true", "symbol": "none"},
                {'type': 'line', 'yAxisIndex': 2, 'data': [[0, 0.0]],
                 'name': 'Total Volume', 'smooth': "true", "symbol": "none"},
                {'type': 'line', 'yAxisIndex': 3, 'data': [[0, 0.0]],
                 'name': 'Flow Rate', 'smooth': "true", "symbol": "none"},
                {'type': 'line', 'yAxisIndex': 3, 'data': [[0, 0.0]],
                 'name': 'Target Flow Rate', 'smooth': "true", "symbol": "none"}
            ],
        })

# Graph Update Loop
counter = 0


def set_echart_values():
    global start_time
    global counter
    if brew.currently_brewing:
        temp_slider.disable()
        temp_slider.set_value(boiler.setpoint)

        new_point_time = time.time()
        temp_echart.options['series'][0]['data'].append([new_point_time - start_time, boiler_thermo.temperature])
        temp_echart.options['series'][1]['data'].append([new_point_time - start_time, grouphead_thermo.temperature])
        temp_echart.options['series'][2]['data'].append([new_point_time - start_time, boiler.setpoint])
        temp_echart.options['series'][3]['data'].append([new_point_time - start_time, boiler.current_u])

        pressure_echart.options['series'][0]['data'].append([new_point_time - start_time, pressure_sensor.pressure])
        pressure_echart.options['series'][1]['data'].append([new_point_time - start_time, pump.setpoint])
        pressure_echart.options['series'][2]['data'].append([new_point_time - start_time, pump.current_u])

        flow_echart.options['series'][0]['data'].append([new_point_time - start_time, flow_sensor.get_ml])
        flow_echart.options['series'][1]['data'].append([new_point_time - start_time, flow_sensor.get_filtered_flow])
        flow_echart.options['series'][2]['data'].append([new_point_time - start_time, pump.setpoint_flow])

        combined_echart.options['series'][0]['data'].append([new_point_time - start_time, pressure_sensor.pressure])
        combined_echart.options['series'][1]['data'].append([new_point_time - start_time, pump.setpoint])
        combined_echart.options['series'][2]['data'].append([new_point_time - start_time, pump.current_u])
        combined_echart.options['series'][3]['data'].append([new_point_time - start_time, flow_sensor.get_ml])
        combined_echart.options['series'][4]['data'].append(
            [new_point_time - start_time, flow_sensor.get_filtered_flow])
        combined_echart.options['series'][5]['data'].append([new_point_time - start_time, pump.setpoint_flow])

        if counter == 0:
            temp_echart.update()
            counter += 1
        elif counter == 1:
            pressure_echart.update()
            counter += 1
        elif counter == 2:
            flow_echart.update()
            counter += 1
        elif counter == 3:
            combined_echart.update()
            counter += 1
        else:
            counter = 0
    else:
        temp_slider.enable()


def reset_echart():
    global start_time
    start_time = time.time()
    temp_echart.options['series'][0]['data'] = [[0, boiler_thermo.temperature]]
    temp_echart.options['series'][1]['data'] = [[0, grouphead_thermo.temperature]]
    temp_echart.options['series'][2]['data'] = [[0, boiler.setpoint]]
    temp_echart.options['series'][3]['data'] = [[0, boiler.current_u]]

    pressure_echart.options['series'][0]['data'] = [[0, pressure_sensor.pressure]]
    pressure_echart.options['series'][1]['data'] = [[0, pump.setpoint]]
    pressure_echart.options['series'][2]['data'] = [[0, pump.current_u]]

    flow_echart.options['series'][0]['data'] = [[0, 0.0]]
    flow_echart.options['series'][1]['data'] = [[0, 0.0]]
    flow_echart.options['series'][2]['data'] = [[0, 0.0]]

    combined_echart.options['series'][0]['data'] = [[0, pressure_sensor.pressure]]
    combined_echart.options['series'][1]['data'] = [[0, pump.setpoint]]
    combined_echart.options['series'][2]['data'] = [[0, pump.current_u]]
    combined_echart.options['series'][3]['data'] = [[0, 0.0]]
    combined_echart.options['series'][4]['data'] = [[0, 0.0]]
    combined_echart.options['series'][5]['data'] = [[0, 0.0]]


ui.timer(0.1, lambda: set_echart_values())

# PID Debug Panel
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
    [p, i, d, u] = boiler.p_i_d_components
    p_circular.set_value(p)
    i_circular.set_value(i)
    d_circular.set_value(-d)

    [pump_p, pump_i, pump_d, pump_u] = pump.p_i_d_components
    pump_p_circular.set_value(pump_p)
    pump_i_circular.set_value(pump_i)
    pump_d_circular.set_value(-pump_d)


# # FastAPI
# @app.get('/state')
# def espresso_state():
#     return {"current_profile": brew_profile_selector.value, "current_pressure": pressure_sensor.pressure,
#             "current_pump_setpoint": pump.setpoint, "current_brew_stage": brew.current_brew_stage_name,
#             "total_time_taken": brew.total_time_taken}
#
#
# @app.get('/profiles')
# def espresso_state():
#     return list(brew_profiles.keys())
#
#
# @app.get('/set_profile')
# def set_state(profile: str = ""):
#     if profile in list(brew_profiles.keys()):
#         brew_profile_selector.set_value(profile)
#         return True
#     return False


# PID Debug Display Update Loop
ui.timer(0.1, lambda: set_pid_component_ui())

# Instantiate Read Timers with UI's asyncio
ui.timer(READ_PERIOD, lambda: boiler_temp_circular.set_value(boiler_thermo.read_temperature()))
ui.timer(READ_PERIOD, lambda: grouphead_temp_circular.set_value(grouphead_thermo.read_temperature()))
ui.timer(1.0 / 200.0, lambda: pressure_circular.set_value(pressure_sensor.read_pressure()))
ui.timer(FlowSensor.FILTER_SAMPLING_PERIOD, lambda: flow_sensor.filter_flow_rate())
ui.timer(READ_PERIOD, lambda: pump_state_circular.set_value(pump.read_pump_state()))

# Control Loops
app.on_startup(boiler.control_loop())
app.on_startup(pump.control_loop())
app.on_startup(brew.monitor_brew_button(brew_profile_selector, brew_status_label, last_brew_data_table, brew_data_rows,
                                        reset_echart, combined_echart))

# Initialize MQTT
mqtt = MQTT()
app.on_startup(mqtt.mqtt_sync(brew_profile_selector, boiler, pump, flow_sensor, pressure_sensor, brew))
app.on_startup(mqtt.mqtt_sub_profile(brew_profile_selector, change_profile_data_table, brew_profile_view_table))

ui.run(port=80, show=False, title="Espresso", favicon="☕")
