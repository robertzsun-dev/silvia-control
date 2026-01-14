from devices import Pump
from enum import Enum
from dataclasses import dataclass


class TransitionType(str, Enum):
    NONE = "NONE"
    PRESSURE_OVER = "PRESSURE_OVER"
    PRESSURE_UNDER = "PRESSURE_UNDER"
    FLOW_OVER = "FLOW_OVER"
    FLOW_UNDER = "FLOW_UNDER"
    VOLUME_OVER = "VOLUME_OVER"


class TargetType(str, Enum):
    PRESSURE = "PRESSURE"
    FLOW = "FLOW"
    FILL = "FILL"
    FLOW_WITH_PRESSURE_LIMIT = "FLOW_WITH_PRESSURE_LIMIT"  # uses previous stage's pressure as limit


@dataclass
class BrewStage:
    target_temperature: float
    target_type: TargetType
    target: float
    pressure_limit: float
    ramp_time: float
    maximum_time: float
    transition_type: TransitionType
    transition_parameter: float
    name: str
    turbo_heating: bool

    def __init__(self, target_temperature: float, target_type: TargetType, target: float, pressure_limit: float,
                 ramp_time: float, maximum_time: float, transition_type: TransitionType, transition_parameter: float,
                 name: str, turbo_heating: bool = True):
        self.target_temperature = target_temperature
        self.target_type = target_type
        self.target = target
        self.pressure_limit = pressure_limit
        self.ramp_time = ramp_time
        self.maximum_time = maximum_time
        self.transition_type = transition_type
        self.transition_parameter = transition_parameter
        self.name = name
        self.turbo_heating = turbo_heating


cleaning = [
    BrewStage(
        target_temperature=93.0,
        target_type=TargetType.FILL,
        target=Pump.PUMP_FULL_ON,
        pressure_limit=0.0,
        ramp_time=0.0,
        maximum_time=60.0,
        transition_type=TransitionType.NONE,
        transition_parameter=0.0,
        name="Pump Full On"
    )
]

lever = [
    BrewStage(
        target_temperature=93.0,
        target_type=TargetType.FILL,
        target=Pump.PUMP_FILL_TARGET_PRESSURE,
        pressure_limit=0.0,
        ramp_time=0.0,
        maximum_time=10.0,
        transition_type=TransitionType.PRESSURE_OVER,
        transition_parameter=3.0,
        name="Fill"
    ),
    BrewStage(
        target_temperature=93.0,
        target_type=TargetType.PRESSURE,
        target=2.5,
        pressure_limit=0.0,
        ramp_time=0.0,
        maximum_time=4.0,
        transition_type=TransitionType.PRESSURE_UNDER,
        transition_parameter=2.0,
        name="Preinfuse2.5Pressure"
    ),
    BrewStage(
        target_temperature=93.0,
        target_type=TargetType.PRESSURE,
        target=9.0,
        pressure_limit=0.0,
        ramp_time=3.0,
        maximum_time=6.0,
        transition_type=TransitionType.NONE,
        transition_parameter=0.0,
        name="Extract"
    ),
    BrewStage(
        target_temperature=93.0,
        target_type=TargetType.PRESSURE,
        target=4.0,
        pressure_limit=0.0,
        ramp_time=38,
        maximum_time=60,
        transition_type=TransitionType.NONE,
        transition_parameter=0.0,
        name="ExtractRampDown"
    ),
    BrewStage(
        target_temperature=93.0,
        target_type=TargetType.PRESSURE,
        target=4.0,
        pressure_limit=0.0,
        ramp_time=0.0,
        maximum_time=90.0,
        transition_type=TransitionType.NONE,
        transition_parameter=0.0,
        name="ExtractFinal"
    )
]

cremina_lever = [
    BrewStage(
        target_temperature=93.0,
        target_type=TargetType.FILL,
        target=Pump.PUMP_FILL_TARGET_PRESSURE,
        pressure_limit=0.0,
        ramp_time=0.0,
        maximum_time=10.0,
        transition_type=TransitionType.PRESSURE_OVER,
        transition_parameter=3.0,
        name="Fill"
    ),
    BrewStage(
        target_temperature=93.0,
        target_type=TargetType.PRESSURE,
        target=2.0,
        pressure_limit=0.0,
        ramp_time=0.0,
        maximum_time=10.0,
        transition_type=TransitionType.NONE,
        transition_parameter=0.0,
        name="Preinfuse2.0Pressure"
    ),
    BrewStage(
        target_temperature=93.0,
        target_type=TargetType.PRESSURE,
        target=9.0,
        pressure_limit=0.0,
        ramp_time=10.0,
        maximum_time=10.0,
        transition_type=TransitionType.NONE,
        transition_parameter=0.0,
        name="Extract"
    ),
    BrewStage(
        target_temperature=93.0,
        target_type=TargetType.PRESSURE,
        target=3.0,
        pressure_limit=0.0,
        ramp_time=50.0,
        maximum_time=50.0,
        transition_type=TransitionType.NONE,
        transition_parameter=0.0,
        name="ExtractRampDown"
    ),
    BrewStage(
        target_temperature=93.0,
        target_type=TargetType.PRESSURE,
        target=3.0,
        pressure_limit=0.0,
        ramp_time=0.0,
        maximum_time=90.0,
        transition_type=TransitionType.NONE,
        transition_parameter=0.0,
        name="ExtractFinal"
    )
]

londinium = [
    BrewStage(
        target_temperature=93.0,
        target_type=TargetType.FILL,
        target=Pump.PUMP_FILL_TARGET_PRESSURE,
        pressure_limit=0.0,
        ramp_time=0.0,
        maximum_time=10.0,
        transition_type=TransitionType.PRESSURE_OVER,
        transition_parameter=2.0,
        name="Fill"
    ),
    BrewStage(
        target_temperature=93.0,
        target_type=TargetType.PRESSURE,
        target=3.0,
        pressure_limit=0.0,
        ramp_time=0.0,
        maximum_time=12.0,
        transition_type=TransitionType.NONE,
        transition_parameter=0.0,
        name="PreinfusePressurized"
    ),
    BrewStage(
        target_temperature=93.0,
        target_type=TargetType.PRESSURE,
        target=9.0,
        pressure_limit=0.0,
        ramp_time=0.0,
        maximum_time=90.0,
        transition_type=TransitionType.NONE,
        transition_parameter=0.0,
        name="Extract"
    )
]

LRv3 = [
    BrewStage(
        target_temperature=93.0,
        target_type=TargetType.FILL,
        target=Pump.PUMP_FILL_TARGET_PRESSURE,
        pressure_limit=0.0,
        ramp_time=0.0,
        maximum_time=10.0,
        transition_type=TransitionType.PRESSURE_OVER,
        transition_parameter=2.0,
        name="Fill"
    ),
    BrewStage(
        target_temperature=93.0,
        target_type=TargetType.PRESSURE,
        target=3.0,
        pressure_limit=0.0,
        ramp_time=0.0,
        maximum_time=12.0,
        transition_type=TransitionType.NONE,
        transition_parameter=0.0,
        name="PreinfusePressurized"
    ),
    BrewStage(
        target_temperature=93.0,
        target_type=TargetType.PRESSURE,
        target=9.0,
        pressure_limit=0.0,
        ramp_time=0.0,
        maximum_time=4.0,
        transition_type=TransitionType.NONE,
        transition_parameter=0.0,
        name="Extract"
    ),
    BrewStage(
        target_temperature=93.0,
        target_type=TargetType.PRESSURE,
        target=9.0,
        pressure_limit=0.0,
        ramp_time=0.0,
        maximum_time=30.0,
        transition_type=TransitionType.FLOW_OVER,
        transition_parameter=1.9,
        name="ExtractHold"
    ),
    BrewStage(
        target_temperature=93.0,
        target_type=TargetType.PRESSURE,
        target=5.5,
        pressure_limit=0.0,
        ramp_time=35.5,
        maximum_time=35.5,
        transition_type=TransitionType.FLOW_OVER,
        transition_parameter=2.8,
        name="ExtractRampDown"
    ),
    BrewStage(
        target_temperature=93.0,
        target_type=TargetType.PRESSURE,
        target=5.5,
        pressure_limit=0.0,
        ramp_time=0.0,
        maximum_time=1.0,
        transition_type=TransitionType.NONE,
        transition_parameter=0.0,
        name="5BarHold"
    ),
    BrewStage(
        target_temperature=93.0,
        target_type=TargetType.PRESSURE,
        target=5.5,
        pressure_limit=0.0,
        ramp_time=0.0,
        maximum_time=127.0,
        transition_type=TransitionType.FLOW_OVER,
        transition_parameter=2.8,
        name="5BarTransition"
    ),
    BrewStage(
        target_temperature=93.0,
        target_type=TargetType.PRESSURE,
        target=4.0,
        pressure_limit=0.0,
        ramp_time=2.0,
        maximum_time=127.0,
        transition_type=TransitionType.NONE,
        transition_parameter=0.0,
        name="4BarHold"
    ),
#    BrewStage(
#        target_temperature=93.0,
#        target_type=TargetType.FLOW,
#        target=2.2,
#        pressure_limit=0.0,
#        ramp_time=0.0,
#        maximum_time=127.0,
#        transition_type=TransitionType.NONE,
#        transition_parameter=0.0,
#        name="FlowLimit"
#    )
]

ExtractamundoDos = [
    BrewStage(
        target_temperature=93.0,
        target_type=TargetType.PRESSURE,
        target=8.0,
        pressure_limit=0.0,
        ramp_time=0.0,
        maximum_time=20.0,
        transition_type=TransitionType.PRESSURE_OVER,
        transition_parameter=4.5,
        name="Preinfuse"
    ),
    BrewStage(
        target_temperature=93.0,
        target_type=TargetType.PRESSURE,
        target=0.0,
        pressure_limit=0.0,
        ramp_time=0.0,
        maximum_time=40.0,
        transition_type=TransitionType.PRESSURE_UNDER,
        transition_parameter=2.2,
        name="DynamicBloom"
    ),
    BrewStage(
        target_temperature=93.0,
        target_type=TargetType.PRESSURE,
        target=6.0,
        pressure_limit=0.0,
        ramp_time=0.0,
        maximum_time=60.0,
        transition_type=TransitionType.FLOW_OVER,
        transition_parameter=3.5,
        name="6bar"
    ),
    BrewStage(
        target_temperature=93.0,
        target_type=TargetType.PRESSURE,
        target=4.5,
        pressure_limit=0.0,
        ramp_time=2.5,
        maximum_time=127.0,
        transition_type=TransitionType.NONE,
        transition_parameter=0.0,
        name="4.5BarHold"
    ),
#    BrewStage(
#        target_temperature=93.0,
#        target_type=TargetType.FLOW_WITH_PRESSURE_LIMIT,
#        target=3.5,
#        pressure_limit=6.0,
#        ramp_time=0.0,
#        maximum_time=60.0,
#        transition_type=TransitionType.NONE,
#        transition_parameter=0.0,
#        name="6bar_flow_limit"
#    )
]

flat9 = [
    BrewStage(
        target_temperature=93.0,
        target_type=TargetType.PRESSURE,
        target=9.0,
        pressure_limit=0.0,
        ramp_time=2.0,
        maximum_time=120.0,
        transition_type=TransitionType.NONE,
        transition_parameter=0.0,
        name="Extract"
    )
]

turbo = [
    BrewStage(
        target_temperature=93.0,
        target_type=TargetType.FILL,
        target=Pump.PUMP_FILL_TARGET_PRESSURE,
        pressure_limit=0.0,
        ramp_time=0.0,
        maximum_time=20.0,
        transition_type=TransitionType.PRESSURE_OVER,
        transition_parameter=4.0,
        name="Preinfuse"
    ),
    BrewStage(
        target_temperature=93.0,
        target_type=TargetType.PRESSURE,
        target=0.0,
        pressure_limit=0.0,
        ramp_time=0.0,
        maximum_time=40.0,
        transition_type=TransitionType.PRESSURE_UNDER,
        transition_parameter=2.2,
        name="DynamicBloom"
    ),
    BrewStage(
        target_temperature=93.0,
        target_type=TargetType.PRESSURE,
        target=6.0,
        pressure_limit=0.0,
        ramp_time=0.0,
        maximum_time=40.0,
        transition_type=TransitionType.NONE,
        transition_parameter=0.0,
        name="6bar"
    )
]

turbobloom = [
    BrewStage(
        target_temperature=93.0,
        target_type=TargetType.FILL,
        target=Pump.PUMP_FILL_TARGET_PRESSURE,
        pressure_limit=0.0,
        ramp_time=0.0,
        maximum_time=20.0,
        transition_type=TransitionType.PRESSURE_OVER,
        transition_parameter=4.0,
        name="Preinfuse"
    ),
    BrewStage(
        target_temperature=93.0,
        target_type=TargetType.PRESSURE,
        target=0.0,
        pressure_limit=0.0,
        ramp_time=0.0,
        maximum_time=40.0,
        transition_type=TransitionType.PRESSURE_UNDER,
        transition_parameter=2.2,
        name="DynamicBloom"
    ),
    BrewStage(
        target_temperature=93.0,
        target_type=TargetType.PRESSURE,
        target=6.0,
        pressure_limit=0.0,
        ramp_time=0.0,
        maximum_time=4.0,
        transition_type=TransitionType.NONE,
        transition_parameter=0.0,
        name="ramp6bar"
    ),
    BrewStage(
        target_temperature=93.0,
        target_type=TargetType.FLOW_WITH_PRESSURE_LIMIT,
        target=4.5,
        pressure_limit=0.0,
        ramp_time=0.0,
        maximum_time=2.0,
        transition_type=TransitionType.NONE,
        transition_parameter=0.0,
        name="6bar_flow_limit"
    ),
    BrewStage(
        target_temperature=93.0,
        target_type=TargetType.PRESSURE,
        target=3.0,
        pressure_limit=0.0,
        ramp_time=0.0,
        maximum_time=0.0,
        transition_type=TransitionType.NONE,
        transition_parameter=0.0,
        name="3bar"
    ),
    BrewStage(
        target_temperature=93.0,
        target_type=TargetType.FLOW_WITH_PRESSURE_LIMIT,
        target=4.5,
        pressure_limit=0.0,
        ramp_time=40.0,
        maximum_time=2.0,
        transition_type=TransitionType.NONE,
        transition_parameter=0.0,
        name="decline"
    )
]

blooming = [
    BrewStage(
        target_temperature=94.0,
        target_type=TargetType.FILL,
        target=Pump.PUMP_FILL_TARGET_PRESSURE,
        pressure_limit=0.0,
        ramp_time=0.0,
        maximum_time=10.0,
        transition_type=TransitionType.PRESSURE_OVER,
        transition_parameter=3.0,
        name="Fill"
    ),
    BrewStage(
        target_temperature=93.0,
        target_type=TargetType.PRESSURE,
        target=0.0,
        pressure_limit=0.0,
        ramp_time=0.0,
        maximum_time=20.0,
        transition_type=TransitionType.NONE,
        transition_parameter=0.0,
        name="PreinfuseNoPressure"
    ),
    BrewStage(
        target_temperature=93.0,
        target_type=TargetType.PRESSURE,
        target=9.0,
        pressure_limit=0.0,
        ramp_time=5.0,
        maximum_time=17.0,
        transition_type=TransitionType.NONE,
        transition_parameter=0.0,
        name="Extract"
    ),
    BrewStage(
        target_temperature=93.0,
        target_type=TargetType.PRESSURE,
        target=5.0,
        pressure_limit=0.0,
        ramp_time=10.0,
        maximum_time=60.0,
        transition_type=TransitionType.NONE,
        transition_parameter=0.0,
        name="ExtractRampDown"
    )
]

oolong_gongfu = [
    BrewStage(
        target_temperature=94.0,
        target_type=TargetType.FILL,
        target=Pump.PUMP_FILL_TARGET_PRESSURE,
        pressure_limit=0.0,
        ramp_time=0.0,
        maximum_time=15.0,
        transition_type=TransitionType.PRESSURE_OVER,
        transition_parameter=3.5,
        name="Fill"
    ),
    BrewStage(
        target_temperature=93.0,
        target_type=TargetType.PRESSURE,
        target=0.0,
        pressure_limit=0.0,
        ramp_time=0.0,
        maximum_time=50.0,
        transition_type=TransitionType.NONE,
        transition_parameter=0.0,
        name="Infuse",
        turbo_heating=False
    ),
    BrewStage(
        target_temperature=93.0,
        target_type=TargetType.FILL,
        target=Pump.PUMP_FULL_ON,
        pressure_limit=0.0,
        ramp_time=0.0,
        maximum_time=30.0,
        transition_type=TransitionType.VOLUME_OVER,
        transition_parameter=40.0,
        name="Flush"
    ),
    BrewStage(
        target_temperature=93.0,
        target_type=TargetType.PRESSURE,
        target=0.0,
        pressure_limit=0.0,
        ramp_time=0.0,
        maximum_time=45.0,
        transition_type=TransitionType.NONE,
        transition_parameter=0.0,
        name="Infuse",
        turbo_heating=False
    ),
    BrewStage(
        target_temperature=93.0,
        target_type=TargetType.FILL,
        target=Pump.PUMP_FULL_ON,
        pressure_limit=0.0,
        ramp_time=0.0,
        maximum_time=30.0,
        transition_type=TransitionType.VOLUME_OVER,
        transition_parameter=40.0,
        name="Flush"
    ),
    BrewStage(
        target_temperature=93.0,
        target_type=TargetType.PRESSURE,
        target=0.0,
        pressure_limit=0.0,
        ramp_time=0.0,
        maximum_time=55.0,
        transition_type=TransitionType.NONE,
        transition_parameter=0.0,
        name="Infuse",
        turbo_heating=False
    ),
    BrewStage(
        target_temperature=93.0,
        target_type=TargetType.FILL,
        target=Pump.PUMP_FULL_ON,
        pressure_limit=0.0,
        ramp_time=0.0,
        maximum_time=30.0,
        transition_type=TransitionType.VOLUME_OVER,
        transition_parameter=40.0,
        name="Flush"
    )
]

oolong_2nd = [
    BrewStage(
        target_temperature=94.0,
        target_type=TargetType.FILL,
        target=Pump.PUMP_FILL_TARGET_PRESSURE,
        pressure_limit=0.0,
        ramp_time=0.0,
        maximum_time=15.0,
        transition_type=TransitionType.PRESSURE_OVER,
        transition_parameter=3.5,
        name="Fill"
    ),
    BrewStage(
        target_temperature=93.0,
        target_type=TargetType.PRESSURE,
        target=0.0,
        pressure_limit=0.0,
        ramp_time=0.0,
        maximum_time=60.0,
        transition_type=TransitionType.NONE,
        transition_parameter=0.0,
        name="Infuse",
        turbo_heating=False
    ),
    BrewStage(
        target_temperature=93.0,
        target_type=TargetType.FILL,
        target=Pump.PUMP_FULL_ON,
        pressure_limit=0.0,
        ramp_time=0.0,
        maximum_time=30.0,
        transition_type=TransitionType.VOLUME_OVER,
        transition_parameter=40.0,
        name="Flush"
    ),
    BrewStage(
        target_temperature=93.0,
        target_type=TargetType.PRESSURE,
        target=0.0,
        pressure_limit=0.0,
        ramp_time=0.0,
        maximum_time=70.0,
        transition_type=TransitionType.NONE,
        transition_parameter=0.0,
        name="Infuse",
        turbo_heating=False
    ),
    BrewStage(
        target_temperature=93.0,
        target_type=TargetType.FILL,
        target=Pump.PUMP_FULL_ON,
        pressure_limit=0.0,
        ramp_time=0.0,
        maximum_time=30.0,
        transition_type=TransitionType.VOLUME_OVER,
        transition_parameter=40.0,
        name="Flush"
    ),
    BrewStage(
        target_temperature=93.0,
        target_type=TargetType.PRESSURE,
        target=0.0,
        pressure_limit=0.0,
        ramp_time=0.0,
        maximum_time=80.0,
        transition_type=TransitionType.NONE,
        transition_parameter=0.0,
        name="Infuse",
        turbo_heating=False
    ),
    BrewStage(
        target_temperature=93.0,
        target_type=TargetType.FILL,
        target=Pump.PUMP_FULL_ON,
        pressure_limit=0.0,
        ramp_time=0.0,
        maximum_time=30.0,
        transition_type=TransitionType.VOLUME_OVER,
        transition_parameter=40.0,
        name="Flush"
    )
]



oolong_gongfu_long = [
    BrewStage(
        target_temperature=94.0,
        target_type=TargetType.FILL,
        target=Pump.PUMP_FILL_TARGET_PRESSURE,
        pressure_limit=0.0,
        ramp_time=0.0,
        maximum_time=15.0,
        transition_type=TransitionType.PRESSURE_OVER,
        transition_parameter=3.5,
        name="Fill"
    ),
    BrewStage(
        target_temperature=93.0,
        target_type=TargetType.PRESSURE,
        target=0.0,
        pressure_limit=0.0,
        ramp_time=0.0,
        maximum_time=50.0,
        transition_type=TransitionType.NONE,
        transition_parameter=0.0,
        name="Infuse",
        turbo_heating=False
    ),
    BrewStage(
        target_temperature=93.0,
        target_type=TargetType.FILL,
        target=Pump.PUMP_FULL_ON,
        pressure_limit=0.0,
        ramp_time=0.0,
        maximum_time=30.0,
        transition_type=TransitionType.VOLUME_OVER,
        transition_parameter=40.0,
        name="Flush"
    ),
    BrewStage(
        target_temperature=93.0,
        target_type=TargetType.PRESSURE,
        target=0.0,
        pressure_limit=0.0,
        ramp_time=0.0,
        maximum_time=45.0,
        transition_type=TransitionType.NONE,
        transition_parameter=0.0,
        name="Infuse",
        turbo_heating=False
    ),
    BrewStage(
        target_temperature=93.0,
        target_type=TargetType.FILL,
        target=Pump.PUMP_FULL_ON,
        pressure_limit=0.0,
        ramp_time=0.0,
        maximum_time=30.0,
        transition_type=TransitionType.VOLUME_OVER,
        transition_parameter=40.0,
        name="Flush"
    ),
    BrewStage(
        target_temperature=93.0,
        target_type=TargetType.PRESSURE,
        target=0.0,
        pressure_limit=0.0,
        ramp_time=0.0,
        maximum_time=55.0,
        transition_type=TransitionType.NONE,
        transition_parameter=0.0,
        name="Infuse",
        turbo_heating=False
    ),
    BrewStage(
        target_temperature=93.0,
        target_type=TargetType.FILL,
        target=Pump.PUMP_FULL_ON,
        pressure_limit=0.0,
        ramp_time=0.0,
        maximum_time=30.0,
        transition_type=TransitionType.VOLUME_OVER,
        transition_parameter=40.0,
        name="Flush"
    ),
    BrewStage(
        target_temperature=93.0,
        target_type=TargetType.PRESSURE,
        target=0.0,
        pressure_limit=0.0,
        ramp_time=0.0,
        maximum_time=60.0,
        transition_type=TransitionType.NONE,
        transition_parameter=0.0,
        name="Infuse",
        turbo_heating=False
    ),
    BrewStage(
        target_temperature=93.0,
        target_type=TargetType.FILL,
        target=Pump.PUMP_FULL_ON,
        pressure_limit=0.0,
        ramp_time=0.0,
        maximum_time=30.0,
        transition_type=TransitionType.VOLUME_OVER,
        transition_parameter=40.0,
        name="Flush"
    ),
    BrewStage(
        target_temperature=93.0,
        target_type=TargetType.PRESSURE,
        target=0.0,
        pressure_limit=0.0,
        ramp_time=0.0,
        maximum_time=70.0,
        transition_type=TransitionType.NONE,
        transition_parameter=0.0,
        name="Infuse",
        turbo_heating=False
    ),
    BrewStage(
        target_temperature=93.0,
        target_type=TargetType.FILL,
        target=Pump.PUMP_FULL_ON,
        pressure_limit=0.0,
        ramp_time=0.0,
        maximum_time=30.0,
        transition_type=TransitionType.VOLUME_OVER,
        transition_parameter=40.0,
        name="Flush"
    ),
    BrewStage(
        target_temperature=93.0,
        target_type=TargetType.PRESSURE,
        target=0.0,
        pressure_limit=0.0,
        ramp_time=0.0,
        maximum_time=80.0,
        transition_type=TransitionType.NONE,
        transition_parameter=0.0,
        name="Infuse",
        turbo_heating=False
    ),
    BrewStage(
        target_temperature=93.0,
        target_type=TargetType.FILL,
        target=Pump.PUMP_FULL_ON,
        pressure_limit=0.0,
        ramp_time=0.0,
        maximum_time=30.0,
        transition_type=TransitionType.VOLUME_OVER,
        transition_parameter=40.0,
        name="Flush"
    ),
    BrewStage(
        target_temperature=93.0,
        target_type=TargetType.PRESSURE,
        target=0.0,
        pressure_limit=0.0,
        ramp_time=0.0,
        maximum_time=80.0,
        transition_type=TransitionType.NONE,
        transition_parameter=0.0,
        name="Infuse",
        turbo_heating=False
    ),
    BrewStage(
        target_temperature=93.0,
        target_type=TargetType.FILL,
        target=Pump.PUMP_FULL_ON,
        pressure_limit=0.0,
        ramp_time=0.0,
        maximum_time=30.0,
        transition_type=TransitionType.VOLUME_OVER,
        transition_parameter=40.0,
        name="Flush"
    ),
    BrewStage(
        target_temperature=93.0,
        target_type=TargetType.PRESSURE,
        target=0.0,
        pressure_limit=0.0,
        ramp_time=0.0,
        maximum_time=80.0,
        transition_type=TransitionType.NONE,
        transition_parameter=0.0,
        name="Infuse",
        turbo_heating=False
    ),
    BrewStage(
        target_temperature=93.0,
        target_type=TargetType.FILL,
        target=Pump.PUMP_FULL_ON,
        pressure_limit=0.0,
        ramp_time=0.0,
        maximum_time=30.0,
        transition_type=TransitionType.VOLUME_OVER,
        transition_parameter=40.0,
        name="Flush"
    ),
    BrewStage(
        target_temperature=93.0,
        target_type=TargetType.PRESSURE,
        target=0.0,
        pressure_limit=0.0,
        ramp_time=0.0,
        maximum_time=80.0,
        transition_type=TransitionType.NONE,
        transition_parameter=0.0,
        name="Infuse",
        turbo_heating=False
    ),
    BrewStage(
        target_temperature=93.0,
        target_type=TargetType.FILL,
        target=Pump.PUMP_FULL_ON,
        pressure_limit=0.0,
        ramp_time=0.0,
        maximum_time=30.0,
        transition_type=TransitionType.VOLUME_OVER,
        transition_parameter=40.0,
        name="Flush"
    ),
    BrewStage(
        target_temperature=93.0,
        target_type=TargetType.PRESSURE,
        target=0.0,
        pressure_limit=0.0,
        ramp_time=0.0,
        maximum_time=80.0,
        transition_type=TransitionType.NONE,
        transition_parameter=0.0,
        name="Infuse",
        turbo_heating=False
    ),
    BrewStage(
        target_temperature=93.0,
        target_type=TargetType.FILL,
        target=Pump.PUMP_FULL_ON,
        pressure_limit=0.0,
        ramp_time=0.0,
        maximum_time=30.0,
        transition_type=TransitionType.VOLUME_OVER,
        transition_parameter=40.0,
        name="Flush"
    ),
    BrewStage(
        target_temperature=93.0,
        target_type=TargetType.PRESSURE,
        target=0.0,
        pressure_limit=0.0,
        ramp_time=0.0,
        maximum_time=80.0,
        transition_type=TransitionType.NONE,
        transition_parameter=0.0,
        name="Infuse",
        turbo_heating=False
    ),
    BrewStage(
        target_temperature=93.0,
        target_type=TargetType.FILL,
        target=Pump.PUMP_FULL_ON,
        pressure_limit=0.0,
        ramp_time=0.0,
        maximum_time=30.0,
        transition_type=TransitionType.VOLUME_OVER,
        transition_parameter=40.0,
        name="Flush"
    )


]


flow_test = [
    BrewStage(
        target_temperature=93.0,
        target_type=TargetType.FLOW,
        target=0.8,
        pressure_limit=6.0,
        ramp_time=0.0,
        maximum_time=60.0,
        transition_type=TransitionType.NONE,
        transition_parameter=0.0,
        name="3.5test"
    )]

brew_profiles = {
    "londinium": londinium,
    "lever": lever,
    "cremina_lever": cremina_lever,
    "LRv3": LRv3,
    "ExtractamundoDos": ExtractamundoDos,
    "flat9": flat9,
    "turbo": turbo,
    "turbobloom": turbobloom,
    "blooming": blooming,
    "oolong_gongfu_4g": oolong_gongfu,
    "oolong_gongfu_4g_2nd": oolong_2nd,
    "oolong_gongfu_long": oolong_gongfu_long,
    "flow_test": flow_test,
    "cleaning": cleaning,
}
