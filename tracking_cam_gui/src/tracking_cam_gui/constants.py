

MANUAL = 0
AUTO = 1

CMD = {"MIN_YAW": 0,
       "PLUS_YAW": 1,
       "MIN_PITCH": 2,
       "PLUS_PITCH": 3,
       "MIN_ROLL": 4,
       "PLUS_ROLL": 5,
       "HOME": 6,
       "ENABLE": 7,
       "DISABLE": 8}

XBOX = {"A": 0,
        "B": 1,
        "X": 2,
        "Y": 3,
        "LB": 4,
        "RB": 5,
        "HOME": 7}

MAX_SPEED = 1

YAW_HOME = 3072
PITCH_HOME = 650
ROLL_HOME = 3072
RANGE = 2048

YAW = [YAW_HOME-(RANGE/2), YAW_HOME+(RANGE/2)]
PITCH = [PITCH_HOME-(RANGE/2), PITCH_HOME+(RANGE/2)]
ROLL = [ROLL_HOME-(RANGE/2), ROLL_HOME+(RANGE/2)]

STEP = RANGE//180
