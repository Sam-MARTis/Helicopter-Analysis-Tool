from typing import List, Callable



class Rotor:
    def __init__(self, twist_function: Callable[[float], float], chord_function: Callable[[float], float], length: float, blade_count: int = 3):
        self.twist_function = twist_function
        self.chord_function = chord_function
        self.length = length
        self.blade_count = blade_count

class Skids:
    def __init__(self, height: float):
        self.height = height

class Helicopter:
    def __init__(self, id:int, startingState: State):
        self.id = id
        
        # self.onGround = onGround