'''This class describes a vehicle paramater that has a fixed, constant value.'''
class ConstantParam():
    def __init__(self, v):
        self.value = v

    def get(self):
        return self.value