class Registers:
    def __init__(self):
        self.A = 0x01
        self.F = 0xB0
        self.B = 0
        self.C = 0
        self.D = 0
        self.E = 0
        self.H = 0
        self.L = 0
        self.SP = 0xFFFE
        self.PC = 0x0100

    FLAG_Z = 0x80
    FLAG_N = 0x40
    FLAG_H = 0x20
    FLAG_C = 0x10

    def get_AF(self):
        return (self.A << 8) | (self.F & 0xF0)

    def set_AF(self, value):
        self.A = (value >> 8) & 0xFF
        self.F = value & 0xF0

    def get_BC(self):
        return (self.B << 8) | self.C

    def set_BC(self, value):
        self.B = (value >> 8) & 0xFF
        self.C = value & 0xFF

    def get_DE(self):
        return (self.D << 8) | self.E

    def set_DE(self, value):
        self.D = (value >> 8) & 0xFF
        self.E = value & 0xFF

    def get_HL(self):
        return (self.H << 8) | self.L

    def set_HL(self, value):
        self.H = (value >> 8) & 0xFF
        self.L = value & 0xFF

    def set_flag(self, flag, value):
        if value:
            self.F |= flag
        else:
            self.F &= ~flag

    def get_flag(self, flag):
        return (self.F & flag) != 0

    def __repr__(self):
        return f"A: {hex(self.A)} F: {hex(self.F)} BC: {hex(self.get_BC())} DE: {hex(self.get_DE())} HL: {hex(self.get_HL())} SP: {hex(self.SP)} PC: {hex(self.PC)}"