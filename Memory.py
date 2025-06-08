class Memory:
    def __init__(self):
        self.memory = bytearray(0x10000)  # 64KB Memory

    def read_byte(self, address):
        return self.memory[address]

    def write_byte(self, address, value):
        self.memory[address] = value & 0xFF  # Ensure 8-bit

    def load_rom(self, filepath):
        with open(filepath, "rb") as f:
            rom_data = f.read()
            rom_size = min(len(rom_data), 0x8000) # Up to 32 KB
            self.memory[:rom_size] = rom_data[:rom_size]
            