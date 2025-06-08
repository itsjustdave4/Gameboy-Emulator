from Registers import Registers
from Memory import Memory
from PPU import PPU

class CPU:
    def __init__(self):
        self.reg = Registers()
        self.memory = Memory()
        self.ppu = PPU(self.memory)
        self.IME = False # Interrupt Master Enable (global interrupt flag)
        self.running = True  # CPU Loop

    def fetch_opcode(self):
        opcode = self.memory.read_byte(self.reg.PC)
        self.reg.PC += 1
        if opcode == 0xCB:
            extended = self.memory.read_byte(self.reg.PC)
            self.reg.PC += 1
            return (0xCB << 8) | extended # return a 16 bit value
        return opcode

    def add(self, value):
        result = self.reg.A + value
        self.reg.set_flag(Registers.FLAG_Z, (result & 0xFF) == 0)
        self.reg.set_flag(Registers.FLAG_N, False)
        self.reg.set_flag(Registers.FLAG_H, (self.reg.A & 0xF) + (value & 0xF) > 0xF)
        self.reg.set_flag(Registers.FLAG_C, result > 0xFF)
        self.reg.A = result & 0xFF

    def sub(self, value):
        result = self.reg.A - value
        self.reg.set_flag(Registers.FLAG_Z, (result & 0xFF) == 0)
        self.reg.set_flag(Registers.FLAG_N, True)
        self.reg.set_flag(Registers.FLAG_H, (self.reg.A & 0xF) < (value & 0xF))
        self.reg.set_flag(Registers.FLAG_C, self.reg.A < value)
        self.reg.A = result & 0xFF

    def inc(self, value):
        result = (value + 1) & 0xFF
        self.reg.set_flag(Registers.FLAG_Z, result == 0)
        self.reg.set_flag(Registers.FLAG_N, False)
        self.reg.set_flag(Registers.FLAG_H, (value & 0xF) + 1 > 0xF)
        return result

    def dec(self, value):
        result = (value - 1) & 0xFF
        self.reg.set_flag(Registers.FLAG_Z, result == 0)
        self.reg.set_flag(Registers.FLAG_N, True)
        self.reg.set_flag(Registers.FLAG_H, (value & 0xF) == 0)
        return result

    def and_op(self, value):
        self.reg.A &= value
        self.reg.set_flag(Registers.FLAG_Z, self.reg.A == 0)
        self.reg.set_flag(Registers.FLAG_N, False)
        self.reg.set_flag(Registers.FLAG_H, True)
        self.reg.set_flag(Registers.FLAG_C, False)

    def or_op(self, value):
        self.reg.A |= value
        self.reg.set_flag(Registers.FLAG_Z, self.reg.A == 0)
        self.reg.set_flag(Registers.FLAG_N, False)
        self.reg.set_flag(Registers.FLAG_H, False)
        self.reg.set_flag(Registers.FLAG_C, False)

    def xor_op(self, value):
        self.reg.A ^= value
        self.reg.set_flag(Registers.FLAG_Z, self.reg.A == 0)
        self.reg.set_flag(Registers.FLAG_N, False)
        self.reg.set_flag(Registers.FLAG_H, False)
        self.reg.set_flag(Registers.FLAG_C, False)

    def cp_op(self, value):
        result = self.reg.A - value
        self.reg.set_flag(Registers.FLAG_Z, (result & 0xFF) == 0)
        self.reg.set_flag(Registers.FLAG_N, True)
        self.reg.set_flag(Registers.FLAG_H, (self.reg.A & 0xF) < (value & 0xF))
        self.reg.set_flag(Registers.FLAG_C, self.reg.A < value)

    def get_cb_target(self, index):
        targets = [
            lambda: self.reg.B,
            lambda: self.reg.C,
            lambda: self.reg.D,
            lambda: self.reg.E,
            lambda: self.reg.H,
            lambda: self.reg.L,
            lambda: self.memory.read_byte(self.reg.get_HL()),
            lambda: self.reg.A,
        ]
        
        return targets[index]

    def set_cb_target(self, index, value):
        if index == 0: self.reg.B = value
        elif index == 1: self.reg.C = value
        elif index == 2: self.reg.D = value
        elif index == 3: self.reg.E = value
        elif index == 4: self.reg.H = value
        elif index == 5: self.reg.L = value
        elif index == 6: self.memory.write_byte(self.reg.get_HL(), value)
        elif index == 7: self.reg.A = value

    def handle_interrupts(self):
        if not self.IME:
            return

        IE = self.memory.read_byte(0xFFFF)
        IF = self.memory.read_byte(0xFF0F)
        triggered = IE & IF

        if triggered == 0:
            return

        for i in range(5):
            if triggered & (1 << i):
                self.IME = False  # Disable interrupts
                self.memory.write_byte(0xFF0F, IF & ~(1 << i))  # Clear IF bit

                self.reg.SP -= 2
                self.memory.write_byte(self.reg.SP, self.reg.PC & 0xFF)
                self.memory.write_byte(self.reg.SP + 1, (self.reg.PC >> 8) & 0xFF)

                self.reg.PC = [0x40, 0x48, 0x50, 0x58, 0x60][i]  # Interrupt vector
                break

    def decode_execute(self, opcode):
        
        z = self.reg.get_flag(Registers.FLAG_Z)
        c = self.reg.get_flag(Registers.FLAG_C)

        if opcode > 0xFF:  # CB-prefixed
            cb_opcode = opcode & 0xFF
            x = cb_opcode >> 3  # operation
            y = cb_opcode & 0x07  # target

            val = self.get_cb_target(y)()

            if 0x00 <= cb_opcode <= 0x07:  # RLC r
                carry = (val >> 7) & 1
                result = ((val << 1) | carry) & 0xFF
                self.reg.set_flag(Registers.FLAG_Z, result == 0)
                self.reg.set_flag(Registers.FLAG_N, False)
                self.reg.set_flag(Registers.FLAG_H, False)
                self.reg.set_flag(Registers.FLAG_C, carry)

            elif 0x08 <= cb_opcode <= 0x0F:  # RRC r
                carry = val & 1
                result = ((carry << 7) | (val >> 1)) & 0xFF
                self.reg.set_flag(Registers.FLAG_Z, result == 0)
                self.reg.set_flag(Registers.FLAG_N, False)
                self.reg.set_flag(Registers.FLAG_H, False)
                self.reg.set_flag(Registers.FLAG_C, carry)

            elif 0x10 <= cb_opcode <= 0x17:  # RL r
                carry_in = 1 if self.reg.get_flag(Registers.FLAG_C) else 0
                carry_out = (val >> 7) & 1
                result = ((val << 1) | carry_in) & 0xFF
                self.reg.set_flag(Registers.FLAG_Z, result == 0)
                self.reg.set_flag(Registers.FLAG_N, False)
                self.reg.set_flag(Registers.FLAG_H, False)
                self.reg.set_flag(Registers.FLAG_C, carry_out)

            elif 0x18 <= cb_opcode <= 0x1F:  # RR r
                carry_in = 1 if self.reg.get_flag(Registers.FLAG_C) else 0
                carry_out = val & 1
                result = ((carry_in << 7) | (val >> 1)) & 0xFF
                self.reg.set_flag(Registers.FLAG_Z, result == 0)
                self.reg.set_flag(Registers.FLAG_N, False)
                self.reg.set_flag(Registers.FLAG_H, False)
                self.reg.set_flag(Registers.FLAG_C, carry_out)

            elif 0x20 <= cb_opcode <= 0x27:  # SLA r
                carry = (val >> 7) & 1
                result = (val << 1) & 0xFF
                self.reg.set_flag(Registers.FLAG_Z, result == 0)
                self.reg.set_flag(Registers.FLAG_N, False)
                self.reg.set_flag(Registers.FLAG_H, False)
                self.reg.set_flag(Registers.FLAG_C, carry)

            elif 0x28 <= cb_opcode <= 0x2F:  # SRA r
                carry = val & 1
                msb = val & 0x80
                result = ((val >> 1) | msb) & 0xFF
                self.reg.set_flag(Registers.FLAG_Z, result == 0)
                self.reg.set_flag(Registers.FLAG_N, False)
                self.reg.set_flag(Registers.FLAG_H, False)
                self.reg.set_flag(Registers.FLAG_C, carry)

            elif 0x30 <= cb_opcode <= 0x37:  # SWAP r
                result = ((val & 0x0F) << 4) | ((val & 0xF0) >> 4)
                self.reg.set_flag(Registers.FLAG_Z, result == 0)
                self.reg.set_flag(Registers.FLAG_N, False)
                self.reg.set_flag(Registers.FLAG_H, False)
                self.reg.set_flag(Registers.FLAG_C, False)

            elif 0x38 <= cb_opcode <= 0x3F:  # SRL r
                carry = val & 1
                result = (val >> 1) & 0xFF
                self.reg.set_flag(Registers.FLAG_Z, result == 0)
                self.reg.set_flag(Registers.FLAG_N, False)
                self.reg.set_flag(Registers.FLAG_H, False)
                self.reg.set_flag(Registers.FLAG_C, carry)

            else:
                print(f"Unknown CB-prefixed opcode: {hex(cb_opcode)}")
                return

            self.set_cb_target(y, result)
            return

        if opcode == 0x00: # NOP
            self.reg.PC = self.reg.PC + 1

        elif opcode == 0x01:  # LD BC, nn
            low = self.memory.read_byte(self.reg.PC)
            high = self.memory.read_byte(self.reg.PC + 1)
            self.reg.set_BC((high << 8) | low)
            self.reg.PC += 2

        elif opcode == 0x04: # INC B
            self.reg.B = self.inc(self.reg.B)

        elif opcode == 0x05: # DEC B
            self.reg.B = self.dec(self.reg.B)

        elif opcode == 0x06: # LD B, n
            self.reg.B = self.memory.read_byte(self.reg.PC)
            self.reg.PC += 1

        elif opcode == 0x08:  # LD (nn), SP
            low = self.memory.read_byte(self.reg.PC)
            high = self.memory.read_byte(self.reg.PC + 1)
            addr = (high << 8) | low
            self.memory.write_byte(addr, self.reg.SP & 0xFF)
            self.memory.write_byte(addr + 1, (self.reg.SP >> 8) & 0xFF)
            self.reg.PC += 2

        elif opcode == 0x09: # ADD HL, BC
            self.add_hl(self.reg.get_BC())
            
        elif opcode == 0x0C: # INC C
            self.reg.C = self.inc(self.reg.C)

        elif opcode == 0x0D: # DEC C
            self.reg.C = self.dec(self.reg.C)

        elif opcode == 0x0E: # LD C, n
            self.reg.C = self.memory.read_byte(self.reg.PC)
            self.reg.PC += 1

        elif opcode == 0x11:  # LD DE, nn
            low = self.memory.read_byte(self.reg.PC)
            high = self.memory.read_byte(self.reg.PC + 1)
            self.reg.set_DE((high << 8) | low)
            self.reg.PC += 2

        elif opcode == 0x14: # INC D
            self.reg.D = self.inc(self.reg.D)

        elif opcode == 0x15: # DEC D
            self.reg.D = self.dec(self.reg.D)

        elif opcode == 0x16:  # LD D, n
            self.reg.D = self.memory.read_byte(self.reg.PC)
            self.reg.PC += 1

        elif opcode == 0x18: # JR n
            offset = self.memory.read_byte(self.reg.PC)
            self.reg.PC += 1
            if offset >= 0x80:  # signed 8-bit
                offset -= 0x100
            self.reg.PC = (self.reg.PC + offset) & 0xFFFF

        elif opcode == 0x1C: # INC E
            self.reg.E = self.inc(self.reg.E)

        elif opcode == 0x1D: # DEC E
            self.reg.E = self.dec(self.reg.E)

        elif opcode == 0x1E:  # LD E, n
            self.reg.E = self.memory.read_byte(self.reg.PC)
            self.reg.PC += 1

        elif opcode == 0x20:  # JR NZ
            offset = self.memory.read_byte(self.reg.PC)
            self.reg.PC += 1
            if not z:
                self.reg.PC = (self.reg.PC + (offset - 0x100 if offset >= 0x80 else offset)) & 0xFFFF

        elif opcode == 0x21:  # LD HL, nn
            low = self.memory.read_byte(self.reg.PC)
            high = self.memory.read_byte(self.reg.PC + 1)
            self.reg.set_HL((high << 8) | low)
            self.reg.PC += 2

        elif opcode == 0x24: # INC H
            self.reg.H = self.inc(self.reg.H)
            
        elif opcode == 0x25: # DEC H
            self.reg.H = self.dec(self.reg.H)

        elif opcode == 0x26:  # LD H, n
            self.reg.H = self.memory.read_byte(self.reg.PC)
            self.reg.PC += 1

        elif opcode == 0x28:  # JR Z
            offset = self.memory.read_byte(self.reg.PC)
            self.reg.PC += 1
            if z:
                self.reg.PC = (self.reg.PC + (offset - 0x100 if offset >= 0x80 else offset)) & 0xFFFF

        elif opcode == 0x30:  # JR NC
            offset = self.memory.read_byte(self.reg.PC)
            self.reg.PC += 1
            if not c:
                self.reg.PC = (self.reg.PC + (offset - 0x100 if offset >= 0x80 else offset)) & 0xFFFF

        elif opcode == 0x38:  # JR C
            offset = self.memory.read_byte(self.reg.PC)
            self.reg.PC += 1
            if c:
                self.reg.PC = (self.reg.PC + (offset - 0x100 if offset >= 0x80 else offset)) & 0xFFFF

        elif opcode == 0x2C: # INC L
            self.reg.L = self.inc(self.reg.L)

        elif opcode == 0x2D: # DEC L
            self.reg.L = self.dec(self.reg.L)
        
        elif opcode == 0x2E:  # LD L, n
            self.reg.L = self.memory.read_byte(self.reg.PC)
            self.reg.PC += 1

        elif opcode == 0x31:  # LD SP, nn
            low = self.memory.read_byte(self.reg.PC)
            high = self.memory.read_byte(self.reg.PC + 1)
            self.reg.SP = (high << 8) | low
            self.reg.PC += 2

        elif opcode == 0x34: # INC HL
            addr = self.reg.get_HL()
            self.memory.write_byte(addr, self.inc(self.memory.read_byte(addr)))

        elif opcode == 0x35: # DEC HL
            addr = self.reg.get_HL()
            self.memory.write_byte(addr, self.dec(self.memory.read_byte(addr)))

        elif opcode == 0x47: # LD B, A
            self.reg.B = self.reg.A

        elif opcode == 0x4F: # LD C, A
            self.reg.C = self.reg.A

        elif opcode == 0x57: # LD D, A
            self.reg.D = self.reg.A

        elif opcode == 0x5F: # LD E, A
            self.reg.E = self.reg.A

        elif opcode == 0x67: # LD H, A
            self.reg.H = self.reg.A
        
        elif opcode == 0x6F: # LD L, A
            self.reg.L = self.reg.A

        elif opcode == 0x77: # LD (HL), A
            self.memory.write_byte(self.reg.get_HL(), self.reg.A)

        elif opcode == 0x7E: # LD A, (HL)
            self.reg.A = self.memory.read_byte(self.reg.get_HL())
        
        elif opcode == 0xA0: # AND B
            self.and_op(self.reg.B)

        elif opcode == 0xA1: # AND C
            self.and_op(self.reg.C)

        elif opcode == 0xA2: # AND D
            self.and_op(self.reg.D)

        elif opcode == 0xA3: # AND E
            self.and_op(self.reg.E)

        elif opcode == 0xA4: # AND H
            self.and_op(self.reg.H)

        elif opcode == 0xA5: # AND L
            self.and_op(self.reg.L)

        elif opcode == 0xA6: # AND (HL)
            self.and_op(self.memory.read_byte(self.reg.get_HL()))

        elif opcode == 0xA8: # XOR B
            self.xor_op(self.reg.B)

        elif opcode == 0xA9: # XOR C
            self.xor_op(self.reg.C)

        elif opcode == 0xAA: # XOR D
            self.xor_op(self.reg.D)
        
        elif opcode == 0xAB: # XOR E
            self.xor_op(self.reg.E)

        elif opcode == 0xAC: # XOR H
            self.xor_op(self.reg.H)
        
        elif opcode == 0xAD: # XOR L
            self.xor_op(self.reg.L)
        
        elif opcode == 0xAE: # XOR (HL)
            self.xor_op(self.memory.read_byte(self.reg.get_HL()))
        
        elif opcode == 0xAF: # XOR A
            self.xor_op(self.reg.A)

        elif opcode == 0xB0: # OR B
            self.or_op(self.reg.B)

        elif opcode == 0xB1: # OR C
            self.or_op(self.reg.C)

        elif opcode == 0xB2: # OR D
            self.or_op(self.reg.D)
        
        elif opcode == 0xB3: # OR E
            self.or_op(self.reg.E)
        
        elif opcode == 0xB4: # OR H
            self.or_op(self.reg.H)
        
        elif opcode == 0xB5: # OR L
            self.or_op(self.reg.L)

        elif opcode == 0xB6: # OR (HL)
            self.or_op(self.memory.read_byte(self.reg.get_HL()))

        elif opcode == 0xB8: # CP B
            self.cp_op(self.reg.B)

        elif opcode == 0xB9: # CP C
            self.cp_op(self.reg.C)

        elif opcode == 0xBA: # CP D
            self.cp_op(self.reg.D)

        elif opcode == 0xBB: # CP E
            self.cp_op(self.reg.E)
        
        elif opcode == 0xBC: # CP H
            self.cp_op(self.reg.H)

        elif opcode == 0xBD: # CP L
            self.cp_op(self.reg.L)

        elif opcode == 0xBE: # CP (HL)
            self.cp_op(self.memory.read_byte(self.reg.get_HL()))

        elif opcode == 0xC0:  # RET NZ
            if not z:
                low = self.memory.read_byte(self.reg.SP)
                high = self.memory.read_byte(self.reg.SP + 1)
                self.reg.SP += 2
                self.reg.PC = (high << 8) | low

        elif opcode == 0xC1:  # POP BC
            low = self.memory.read_byte(self.reg.SP)
            high = self.memory.read_byte(self.reg.SP + 1)
            self.reg.set_BC((high << 8) | low)
            self.reg.SP += 2

        elif opcode == 0xC2:  # JP NZ
            addr = self.memory.read_byte(self.reg.PC) | (self.memory.read_byte(self.reg.PC + 1) << 8)
            self.reg.PC = addr if not z else self.reg.PC + 2

        elif opcode == 0xC3: # JP nn
            low = self.memory.read_byte(self.reg.PC)
            high = self.memory.read_byte(self.reg.PC + 1)
            addr = (high << 8) | low
            self.reg.PC = addr

        elif opcode == 0xC4:  # CALL NZ
            addr = self.memory.read_byte(self.reg.PC) | (self.memory.read_byte(self.reg.PC + 1) << 8)
            self.reg.PC += 2
            if not z:
                self.reg.SP -= 2
                self.memory.write_byte(self.reg.SP, self.reg.PC & 0xFF)
                self.memory.write_byte(self.reg.SP + 1, (self.reg.PC >> 8) & 0xFF)
                self.reg.PC = addr

        elif opcode == 0xC5:  # PUSH BC
            self.reg.SP -= 2
            bc = self.reg.get_BC()
            self.memory.write_byte(self.reg.SP, bc & 0xFF)
            self.memory.write_byte(self.reg.SP + 1, (bc >> 8) & 0xFF)

        elif opcode == 0xC8:  # RET Z
            if z:
                low = self.memory.read_byte(self.reg.SP)
                high = self.memory.read_byte(self.reg.SP + 1)
                self.reg.SP += 2
                self.reg.PC = (high << 8) | low

        elif opcode == 0xC9:
            low = self.memory.read_byte(self.reg.SP)
            high = self.memory.read_byte(self.reg.SP + 1)
            self.reg.SP += 2
            self.reg.PC = (high << 8) | low

        elif opcode == 0xCA:  # JP Z
            addr = self.memory.read_byte(self.reg.PC) | (self.memory.read_byte(self.reg.PC + 1) << 8)
            self.reg.PC = addr if z else self.reg.PC + 2

        elif opcode == 0xCC:  # CALL Z, nn
            low = self.memory.read_byte(self.reg.PC)
            high = self.memory.read_byte(self.reg.PC + 1)
            addr = (high << 8) | low
            if z:
                ret_addr = self.reg.PC + 2  # Correct return address BEFORE jump
                self.reg.SP -= 2
                self.memory.write_byte(self.reg.SP, ret_addr & 0xFF)
                self.memory.write_byte(self.reg.SP + 1, (ret_addr >> 8) & 0xFF)
                self.reg.PC = addr
            else:
                self.reg.PC += 2

        elif opcode == 0xCD: # CALL nn
            low = self.memory.read_byte(self.reg.PC)
            high = self.memory.read_byte(self.reg.PC + 1)
            addr = (high << 8) | low
            self.reg.PC += 2
            self.reg.SP -= 2
            self.memory.write_byte(self.reg.SP, self.reg.PC & 0xFF)
            self.memory.write_byte(self.reg.SP + 1, (self.reg.PC >> 8) & 0xFF)
            self.reg.PC = addr

        elif opcode == 0xD0:  # RET NC
            if not c:
                low = self.memory.read_byte(self.reg.SP)
                high = self.memory.read_byte(self.reg.SP + 1)
                self.reg.SP += 2
                self.reg.PC = (high << 8) | low

        elif opcode == 0xD1:  # POP DE
            low = self.memory.read_byte(self.reg.SP)
            high = self.memory.read_byte(self.reg.SP + 1)
            self.reg.set_DE((high << 8) | low)
            self.reg.SP += 2

        elif opcode == 0xD2:  # JP NC
            addr = self.memory.read_byte(self.reg.PC) | (self.memory.read_byte(self.reg.PC + 1) << 8)
            self.reg.PC = addr if not c else self.reg.PC + 2

        elif opcode == 0xD4:  # CALL NC
            addr = self.memory.read_byte(self.reg.PC) | (self.memory.read_byte(self.reg.PC + 1) << 8)
            self.reg.PC += 2
            if not c:
                self.reg.SP -= 2
                self.memory.write_byte(self.reg.SP, self.reg.PC & 0xFF)
                self.memory.write_byte(self.reg.SP + 1, (self.reg.PC >> 8) & 0xFF)
                self.reg.PC = addr

        elif opcode == 0xD5:  # PUSH DE
            self.reg.SP -= 2
            de = self.reg.get_DE()
            self.memory.write_byte(self.reg.SP, de & 0xFF)
            self.memory.write_byte(self.reg.SP + 1, (de >> 8) & 0xFF)

        elif opcode == 0xD8:  # RET C
            if c:
                low = self.memory.read_byte(self.reg.SP)
                high = self.memory.read_byte(self.reg.SP + 1)
                self.reg.SP += 2
                self.reg.PC = (high << 8) | low

        elif opcode == 0xDA:  # JP C
            addr = self.memory.read_byte(self.reg.PC) | (self.memory.read_byte(self.reg.PC + 1) << 8)
            self.reg.PC = addr if c else self.reg.PC + 2

        elif opcode == 0xDC:  # CALL C
            addr = self.memory.read_byte(self.reg.PC) | (self.memory.read_byte(self.reg.PC + 1) << 8)
            self.reg.PC += 2
            if c:
                self.reg.SP -= 2
                self.memory.write_byte(self.reg.SP, self.reg.PC & 0xFF)
                self.memory.write_byte(self.reg.SP + 1, (self.reg.PC >> 8) & 0xFF)
                self.reg.PC = addr

        elif opcode == 0xE1:  # POP HL
            low = self.memory.read_byte(self.reg.SP)
            high = self.memory.read_byte(self.reg.SP + 1)
            self.reg.set_HL((high << 8) | low)
            self.reg.SP += 2

        elif opcode == 0xE5:  # PUSH HL
            self.reg.SP -= 2
            hl = self.reg.get_HL()
            self.memory.write_byte(self.reg.SP, hl & 0xFF)
            self.memory.write_byte(self.reg.SP + 1, (hl >> 8) & 0xFF)

        elif opcode == 0xE6:  # AND n
            value = self.memory.read_byte(self.reg.PC)
            self.reg.PC += 1
            self.and_op(value)

        elif opcode == 0xEE:  # XOR n
            value = self.memory.read_byte(self.reg.PC)
            self.reg.PC += 1
            self.xor_op(value)

        elif opcode == 0xF1:  # POP AF
            low = self.memory.read_byte(self.reg.SP)
            high = self.memory.read_byte(self.reg.SP + 1)
            self.reg.set_AF((high << 8) | low)
            self.reg.SP += 2

        elif opcode == 0xF3: # DI
            self.IME = False

        elif opcode == 0xF5:  # PUSH AF
            self.reg.SP -= 2
            af = self.reg.get_AF()
            self.memory.write_byte(self.reg.SP, af & 0xFF)
            self.memory.write_byte(self.reg.SP + 1, (af >> 8) & 0xFF)

        elif opcode == 0xF6:  # OR n
            value = self.memory.read_byte(self.reg.PC)
            self.reg.PC += 1
            self.or_op(value)

        elif opcode == 0xF9:  # LD SP, HL
            self.reg.SP = self.reg.get_HL()

        elif opcode == 0xFB: # EI
            self.IME = True

        elif opcode == 0xFE:  # CP n
            value = self.memory.read_byte(self.reg.PC)
            self.reg.PC += 1
            self.cp_op(value)

        else:
            print(f"Unknown opcode: {hex(opcode)} at PC: {hex(self.reg.PC - 1)}")

    def run(self, steps=None):
        executed = 0
        cycles_per_instruction = 4  # We'll generalize later
        while self.running and (steps is None or executed < steps):
            self.handle_interrupts()
            opcode = self.fetch_opcode()
            self.decode_execute(opcode)
            self.ppu.step(cycles_per_instruction)  # simulate PPU timing
            print(self.reg)
            executed += 1

cpu = CPU()
cpu.memory.load_rom("cpu_instrs.gb")

cpu.memory.write_byte(0xFF42, 32)  # SCY = scroll down
cpu.memory.write_byte(0xFF43, 16)  # SCX = scroll right
for row in cpu.ppu.framebuffer[:10]:
    print("".join(str(p) for p in row[:40]))

    
