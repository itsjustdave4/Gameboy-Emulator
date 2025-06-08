class PPU:
    def __init__(self, memory):
        self.memory = memory
        self.scanline_counter = 456  # cycles per scanline
        self.framebuffer = [[0 for _ in range(160)] for _ in range(144)] # 160x144

    def step(self, cycles):
        self.scanline_counter -= cycles
        if self.scanline_counter <= 0:
            self.scanline_counter += 456
            ly = self.memory.read_byte(0xFF44)
            ly = (ly + 1) % 154  # 0–153 scanlines
            self.memory.write_byte(0xFF44, ly)

            if ly == 144:  # Start of VBlank
                self.memory.write_byte(0xFF0F, self.memory.read_byte(0xFF0F) | 0x01)  # VBlank IF bit
            elif ly == 0:
                self.render_background()  # <--- Trigger tilemap redraw once per frame
    
    def read_tile(self, tile_index, tile_data_base=0x8000, signed=False):
        pixels = [[0] * 8 for _ in range(8)]
        offset = tile_index * 16
        if signed and tile_index > 127:
            tile_index -= 256  # wrap as signed
            offset = (tile_index & 0xFF) * 16

        for row in range(8):
            byte1 = self.memory.read_byte(tile_data_base + offset + row * 2)
            byte2 = self.memory.read_byte(tile_data_base + offset + row * 2 + 1)
            for col in range(8):
                bit = 7 - col
                low = (byte1 >> bit) & 1
                high = (byte2 >> bit) & 1
                pixels[row][col] = (high << 1) | low
        return pixels

    def render_background(self):
        lcdc = self.memory.read_byte(0xFF40)
        tilemap_base = 0x9C00 if (lcdc & 0x08) else 0x9800
        tiledata_base = 0x8800 if (lcdc & 0x10) == 0 else 0x8000
        signed = (lcdc & 0x10) == 0

        scy = self.memory.read_byte(0xFF42)
        scx = self.memory.read_byte(0xFF43)

        for y in range(144):  # screen height
            tile_y = ((y + scy) // 8) % 32
            pixel_y = (y + scy) % 8

            for x in range(160):  # screen width
                tile_x = ((x + scx) // 8) % 32
                pixel_x = (x + scx) % 8

                tile_index = self.memory.read_byte(tilemap_base + tile_y * 32 + tile_x)
                tile = self.read_tile(tile_index, tiledata_base, signed)
                color = tile[pixel_y][pixel_x]

                self.framebuffer[y][x] = color

        for ty in range(18):  # 32 tiles → 18 = 144 / 8
            for tx in range(20):  # 160 / 8
                tile_index = self.memory.read_byte(tilemap_base + ty * 32 + tx)
                tile = self.read_tile(tile_index, tiledata_base, signed)
                for y in range(8):
                    for x in range(8):
                        screen_x = tx * 8 + x
                        screen_y = ty * 8 + y
                        if screen_x < 160 and screen_y < 144:
                            self.framebuffer[screen_y][screen_x] = tile[y][x]

