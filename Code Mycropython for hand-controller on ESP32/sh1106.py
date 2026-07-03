from micropython import const
import framebuf

# địa chỉ
SET_DISP = const(0xAE)
SET_DISP_START_LINE = const(0x40)
SET_SEG_REMAP = const(0xA0)
SET_MUX_RATIO = const(0xA8)
SET_COM_OUT_DIR = const(0xC0)
SET_DISP_OFFSET = const(0xD3)
SET_COM_PIN_CFG = const(0xDA)
SET_DISP_CLK_DIV = const(0xD5)
SET_PRECHARGE = const(0xD9)
SET_VCOM_DESEL = const(0xDB)
SET_CONTRAST = const(0x81)
SET_ENTIRE_ON = const(0xA4)
SET_NORM_INV = const(0xA6)
SET_CHARGE_PUMP = const(0x8D)


SET_LOW_COLUMN = const(0x00)
SET_HIGH_COLUMN = const(0x10)
SET_PAGE_ADDRESS = const(0xB0)



class SH1106(framebuf.FrameBuffer):
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.pages = height // 8
        self.buffer = bytearray(self.pages * width)

        super().__init__(self.buffer, width, height, framebuf.MONO_VLSB)

        self.init_display()

    def init_display(self):
        for cmd in (
            SET_DISP | 0x00,
            SET_DISP_START_LINE | 0x00,
            SET_SEG_REMAP | 0x01,
            SET_MUX_RATIO, self.height - 1,
            SET_COM_OUT_DIR | 0x08,
            SET_DISP_OFFSET, 0x00,
            SET_COM_PIN_CFG, 0x12,
            SET_DISP_CLK_DIV, 0x80,
            SET_PRECHARGE, 0xF1,
            SET_VCOM_DESEL, 0x30,
            SET_CONTRAST, 0xFF,
            SET_ENTIRE_ON,
            SET_NORM_INV,
            SET_CHARGE_PUMP, 0x14,
            SET_DISP | 0x01,
        ):
            self.write_cmd(cmd)

        self.fill(0)
        self.show()

    def show(self):
        for page in range(self.pages):
            self.write_cmd(SET_PAGE_ADDRESS | page)

            self.write_cmd(SET_LOW_COLUMN | 2)
            self.write_cmd(SET_HIGH_COLUMN | 0)

            start = page * self.width
            end = start + self.width
            self.write_data(self.buffer[start:end])


# =========================================================
# khởi I2C
# =========================================================
class SH1106_I2C(SH1106):
    def __init__(self, width, height, i2c, addr=0x3C):
        self.i2c = i2c
        self.addr = addr

        self.temp = bytearray(2)
        self.write_list = [b"\x40", None]

        super().__init__(width, height)

    def write_cmd(self, cmd):
        self.temp[0] = 0x80
        self.temp[1] = cmd
        self.i2c.writeto(self.addr, self.temp)

    def write_data(self, buf):
        self.write_list[1] = buf
        self.i2c.writevto(self.addr, self.write_list)