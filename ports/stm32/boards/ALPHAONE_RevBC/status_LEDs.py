# this class controls the status LEDs on the alpha:ONE board

class status_LEDs():

    def __init__(
        self,
        LEDs_nr = 3,
        byte_order = 'rbg',
        default_brightness = 1,
        baudrate = 1000000
    ):
        import machine

        self.LEDs_nr = LEDs_nr
        self.byte_order = byte_order
        self.default_brightness = default_brightness

        # colors defined using RGB notation (0 to 255)
        self.colors = {
            'red': [255, 0, 0],
            'green': [0, 255, 0],
            'blue': [0, 0, 255],
            'yellow': [255, 255, 0],
            'orange': [255, 63, 0],
            'turquoise': [0, 63, 255],
            'aqua': [0, 255, 255],
            'pink': [255, 0, 255],
            'white': [255, 255, 255],
            'off': [0, 0, 0]
            }
        
        self._ck_pin = machine.Pin(machine.Pin.cpu.B3, machine.Pin.AF_PP, af=machine.Pin.AF5_SPI1)
        self._do_pin = machine.Pin(machine.Pin.cpu.B5, machine.Pin.AF_PP, af=machine.Pin.AF5_SPI1)
        self._spi = machine.SPI(1, baudrate=baudrate, polarity=0, phase=0, bits=8, firstbit=machine.SPI.MSB)
        
        self._header = bytearray(b'\x00' * 4)
        self._trailer  = bytearray(b'\xff' * 4)
        self._mask = 0b11100000

        self._byte_array = bytearray(4 * LEDs_nr)
        for i in range(0, LEDs_nr):
            self._byte_array[4 * i] |= self._mask
        self._write_buffer()

    
    def _write_buffer(self):
        # print(list(self._byte_array))      # for debugging
        self._spi.write(self._header + self._byte_array + self._trailer)

    
    def set_LED_rgb(self, LED_nr, red, green, blue, brightness=None):
        if (brightness is None) or (brightness < 0) or (brightness > 31):
            brightness = self.default_brightness
        LED_frame_start = 4 * (LED_nr - 1)
        if self.byte_order == 'bgr':
            self._byte_array[LED_frame_start:LED_frame_start + 4] = bytearray([self._mask|brightness, blue, green, red])
        elif self.byte_order == 'rbg':
            self._byte_array[LED_frame_start:LED_frame_start + 4] = bytearray([self._mask|brightness, red, blue, green])
        elif self.byte_order == 'grb':
            self._byte_array[LED_frame_start:LED_frame_start + 4] = bytearray([self._mask|brightness, green, red, blue])
        self._write_buffer()


    def set_LED_color(self, LED_nr, color, brightness=None):
        if (brightness is None) or (brightness < 0) or (brightness > 31):
            brightness = self.default_brightness
        if color in self.colors.keys():
            self.set_LED_rgb(LED_nr, *(self.colors[color] + [brightness]) )
