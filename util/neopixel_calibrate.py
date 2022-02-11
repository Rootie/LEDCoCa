import time
import board
import neopixel


class NeoPixelCalibration:
    neopixel_length = 100
    interval = .5
    
    def calibration_timing_set_color(self, color):
        for idx in range(20):
            self.pixels[idx] = (color)
            self.pixels[self.neopixel_length - idx - 1] = (color)
        self.pixels.show()
        time.sleep(self.interval)

    def run(self):
        self.pixels = neopixel.NeoPixel(board.D18, self.neopixel_length, pixel_order = 'RGB', auto_write = False)
        self.calibration_timing_set_color((255,0,0))
        self.calibration_timing_set_color((0,255,0))
        self.calibration_timing_set_color((0,0,255))
        self.calibration_timing_set_color((0,0,0))

        for idx in range(self.neopixel_length):
            self.pixels[idx] = ((255, 255, 255))
            self.pixels.show()

            time.sleep(self.interval)
            self.pixels[idx] = ((0, 0, 0))
        
        self.calibration_timing_set_color((255,0,0))
        self.calibration_timing_set_color((0,255,0))
        self.calibration_timing_set_color((0,0,255))
        self.calibration_timing_set_color((0,0,0))

c = NeoPixelCalibration()
while True:
    c.run()
    time.sleep(2)
