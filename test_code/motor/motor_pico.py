import serial
import time


class motor():
    id = 0
    try:
        uart = serial.Serial('COM5', 115200)
    except:
        print('Failed to open serial port')

    def __init__(self, pin1=0, pin2=0, vref=0):
        self.velocity = 0
        self.id = motor.id
        print(motor.id)
        motor.id = motor.id + 1
        buf = "%d 0 0\n" % (self.id)
        try:
            motor.uart.write(buf.encode('utf-8'))
        except:
            print('Motor controller not coneected')

    def go(self, v):
        self.velocity = v 
        v = v * 360 / 100
        buf = "%d %d %d\n" % (self.id, 0, int(v))
        try:
            motor.uart.write(buf.encode('utf-8'))
        except:
            print('Motor controller not coneected')
        print(buf[:-1])

    def go_deg_per_sec(self, v):
        self.velocity = v * 100 / 360
        buf = "%d %d %d\n" % (self.id, 0, int(v))
        try:
            motor.uart.write(buf.encode('utf-8'))
        except:
            print('Motor controller not coneected')
        print(buf[:-1])

    def go_deg(self, x):
        buf = "%d %d %d\n" % (self.id, 1, x)
        try:
            motor.uart.write(buf.encode('utf-8'))
        except:
            print('Motor controller not coneected')
        print(buf[:-1])

    def back(self, v):
        self.go(-v)

    def stop(self):
        self.go(0)

    def stopslowly(self):
        if not self.velocity == 0:
            # 少しずつDuty比を落として速度を落とす、-10のところは実験によって変えられそう
            for _velocity in range(self.velocity, 0, -10):
                buf = "%d %d %d\n" % (self.id, 0, int(_velocity*360/100))
                try:
                    motor.uart.write(buf.encode('utf-8'))
                except:
                    print('Motor controller not coneected')
                print(buf[:-1])
                time.sleep(0.5)
            self.velocity = 0
        buf = "%d 0 0\n" % (self.id)
        try:
            motor.uart.write(buf.encode('utf-8'))
        except:
            print('Motor controller not coneected')
        print("%d %d %d\n" % (self.id, 0, 0))

# ブレーキ（何であるんだろう？）
    def brake(self):
        self.stop()
