import serial
import time


class motor():
    id = 0
    try:
        uart = serial.Serial('/dev/ttyACM0', 115200)
    except:
        print('Failed to open serial port')

    def send(self, buf):
        try:
            motor.uart.write(buf.encode('utf-8'))
        except:
            print('Motor controller not coneected')

    def __init__(self, pin1=0, pin2=0, vref=0):
        self.velocity = 0
        self.id = motor.id
        print(motor.id)
        motor.id = motor.id + 1
        self.send("%d 0 0\n" % (self.id))
            
    def set_id(self, id):
        self.id = id
        
    def go(self, v):
        self.velocity = v 
        v = v * 360.0 / 100.0
        self.send("%d %d %d\n" % (self.id, 0, int(v)))

    def go_deg_per_sec(self, v):
        self.velocity = v * 100.0 / 360.0
        self.go(self.velocity)
        
    def go_deg(self, x):
        self.send("%d %d %d\n" % (self.id, 1, x))

    def back(self, v):
        self.go(-v)

    def stop(self):
        self.go(0)

    def stopslowly(self):
        if not self.velocity == 0:
            # 少しずつDuty比を落として速度を落とす、-10のところは実験によって変えられそう
            for _velocity in range(self.velocity, 0, -10):
                self.go(_velocity)
                time.sleep(0.5)
            self.velocity = 0
        self.stop()

# ブレーキ（何であるんだろう？）
    def brake(self):
        self.stop()
