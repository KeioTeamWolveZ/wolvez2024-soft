from tkinter import *
from tkinter import ttk
import time
import cv2
from picamera2 import Picamera2
from PIL import Image, ImageTk
import RPi.GPIO as GPIO
from motor_pico import motor as motor

class Tkmain():
    __run = False
    __arm = True

    def __init__(self):
        GPIO.setwarnings(False)
        self.MotorR = motor(dir=-1)
        self.MotorL = motor()

        self.root = Tk()
        self.root.title('CONTROLLER')
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)

        # Frame
        self.frame = ttk.Frame(self.root, padding=10)
        self.frame.grid(sticky=(N, W, S, E))
        self.frame.columnconfigure(0, weight=1)
        self.frame.rowconfigure(0, weight=1)

        # Initialize Picamera2
        self.picam2 = Picamera2()
        size = (1800, 2400)
        config = self.picam2.create_preview_configuration(
            main={"format": 'XRGB8888', "size": size})
        self.picam2.align_configuration(config)
        self.picam2.configure(config)
        self.picam2.start()
        self.picam2.set_controls({"AfMode": 0, "LensPosition": 5.5})

        # Label to display captured image
        self.image_label = ttk.Label(self.frame)
        self.image_label.grid(row=6, column=0, columnspan=5, sticky=(N, S, E, W))

    def scale_and_button(self):

        # スケールの作成
        self.val = DoubleVar()
        self.sc = ttk.Scale(
            self.frame,
            variable=self.val,
            orient=VERTICAL,
            length=200,
            from_=100,
            to=50,
            command=lambda e: print('速度:%4d' % self.val.get()))
        self.sc.grid(row=2, column=0, sticky=(N, E, S, W))
        self.style = ttk.Style()
        self.style.configure("office.TButton", font=20, anchor="s")
        self.val.set(50)

        # textbox
        self.textbox = ttk.Label(
            self.frame,
            text='( ◜ω◝ )',
            width=10)
        self.textbox.grid(row=5, column=2, padx=5, sticky=(E))

        # スケールの作成
        self.val_arm = DoubleVar()
        self.sc_arm = ttk.Scale(
            self.frame,
            variable=self.val_arm,
            orient=VERTICAL,
            length=200,
            from_=1650,
            to=950,
            command=self.hset)
        self.sc_arm.grid(row=2, column=4, sticky=(N, E, S, W))
        self.val_arm.set(1650)

        # Button
        self.faster = ttk.Button(
            self.frame,
            text='はやい',
            width=10,
            style="office.TButton",
            command=lambda: self.vup())
        self.faster.grid(row=0, column=0, padx=5, sticky=(E))

        # Button
        self.slower = ttk.Button(
            self.frame,
            text='おそい',
            width=10,
            style="office.TButton",
            command=lambda: self.vdown())
        self.slower.grid(row=4, column=0, padx=5, sticky=(E))

        # Button (Added by kazu)
        self.upward = ttk.Button(
            self.frame,
            text='上げる',
            width=10,
            style="office.TButton",
            command=lambda: self.hup())
        self.upward.grid(row=0, column=4, padx=5, sticky=(E))

        # Button (Added by kazu)
        self.downward = ttk.Button(
            self.frame,
            text='下げる',
            width=10,
            style="office.TButton",
            command=lambda: self.hdown())
        self.downward.grid(row=4, column=4, padx=5, sticky=(E))

        # Button
        self.go = ttk.Button(
            self.frame,
            text='走る/止まる',
            width=10,
            style="office.TButton",
            command=lambda: self.__go())
        self.go.grid(row=0, column=2, padx=5, sticky=(E))

        # Button
        self.go_a_little = ttk.Button(
            self.frame,
            text='少し走る',
            width=10,
            style="office.TButton",
            command=lambda: self.__go_a_little())
        self.go_a_little.grid(row=1, column=2, padx=5, sticky=(E))

        # Button
        self.back = ttk.Button(
            self.frame,
            text='うしろ',
            width=10,
            style="office.TButton",
            command=lambda: self.__back())
        self.back.grid(row=3, column=2, padx=5, sticky=(E))

        # Button
        self.right = ttk.Button(
            self.frame,
            text='みぎ',
            width=10,
            style="office.TButton",
            command=lambda: self.__right())
        self.right.grid(row=2, column=3, padx=5, sticky=(E))

        # Button
        self.left = ttk.Button(
            self.frame,
            text='ひだり',
            width=10,
            style="office.TButton",
            command=lambda: self.__left())
        self.left.grid(row=2, column=1, padx=5, sticky=(E))

        # Button
        self.grasp = ttk.Button(
            self.frame,
            text='TRY!!',
            width=10,
            style="office.TButton",
            command=lambda: self.__arm_updown())
        self.grasp.grid(row=2, column=2, padx=5, sticky=(E))

        # Button (added by kazu)
        self.picture = ttk.Button(
            self.frame,
            text='写真',
            width=10,
            style="office.TButton",
            command=lambda: self.__picture())
        self.picture.grid(row=1, column=3, padx=5, sticky=(E))

        # Button (added by kazu)
        self.dance = ttk.Button(
            self.frame,
            text='踊る',
            width=10,
            style="office.TButton",
            command=lambda: self.__dance())
        self.dance.grid(row=1, column=1, padx=5, sticky=(E))

        # Button (added by kazu)
        self.finish = ttk.Button(
            self.frame,
            text='終わる',
            width=10,
            style="office.TButton",
            command=lambda: self.__finish())
        self.finish.grid(row=3, column=3, padx=5, sticky=(E))

    def vup(self):
        if self.val.get() < 90:
            self.val.set(self.val.get() + 10)
        else:
            self.val.set(100)
        print('速度:%4d' % self.val.get())
        self.textbox["text"] = '速度:%4d' % self.val.get()

    def vdown(self):
        if self.val.get() > 60:
            self.val.set(self.val.get() - 10)
        else:
            self.val.set(50)
        print('速度:%4d' % self.val.get())
        self.textbox["text"] = '速度:%4d' % self.val.get()

    def hup(self):
        if self.val_arm.get() < 1600:
            self.val_arm.set(self.val_arm.get() + 50)
        else:
            self.val_arm.set(1650)
        print('高さ:%4d' % self.val_arm.get())
        self.textbox["text"] = 'wait a second'
        self.textbox["text"] = '高さ:%4d' % self.val_arm.get()

    def hdown(self):
        if self.val_arm.get() > 1000:
            self.val_arm.set(self.val_arm.get() - 50)
        else:
            self.val_arm.set(950)
        print('高さ:%4d' % self.val_arm.get())
        self.textbox["text"] = 'wait a second'
        self.textbox["text"] = '高さ:%4d' % self.val_arm.get()

    def hset(self, var):
        try:
            value = int(var[:4])
        except ValueError:
            value = int(var[:3])
        self.val_arm.set(value)
        self.textbox["text"] = '高さ:%4d' % self.val_arm.get()

    def __arm_updown(self):
        self.textbox["text"] = '!ATTACK!'
        self.textbox.update()
        self.textbox["text"] = '高さ:%4d' % self.val_arm.get()

    def __go(self):
        if self.__run == False:
            self.textbox["text"] = '走ります'
            self.MotorR.forward(speed=self.val.get())
            self.MotorL.forward(speed=self.val.get())
            self.__run = True
        else:
            self.textbox["text"] = '止まります'
            self.MotorR.stop()
            self.MotorL.stop()
            self.__run = False

    def __go_a_little(self):
        self.textbox["text"] = 'ちょっとだけ走る'
        self.MotorR.forward(speed=self.val.get())
        self.MotorL.forward(speed=self.val.get())
        time.sleep(1)
        self.MotorR.stop()
        self.MotorL.stop()
        self.textbox["text"] = '止まりました'

    def __back(self):
        self.textbox["text"] = 'うしろに下がります'
        self.MotorR.backward(speed=self.val.get())
        self.MotorL.backward(speed=self.val.get())
        time.sleep(1)
        self.MotorR.stop()
        self.MotorL.stop()
        self.textbox["text"] = '止まりました'

    def __right(self):
        self.textbox["text"] = '右を向きます'
        self.MotorL.forward(speed=self.val.get())
        self.MotorR.backward(speed=self.val.get())
        time.sleep(0.5)
        self.MotorR.stop()
        self.MotorL.stop()
        self.textbox["text"] = '止まりました'

    def __left(self):
        self.textbox["text"] = '左を向きます'
        self.MotorR.forward(speed=self.val.get())
        self.MotorL.backward(speed=self.val.get())
        time.sleep(0.5)
        self.MotorR.stop()
        self.MotorL.stop()
        self.textbox["text"] = '止まりました'

    def __dance(self):
        self.textbox["text"] = '踊ります'
        self.MotorR.forward(speed=self.val.get())
        self.MotorL.backward(speed=self.val.get())
        time.sleep(1)
        self.MotorR.backward(speed=self.val.get())
        self.MotorL.forward(speed=self.val.get())
        time.sleep(1)
        self.MotorR.stop()
        self.MotorL.stop()
        self.textbox["text"] = '止まりました'

    def __finish(self):
        self.MotorR.stop()
        self.MotorL.stop()
        self.root.quit()

    def __picture(self):
        # Capture image
        self.frame = self.picam2.capture_array()
        self.frame2 = cv2.rotate(self.frame, cv2.ROTATE_90_CLOCKWISE)
        filename = f'/home/pi/{time.time()}.jpg'
        cv2.imwrite(filename, self.frame2)
        print(f"Image saved as {filename}")

        # Display image
        image = Image.fromarray(cv2.cvtColor(self.frame2, cv2.COLOR_BGR2RGB))
        image.thumbnail((300, 300))  # Resize for display in label
        photo = ImageTk.PhotoImage(image)
        self.image_label.config(image=photo)
        self.image_label.image = photo  # Keep a reference to avoid garbage collection

    def tkstart(self):
        self.scale_and_button()
        self.root.mainloop()

    def tkstop(self):
        self.MotorR.stop()
        self.MotorL.stop()
        GPIO.cleanup()

if __name__ == '__main__':
    tkinter = Tkmain()
    tkinter.tkstart()
