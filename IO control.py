from machine import Pin, PWM
import W5500_EVB_PICO as W5500
import time


class MainHandler:
    def __init__(self):
        self.f_executeTask = False
        self.idx_executeTask = 0
        self.sysBuzzer = Pin(14, Pin.OUT)
        self.sysBuzzer.off()
        self.gpioIn_sel = Pin(15, Pin.OUT)
        self.sysLed_pico = Pin(25, Pin.OUT)
        self.sysLed_board = Pin(28, Pin.OUT)
        self.servo_pwm = PWM(Pin(5), freq=50, duty_u16=int(2**16 * 0.5))      # 2**16 * 0.5 = 65535/2 인데 무슨뜻인지?

# region Init GPIO_OUT
        self.gpioOut_clampClose = Pin(0, Pin.OUT)
        self.gpioOut_clampOpen = Pin(1, Pin.OUT)
        self.gpioOut_rotation0 = Pin(2, Pin.OUT)
        self.gpioOut_rotation90 = Pin(3, Pin.OUT)
        self.gpioOut_vacuum = Pin(4, Pin.OUT)
        self.init_gpioOut()
        # endregion

# region Init GPIO_IN
        self.gpioIn0 = Pin(10, Pin.IN)      # PULL_DOWN 안해도 되는지?
        self.gpioIn1 = Pin(11, Pin.IN)
        self.gpioIn2 = Pin(12, Pin.IN)
        self.gpioIn3 = Pin(13, Pin.IN)

        self.gpioIn_ipSel1 = Pin(22, Pin.IN)            # 8포트 IP 셀렉트를 위해 핀 3개 할당
        self.gpioIn_ipSel2 = Pin(26, Pin.IN)
        self.gpioIn_ipSel4 = Pin(27, Pin.IN)

        self.gpioIn_clampClose = None           # 상태 체크를 위해 변수 지정. 예전에는 딕셔너리로 했는데?
        self.gpioIn_clampOpen = None
        self.gpioIn_rotation0 = None
        self.gpioIn_rotation90 = None
        self.gpioIn_vacumm = None
        self.gpioIn_socketClose = None
        self.gpioIn_socketOpen = None
# endregion

# region Init Ethernet
ipAddress = ''
if self.gpioIn_ipSel1.value() == 0 and self.gpioIn_ipSel2.value() == 0 and self.gpioIn_ipSel4.value() == 0:
            ipAddress = '192.168.0.51'
elif self.gpioIn_ipSel1.value() == 0 and self.gpioIn_ipSel2.value() == 0 and self.gpioIn_ipSel4.value() == 1:
            ipAddress = '192.168.0.52'
elif self.gpioIn_ipSel1.value() == 0 and self.gpioIn_ipSel2.value() == 1 and self.gpioIn_ipSel4.value() == 0:
            ipAddress = '192.168.0.53'
elif self.gpioIn_ipSel1.value() == 0 and self.gpioIn_ipSel2.value() == 1 and self.gpioIn_ipSel4.value() == 1:
            ipAddress = '192.168.0.54'
elif self.gpioIn_ipSel1.value() == 1 and self.gpioIn_ipSel2.value() == 0 and self.gpioIn_ipSel4.value() == 0:
            ipAddress = '192.168.0.55'
elif self.gpioIn_ipSel1.value() == 1 and self.gpioIn_ipSel2.value() == 0 and self.gpioIn_ipSel4.value() == 1:
            ipAddress = '192.168.0.56'
elif self.gpioIn_ipSel1.value() == 1 and self.gpioIn_ipSel2.value() == 1 and self.gpioIn_ipSel4.value() == 0:
            ipAddress = '192.168.0.57'
elif self.gpioIn_ipSel1.value() == 1 and self.gpioIn_ipSel2.value() == 1 and self.gpioIn_ipSel4.value() == 1:
            ipAddress = '192.168.0.58'

#W5500.init(ipAddress=ipAddress)
# endregion

    def init_gpioOut(self):
        self.set_gpioOut(self.gpioOut_clampClose, False)
        self.set_gpioOut(self.gpioOut_clampOpen, False)
        self.set_gpioOut(self.gpioOut_rotation0, True)
        self.set_gpioOut(self.gpioOut_rotation90, False)
        self.set_gpioOut(self.gpioOut_vacumm, False)

    @staticmethod
    def set_gpioOut(target, value):
        target.value(not value)

    def get_gpioIn(self):
        # GPIO_IN0:IN3
        self.gpioIn_sel.on()            # self.gpioIn_sel.value(1) 과 뭐가 다른지??
        time.sleep_us(1)
        self.gpioIn_clampClose = not self.gpioIn0.value()           # not을 사용한 이유 회로 확인
        self.gpioIn_clampOpen = not self.gpioIn1.value()
        self.gpioIn_rotation0 = not self.gpioIn2.value()
        self.gpioIn_rotation90 = not self.gpioIn3.value()

        # GPIO_IN4:IN7 -> IN7: Spare
        self.gpioIn_sel.off()
        time.sleep_us(1)
        self.gpioIn_vacumm = not self.gpioIn0.value()
        self.gpioIn_socketClose = not self.gpioIn1.value()
        self.gpioIn_socketOpen = not self.gpioIn2.value()

    def set_socketCover(self, open_close):
        if open_close == 'open':
            self.servo_pwm.duty_u16(2**16 * 0.1)
        elif open_close == 'close':
            self.servo_pwm.duty_u16(2**16 * 0.5)

    def func_1msec(self):
        message = W5500.checkRxBuffer()
        if message is not None:
            self.parseMessage(message)

    def func_10msec(self):
        self.get_gpioIn()

    def func_100msec(self):
        if self.f_executeTask:
            self.execute_process()

    def func_500msec(self):
        self.sysLed_pico(not self.sysLed_pico.value())
        self.sysLed_board(not self.sysLed_pico.value())

    def parseMessage(self, message):
        if message == 'startTest':
            self.f_executeTask = True
            self.idx_executeTask = 0
        elif message == 'abortTest':
            pass
        elif message == 'emergency':
            pass
        elif message == 'clamp_close':
            pass
        elif message == 'clamp_open':
            pass
        elif message == 'vacuum_on':
            pass
        elif message == 'vacuum_off':
            pass
        elif message == 'rotation_0' or message == 'rotation_90':
            if self.gpioIn_clampClose and self.gpioIn_socketClose:
                if message == 'rotation_0':
                    pass
                else:
                    pass

    def execute_process(self):
        if self.idx_executeTask == 0:
            self.gpioOut_vacuum.value(1)
            self.idx_executeTask += 1
            cntTimeout = 0
        elif self.idx_executeTask == 1:
            pass


if __name__ == "__main__":
    cnt_msec = 0
main = MainHandler()

    while True:
        cnt_msec += 1
        main.func_1msec()

        if not cnt_msec % 10:          # 10으로 나눠서 0이 되는 10, 20, 30 .. 에서 실행
            main.func_10msec()

        if not cnt_msec % 100:
            main.func_100msec()

        if not cnt_msec % 500:
            main.func_500msec()

        time.sleep_ms(1)
