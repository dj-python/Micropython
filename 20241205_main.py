from machine import Pin, PWM
import W5500_EVB_PICO as W5500
import time

from W5500_EVB_PICO import sendMessage

APP_VER = 1.0

TIMEOUT_INIT_SOCKET_POS = 600  # 25ms * 600 = 12.5s
TIMEOUT_UNIT_OP = 30  # 100ms * 30 = 3.0s
TIMEOUT_LOAD_UNLOAD = 30  # 100ms * 30 = 3.0s


class SocketStatus:
    UNKNOWN = 1
    READY = 2
    DOING = 3
    ERROR = 4


class SocketError:
    NONE = 'ERROR_NONE'
    INIT_SOCKET_POS = 'ERROR_INIT_SOCKET_POS'
    VACUUM_ON = 'ERROR_VACUUM_ON'
    VACUUM_OFF = 'ERROR_VACUUM_OFF'
    CLAMP_CLOSE = 'ERROR_CLAMP_CLOSE'
    CLAMP_OPEN = 'ERROR_CLAMP_OPEN'
    SOCKET_CLOSE = 'ERROR_SOCKET_CLOSE'
    SOCKET_OPEN = 'ERROR_SOCKET_OPEN'
    ROTATION_0D = 'ERROR_ROTATION_0D'
    ROTATION_90D = 'ERROR_ROTATION_90D'
    LOAD_UNLOAD = 'ERROR_UNLOAD'


class MainHandler:
    cTemp = 0

    def __init__(self):
        self.sysBuzzer = Pin(14, Pin.OUT)
        self.gpioIn_sel = Pin(15, Pin.OUT)
        self.sysLed_pico = Pin(25, Pin.OUT)
        self.sysLed_board = Pin(28, Pin.OUT)

        # region Init GPIO_OUT
        self.gpioOut_clampClose = Pin(0, Pin.OUT)
        self.gpioOut_clampOpen = Pin(1, Pin.OUT)
        self.gpioOut_rotation0 = Pin(2, Pin.OUT)
        self.gpioOut_rotation90 = Pin(3, Pin.OUT)
        self.gpioOut_vacumm = Pin(4, Pin.OUT)
        self.gpioOut_spare1 = Pin(6, Pin.OUT)
        self.gpioOut_spare2 = Pin(7, Pin.OUT)
        self.init_gpioOut()
        # endregion

        # region Init GPIO_IN
        self.gpioIn0 = Pin(10, Pin.IN)
        self.gpioIn1 = Pin(11, Pin.IN)
        self.gpioIn2 = Pin(12, Pin.IN)
        self.gpioIn3 = Pin(13, Pin.IN)

        self.gpioIn_ipSel1 = Pin(22, Pin.IN)
        self.gpioIn_ipSel2 = Pin(26, Pin.IN)
        self.gpioIn_ipSel4 = Pin(27, Pin.IN)

        self.gpioIn_clampClose = None
        self.gpioIn_clampOpen = None
        self.gpioIn_rotation0 = None
        self.gpioIn_rotation90 = None
        self.gpioIn_vacumm = None
        self.gpioIn_socketClose = None
        self.gpioIn_socketOpen = None
        # endregion

        # region Init Ethernet
        ipAddress = ''
        portNumber = 0
        if self.gpioIn_ipSel1.value() == 0 and self.gpioIn_ipSel2.value() == 0 and self.gpioIn_ipSel4.value() == 0:
            ipAddress = '166.79.25.110'
            portNumber = 6571
        elif self.gpioIn_ipSel1.value() == 0 and self.gpioIn_ipSel2.value() == 0 and self.gpioIn_ipSel4.value() == 1:
            ipAddress = '166.79.25.111'
            portNumber = 6572
        elif self.gpioIn_ipSel1.value() == 0 and self.gpioIn_ipSel2.value() == 1 and self.gpioIn_ipSel4.value() == 0:
            ipAddress = '166.79.25.112'
            portNumber = 6573
        elif self.gpioIn_ipSel1.value() == 0 and self.gpioIn_ipSel2.value() == 1 and self.gpioIn_ipSel4.value() == 1:
            ipAddress = '166.79.25.113'
            portNumber = 6574
        elif self.gpioIn_ipSel1.value() == 1 and self.gpioIn_ipSel2.value() == 0 and self.gpioIn_ipSel4.value() == 0:
            ipAddress = '166.79.25.114'
            portNumber = 6575
        elif self.gpioIn_ipSel1.value() == 1 and self.gpioIn_ipSel2.value() == 0 and self.gpioIn_ipSel4.value() == 1:
            ipAddress = '166.79.25.115'
            portNumber = 6576
        elif self.gpioIn_ipSel1.value() == 1 and self.gpioIn_ipSel2.value() == 1 and self.gpioIn_ipSel4.value() == 0:
            ipAddress = '166.79.25.116'
            portNumber = 6577
        elif self.gpioIn_ipSel1.value() == 1 and self.gpioIn_ipSel2.value() == 1 and self.gpioIn_ipSel4.value() == 1:
            ipAddress = '166.79.25.117'
            portNumber = 6578

        W5500.init(ipAddress=ipAddress, portNumber=portNumber)
        self.UDP_CLIENT = ('166.79.25.100', portNumber)
        # endregion

        self.rxMessage = str()

        self.servo_pwm = PWM(Pin(5), freq=50, duty_u16=0)

        self.cntExecProcess = 0
        self.cntTimeOutExecProcess = 0

        self.isExecProcess_unitOp= False
        self.idxExecProcess_unitOp = 0
        self.isExecProcess_loadUnload = False
        self.idxExecProcess_loadUnload = 0

        self.isInitedSocket = False
        self.isExecProcess_initSocketPos = False
        self.idxExecProcess_initSocketPos = 0
        self.socketCover_pwmDuty_close = 0
        self.socketCover_pwmDuty_open = 0
        self.socketStatus = SocketStatus.UNKNOWN
        self.socketError = SocketError.NONE

    def init_gpioOut(self):
        self.set_gpioOut(self.sysBuzzer, False)

        self.set_gpioOut(self.gpioOut_clampClose, False)
        self.set_gpioOut(self.gpioOut_clampOpen, True)
        self.set_gpioOut(self.gpioOut_rotation0, True)
        self.set_gpioOut(self.gpioOut_rotation90, False)
        self.set_gpioOut(self.gpioOut_vacumm, False)
        self.set_gpioOut(self.gpioOut_spare1, False)
        self.set_gpioOut(self.gpioOut_spare2, False)

    @staticmethod
    def set_gpioOut(target, value):
        target.value(not value)

    def get_gpioIn(self):
        # GPIO_IN0:IN3
        self.gpioIn_sel.on()
        time.sleep_us(1)
        self.gpioIn_clampClose = not self.gpioIn0.value()
        self.gpioIn_clampOpen = not self.gpioIn1.value()
        self.gpioIn_rotation0 = not self.gpioIn2.value()
        self.gpioIn_rotation90 = not self.gpioIn3.value()

        # GPIO_IN4:IN7 -> IN7: Spare
        self.gpioIn_sel.off()
        time.sleep_us(1)
        self.gpioIn_vacumm = not self.gpioIn0.value()
        self.gpioIn_socketClose = not self.gpioIn1.value()
        self.gpioIn_socketOpen = not self.gpioIn2.value()

    def func_1msec(self):
        pass

    def func_10msec(self):
        self.get_gpioIn()

        # if not self.isExecProcess_initSocketPos:
        message, address = W5500.readMessage()
        if message is not None:
            self.rxMessage = message.decode('utf-8')
            print(address, self.rxMessage, self.rxMessage[1:3], self.rxMessage[3:5], self.socketStatus)

            # Command1: rxMessage[1:3], Command2: rxMessage[3:5]
            # Init socket
            if self.rxMessage[1:3] == '20':
                self.cntTimeOutExecProcess = 0
                self.idxExecProcess_initSocketPos = 0
                self.socketStatus = SocketStatus.DOING
                self.isExecProcess_initSocketPos = True
            # Reset error
            elif self.rxMessage[1:3] == '14':
                if self.isInitedSocket:
                    self.socketStatus = SocketStatus.READY
                    self.socketError = SocketError.NONE
                    self.replyMessage('S' + self.rxMessage[1:5] + '000')
                else:
                    self.replyMessage('S' + self.rxMessage[1:5] + '001')
            else:
                if self.socketStatus is SocketStatus.READY:
                    # load/unload
                    if self.rxMessage[1:3] == '21':
                        self.idxExecProcess_loadUnload = 0
                        self.isExecProcess_loadUnload = True
                    # unit operation
                    else:
                        self.idxExecProcess_unitOp = 0
                        self.isExecProcess_unitOp = True

                    self.cntTimeOutExecProcess = 0
                    self.socketStatus = SocketStatus.DOING
                else:
                    if self.socketStatus is SocketStatus.UNKNOWN:
                        pass
                    elif self.socketStatus is SocketStatus.DOING:
                        pass
                    elif self.socketStatus is SocketStatus.ERROR:
                        pass
                        # self.cntTimeOutExecProcess = 0
                        # self.idxExecProcess_unitOp = 0
                        # self.socketStatus = SocketStatus.DOING
                        # self.isExecProcess_unitOp = True

    def func_25msec(self):
        if self.isExecProcess_initSocketPos:
            self.execProcess_setSocketPos()

    def func_50msec(self):
        pass

    def func_100msec(self):
        if self.isExecProcess_loadUnload:
            self.execProcess_loadUnload()
        elif self.isExecProcess_unitOp:
            self.execProcess_unitOp()

    def func_500msec(self):
        self.sysLed_pico(not self.sysLed_pico.value())
        self.sysLed_board(not self.sysLed_pico.value())

    def execProcess_setSocketPos(self):
        if self.idxExecProcess_initSocketPos == 0:
            self.set_gpioOut(self.gpioOut_rotation0, True)
            self.set_gpioOut(self.gpioOut_rotation90, False)
            self.idxExecProcess_initSocketPos += 1
        elif self.idxExecProcess_initSocketPos == 1:
            self.socketError = SocketError.ROTATION_0D
            if self.gpioIn_rotation0:
                self.cntExecProcess = 0
                self.idxExecProcess_initSocketPos += 1
        elif self.idxExecProcess_initSocketPos == 2:
            self.cntExecProcess += 1
            if self.cntExecProcess >= 5:
                self.set_gpioOut(self.gpioOut_clampClose, False)
                self.set_gpioOut(self.gpioOut_clampOpen, True)
                self.set_gpioOut(self.gpioOut_vacumm, False)
                self.idxExecProcess_initSocketPos += 1
        elif self.idxExecProcess_initSocketPos == 3:
            self.socketError = SocketError.CLAMP_OPEN
            if self.gpioIn_clampOpen:
                self.cntExecProcess = 0
                self.idxExecProcess_initSocketPos += 1
        elif self.idxExecProcess_initSocketPos == 4:
            self.cntExecProcess += 1
            if self.cntExecProcess >= 5:
                self.idxExecProcess_initSocketPos += 1
        elif self.idxExecProcess_initSocketPos == 5:
            self.servo_pwm = PWM(Pin(5), freq=50, duty_u16=4900)
            # self.servo_pwm = PWM(Pin(5), freq=50, duty_u16=int(2 ** 16 * 0.5))
            self.cntExecProcess = 0
            self.idxExecProcess_initSocketPos += 1
        elif self.idxExecProcess_initSocketPos == 6:
            self.cntExecProcess += 1
            if self.cntExecProcess >= 20:
                self.idxExecProcess_initSocketPos += 1
        elif self.idxExecProcess_initSocketPos == 7:
            self.servo_pwm.duty_u16(self.servo_pwm.duty_u16() - 50)
            self.idxExecProcess_initSocketPos += 1
        elif self.idxExecProcess_initSocketPos == 8:
            self.socketError = SocketError.SOCKET_CLOSE
            if self.gpioIn_socketClose:
                self.servo_pwm.duty_u16(self.servo_pwm.duty_u16() + 100)
                time.sleep(0.3)
                self.idxExecProcess_initSocketPos += 1
            else:
                self.idxExecProcess_initSocketPos -= 1
        elif self.idxExecProcess_initSocketPos == 9:
            self.servo_pwm.duty_u16(self.servo_pwm.duty_u16() - 10)
            self.idxExecProcess_initSocketPos += 1
        elif self.idxExecProcess_initSocketPos == 10:
            if self.gpioIn_socketClose:
                self.socketCover_pwmDuty_close = self.servo_pwm.duty_u16() - 20
                self.servo_pwm.duty_u16(self.socketCover_pwmDuty_close)
                self.idxExecProcess_initSocketPos += 1
            else:
                self.idxExecProcess_initSocketPos -= 1
        elif self.idxExecProcess_initSocketPos == 11:
            self.servo_pwm.duty_u16(4900)
            # self.servo_pwm = PWM(Pin(5), freq=50, duty_u16=int(2 ** 16 * 0.5))
            self.cntExecProcess = 0
            self.idxExecProcess_initSocketPos += 1
        elif self.idxExecProcess_initSocketPos == 12:
            self.cntExecProcess += 1
            if self.cntExecProcess >= 20:
                self.idxExecProcess_initSocketPos += 1
        elif self.idxExecProcess_initSocketPos == 13:
            self.servo_pwm.duty_u16(self.servo_pwm.duty_u16() + 50)
            self.idxExecProcess_initSocketPos += 1
        elif self.idxExecProcess_initSocketPos == 14:
            self.socketError = SocketError.SOCKET_OPEN
            if self.gpioIn_socketOpen:
                self.servo_pwm.duty_u16(self.servo_pwm.duty_u16() - 100)
                time.sleep(0.3)
                self.idxExecProcess_initSocketPos += 1
            else:
                self.idxExecProcess_initSocketPos -= 1
        elif self.idxExecProcess_initSocketPos == 15:
            self.servo_pwm.duty_u16(self.servo_pwm.duty_u16() + 10)
            self.idxExecProcess_initSocketPos += 1
        elif self.idxExecProcess_initSocketPos == 16:
            if self.gpioIn_socketOpen:
                self.socketCover_pwmDuty_open = self.servo_pwm.duty_u16() + 230
                time.sleep(0.3)
                self.servo_pwm.duty_u16(self.socketCover_pwmDuty_open)
                self.isExecProcess_initSocketPos = False
                self.socketStatus = SocketStatus.READY
                self.socketError = SocketError.NONE
                self.isInitedSocket = True
                self.replyMessage('S' + self.rxMessage[1:5] + '000')
            else:
                self.idxExecProcess_initSocketPos -= 1

        self.cntTimeOutExecProcess += 1
        if self.cntTimeOutExecProcess >= TIMEOUT_INIT_SOCKET_POS:
            errorCode = self.checkErrorCode()

            self.isExecProcess_initSocketPos = False
            self.socketStatus = SocketStatus.ERROR
            self.socketError = SocketError.INIT_SOCKET_POS
            self.isInitedSocket = False
            self.replyMessage('S' + self.rxMessage[1:5] + errorCode)

    def execProcess_unitOp(self):
        if self.idxExecProcess_unitOp == 0:
            # vacuum
            if self.rxMessage[1:3] == '10':
                # off
                if self.rxMessage[3:5] == '00':
                    self.set_gpioOut(self.gpioOut_vacumm, False)
                else:
                    self.set_gpioOut(self.gpioOut_vacumm, True)
            # socket close/open
            elif self.rxMessage[1:3] == '11':
                if self.gpioIn_rotation0:
                    # close
                    if self.rxMessage[3:5] == '00':
                        self.servo_pwm.duty_u16(self.socketCover_pwmDuty_close)
                    else:
                        self.servo_pwm.duty_u16(self.socketCover_pwmDuty_open)
            # clamp
            elif self.rxMessage[1:3] == '12':
                if self.gpioIn_rotation0:
                    # close
                    if self.rxMessage[3:5] == '00':
                        self.set_gpioOut(self.gpioOut_clampClose, True)
                        self.set_gpioOut(self.gpioOut_clampOpen, False)
                    # open
                    else:
                        self.set_gpioOut(self.gpioOut_clampClose, False)
                        self.set_gpioOut(self.gpioOut_clampOpen, True)
            # rotation 0d/90d
            elif self.rxMessage[1:3] == '13':
                if self.gpioIn_socketClose and self.gpioIn_clampClose:
                    # 0d
                    if self.rxMessage[3:5] == '00':
                        self.set_gpioOut(self.gpioOut_rotation0, True)
                        self.set_gpioOut(self.gpioOut_rotation90, False)
                    else:
                        self.set_gpioOut(self.gpioOut_rotation0, False)
                        self.set_gpioOut(self.gpioOut_rotation90, True)

            self.cntExecProcess = 0
            self.idxExecProcess_unitOp += 1
        elif self.idxExecProcess_unitOp == 1:
            UNIT_OP_SUCCESS = False

            # vacuum
            if self.rxMessage[1:3] == '10':
                if ((self.rxMessage[3:5] == '00' and not self.gpioIn_vacumm) or
                        (self.rxMessage[3:5] == '01' and self.gpioIn_vacumm)):
                    UNIT_OP_SUCCESS = True
            # socket close/open
            elif self.rxMessage[1:3] == '11':
                if ((self.rxMessage[3:5] == '00' and self.gpioIn_socketClose) or
                        (self.rxMessage[3:5] == '01' and self.gpioIn_socketOpen)):
                    UNIT_OP_SUCCESS = True
            # clamp
            elif self.rxMessage[1:3] == '12':
                if ((self.rxMessage[3:5] == '00' and self.gpioIn_clampClose) or
                        (self.rxMessage[3:5] == '01' and self.gpioIn_clampOpen)):
                    UNIT_OP_SUCCESS = True
            # rotation 0d/90d
            elif self.rxMessage[1:3] == '13':
                if ((self.rxMessage[3:5] == '00' and self.gpioIn_rotation0) or
                        (self.rxMessage[3:5] == '01' and self.gpioIn_rotation90)):
                    UNIT_OP_SUCCESS = True
            # reset Error
            elif self.rxMessage[1:3] == '14':
                self.socketStatus = SocketStatus.READY
                self.socketError = SocketError.NONE

            if UNIT_OP_SUCCESS:
                self.isExecProcess_unitOp = False
                self.socketStatus = SocketStatus.READY
                self.socketError = SocketError.NONE
                self.replyMessage('S' + self.rxMessage[1:5] + '000')
            else:
                self.cntTimeOutExecProcess += 1
                if self.cntTimeOutExecProcess >= TIMEOUT_UNIT_OP:
                    if self.rxMessage[1:3] == '10':
                        if self.rxMessage[3:5] == '00':
                            self.socketError = SocketError.VACUUM_OFF
                        else:
                            self.socketError = SocketError.VACUUM_ON
                    elif self.rxMessage[1:3] == '11':
                        if self.rxMessage[3:5] == '00':
                            self.socketError = SocketError.CLAMP_CLOSE
                        else:
                            self.socketError = SocketError.CLAMP_OPEN
                    elif self.rxMessage[1:3] == '12':
                        if self.rxMessage[3:5] == '00':
                            self.socketError = SocketError.SOCKET_CLOSE
                        else:
                            self.socketError = SocketError.SOCKET_OPEN
                    elif self.rxMessage[1:3] == '13':
                        if self.rxMessage[3:5] == '00':
                            self.socketError = SocketError.ROTATION_0D
                        else:
                            self.socketError = SocketError.ROTATION_90D

                    self.isExecProcess_unitOp = False
                    self.socketStatus = SocketStatus.ERROR
                    self.replyMessage('S' + self.rxMessage[1:5] + '001')

    def execProcess_loadUnload(self):
        # print(self.isExecProcess_loadUnload, self.idxExecProcess_loadUnload)

        # Unload
        if self.rxMessage[3:5] == '00':
            """
            if self.idxExecProcess_loadUnload == 0:
                if self.gpioIn_rotation90 and self.gpioIn_clampClose and self.gpioIn_socketClose:
                    self.cntTimeOutExecProcess = 0
                    self.idxExecProcess_loadUnload += 1
                else:
                    if not self.gpioIn_rotation90:
                        self.socketError = SocketError.ROTATION_90D
                    elif not self.gpioIn_clampClose:
                        self.socketError = SocketError.CLAMP_CLOSE
                    elif not self.gpioIn_socketClose:
                        self.socketError = SocketError.SOCKET_CLOSE
            """
            if self.idxExecProcess_loadUnload == 1:
                self.set_gpioOut(self.gpioOut_rotation0, True)
                self.set_gpioOut(self.gpioOut_rotation90, False)
                self.idxExecProcess_loadUnload += 1
            elif self.idxExecProcess_loadUnload == 2:
                self.socketError = SocketError.ROTATION_0D
                if self.gpioIn_rotation0:
                    self.cntTimeOutExecProcess = 0
                    self.cntExecProcess = 0
                    self.idxExecProcess_loadUnload += 1
            elif self.idxExecProcess_loadUnload == 3:
                self.cntExecProcess += 1
                if self.cntExecProcess >= 5:
                    self.cntTimeOutExecProcess = 0
                    self.idxExecProcess_loadUnload += 1
            elif self.idxExecProcess_loadUnload == 4:
                self.set_gpioOut(self.gpioOut_clampClose, False)
                self.set_gpioOut(self.gpioOut_clampOpen, True)
                self.cntExecProcess = 0
                self.idxExecProcess_loadUnload += 1
            elif self.idxExecProcess_loadUnload == 5:
                self.cntExecProcess += 1
                if self.cntExecProcess >= 5:
                    self.cntTimeOutExecProcess = 0
                    self.idxExecProcess_loadUnload += 1
            elif self.idxExecProcess_loadUnload == 6:
                self.socketError = SocketError.CLAMP_OPEN
                if self.gpioIn_clampOpen:
                    self.cntTimeOutExecProcess = 0
                    self.servo_pwm.duty_u16(self.socketCover_pwmDuty_open)
                    self.idxExecProcess_loadUnload += 1
            elif self.idxExecProcess_loadUnload == 7:
                self.socketError = SocketError.SOCKET_OPEN
                if self.gpioIn_socketOpen:
                    self.isExecProcess_loadUnload = False
                    self.socketStatus = SocketStatus.READY
                    self.socketError = SocketError.NONE
                    self.replyMessage('S' + self.rxMessage[1:5] + '000')
        # Load
        else:
            if self.idxExecProcess_loadUnload == 0:
                if self.gpioIn_rotation0 and self.gpioIn_clampOpen and self.gpioIn_socketOpen and not self.gpioIn_vacumm:
                    self.cntTimeOutExecProcess = 0
                    self.idxExecProcess_loadUnload += 1
                else:
                    if not self.gpioIn_rotation0:
                        self.socketError = SocketError.ROTATION_0D
                    elif not self.gpioIn_clampOpen:
                        self.socketError = SocketError.CLAMP_OPEN
                    elif not self.gpioIn_socketOpen:
                        self.socketError = SocketError.SOCKET_OPEN
            elif self.idxExecProcess_loadUnload == 1:
                self.set_gpioOut(self.gpioOut_vacumm, True)
                self.idxExecProcess_loadUnload += 1
            elif self.idxExecProcess_loadUnload == 2:
                self.socketError = SocketError.VACUUM_ON
                if self.gpioIn_vacumm:
                    self.cntExecProcess = 0
                    self.cntTimeOutExecProcess = 0
                    self.idxExecProcess_loadUnload += 1
            elif self.idxExecProcess_loadUnload == 3:
                self.servo_pwm.duty_u16(self.socketCover_pwmDuty_close)
                self.idxExecProcess_loadUnload += 1
            elif self.idxExecProcess_loadUnload == 4:
                self.socketError = SocketError.SOCKET_CLOSE
                if self.gpioIn_socketClose:
                    self.cntTimeOutExecProcess = 0
                    self.idxExecProcess_loadUnload += 1
            elif self.idxExecProcess_loadUnload == 5:
                self.set_gpioOut(self.gpioOut_clampClose, True)
                self.set_gpioOut(self.gpioOut_clampOpen, False)
                self.set_gpioOut(self.gpioOut_vacumm, False)
                self.idxExecProcess_loadUnload += 1
            elif self.idxExecProcess_loadUnload == 6:
                self.socketError = SocketError.CLAMP_CLOSE
                if self.gpioIn_clampClose:
                    self.cntExecProcess = 0
                    self.cntTimeOutExecProcess = 0
                    self.idxExecProcess_loadUnload += 1
            elif self.idxExecProcess_loadUnload == 7:
                self.cntExecProcess += 1
                if self.cntExecProcess >= 5:
                    self.idxExecProcess_loadUnload += 1
            elif self.idxExecProcess_loadUnload == 8:
                self.set_gpioOut(self.gpioOut_rotation0, False)
                self.set_gpioOut(self.gpioOut_rotation90, True)
                self.idxExecProcess_loadUnload += 1
            elif self.idxExecProcess_loadUnload == 9:
                self.socketError = SocketError.ROTATION_90D
                if self.gpioIn_rotation90:
                    self.isExecProcess_loadUnload = False
                    self.socketStatus = SocketStatus.READY
                    self.socketError = SocketError.NONE
                    self.replyMessage('S' + self.rxMessage[1:5] + '000')

        self.cntTimeOutExecProcess += 1
        if self.cntTimeOutExecProcess >= TIMEOUT_LOAD_UNLOAD:
            errorCode = self.checkErrorCode()

            self.isExecProcess_loadUnload = False
            self.socketStatus = SocketStatus.ERROR
            self.socketError = SocketError.LOAD_UNLOAD
            self.replyMessage('S' + self.rxMessage[1:5] + errorCode)

    def replyMessage(self, message):
        # Status_All 응답
        if self.rxMessage[1:3] == '31':
            pass
        else:
            W5500.sendMessage(self.UDP_CLIENT, message)

    def checkErrorCode(self):
        errorCode = '001'

        if self.socketError == SocketError.VACUUM_OFF:
            errorCode = '010'
        elif self.socketError == SocketError.VACUUM_ON:
            errorCode = '011'
        elif self.socketError == SocketError.CLAMP_CLOSE:
            errorCode = '012'
        elif self.socketError == SocketError.CLAMP_OPEN:
            errorCode = '013'
        elif self.socketError == SocketError.SOCKET_CLOSE:
            errorCode = '014'
        elif self.socketError == SocketError.SOCKET_OPEN:
            errorCode = '015'
        elif self.socketError == SocketError.ROTATION_0D:
            errorCode = '016'
        elif self.socketError == SocketError.ROTATION_90D:
            errorCode = '017'
        elif self.socketError == SocketError.LOAD_UNLOAD:
            errorCode = '018'

        return errorCode

if __name__ == "__main__":
    cnt_msec = 0
    main = MainHandler()

    while True:
        cnt_msec += 1
        main.func_1msec()

        if not cnt_msec % 10:
            main.func_10msec()

        if not cnt_msec % 25:
            main.func_25msec()

        if not cnt_msec % 50:
            main.func_50msec()

        if not cnt_msec % 100:
            main.func_100msec()

        if not cnt_msec % 500:
            main.func_500msec()

        time.sleep_ms(1)
