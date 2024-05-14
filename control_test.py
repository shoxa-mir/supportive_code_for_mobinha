#!/usr/bin/python3

import can
import cantools
import threading
import time
import datetime

class JIAT:
    def __init__(self):
        self.bus = can.ThreadSafeBus(
            interface='socketcan', channel='can2', bitrate=500000)
        self.db = cantools.database.load_file('jiat_can.dbc')
        self.accel = 0
        self.enable = 1
        self.brake = 0
        self.steering = 0
        self.steering_takeover_th = 30
        self.alv_cnt = 0
        self.reset = 1
        self.err_time = datetime.datetime.now()
        self.recv_err_cnt = 0
        self.right_turn = 0
        self.left_turn = 0
        self.acc_override = None
        self.brk_override = None
        self.steering_override = None
        self.tick = {1: 0, 0.5: 0, 0.2: 0, 0.1: 0, 0.02: 0}

    def reset_trigger(self):
        if self.acc_override or self.brk_override or self.steering_override:
            self.reset = 1
        elif self.reset and not (self.acc_override or self.brk_override or self.steering_override):
            self.reset = 0

    def timer(self, sec):
        current_time = time.time()
        if current_time - self.tick[sec] > sec:
            self.tick[sec] = current_time
            return True
        return False

    def daemon(self):
        while True:
            self.longitudinal_cmd()
            self.cbus_rcv()
            if self.acc_override or self.brk_override or self.steering_override:
                print("TAKEOVER")
                self.enable = 0
                self.reset_trigger()

    def alive_counter(self):
        self.alv_cnt = (self.alv_cnt + 1) % 256

    def right_mover(self):
        for i in range(0, 480, 5):
            self.steering = i
            time.sleep(0.02)

    def left_mover(self):
        for i in range(480, 0, -5):
            self.steering = i
            time.sleep(0.02)

    def longitudinal_cmd(self):
        self.alive_counter()
        cmd_str = self.create_cmd_str()
        cmd_acc_brk = self.create_cmd_acc_brk()
        cmd_light = self.create_cmd_light()

        self.send_message('HLC_LAT_CMD', 0x300, cmd_str)
        self.send_message('HLC_LONG_CMD', 0x310, cmd_acc_brk)
        self.send_message('HLC_LAMP_CMD', 0x320, cmd_light)
        self.reset_trigger()

    def create_cmd_str(self):
        return {
            'HLC_STR_MODE_CMD': self.enable,
            'HLC_STR_ANG_CMD': self.steering,
            'HLC_LAT_Alive_Cnt': self.alv_cnt,
            'HLC_TRQ_TAKEOVER_THREHOLD': self.steering_takeover_th
        }

    def create_cmd_acc_brk(self):
        return {
            'HLC_ACC_MODE_CMD': self.enable,
            'HLC_ACC_Pedal_CMD': self.accel,
            'HLC_BRK_MODE_CMD': self.enable,
            'HLC_BRK_Pedal_CMD': self.brake,
            'HLC_LONG_Alive_Cnt': self.alv_cnt
        }

    def create_cmd_light(self):
        return {
            'HLC_TAKEOVER_RESET_CMD': self.reset,
            'HLC_LEFT_TURN_SIG_CMD': self.left_turn,
            'HLC_RIGHT_TURN_SIG_CMD': self.right_turn,
            'HLC_LAMP_Alive_Cnt': self.alv_cnt,
            'HLC_Button_Lamp_SIG_CMD': 0,
            'HLC_HAZARD_TURN_SIG_CMD': 0,
            'HLC_LAMP_MODE_CMD': self.enable
        }

    def send_message(self, message_name, arb_id, cmd):
        try:
            msg = self.db.encode_message(message_name, cmd)
            can_msg = can.Message(arbitration_id=arb_id, data=msg, is_extended_id=False)
            self.bus.send(can_msg)
        except Exception as e:
            print(f"Error sending {message_name}: {e}")

    def cbus_rcv(self):
        data = self.bus.recv(0.2)
        if data is None:
            self.recv_err_cnt += 1
            print("recv err cnt:", self.recv_err_cnt)
            if self.err_time is None:
                self.err_time = datetime.datetime.now()
            return

        try:
            if data.arbitration_id == 1296:
                res = self.db.decode_message(data.arbitration_id, data.data)
                print("LLC_LONG_STAT(acc_brk_motor):", res)
            elif data.arbitration_id == 1280:
                res = self.db.decode_message(data.arbitration_id, data.data)
                print("LLC_LAT_STAT(str_):", res)
            elif data.arbitration_id == 1312:
                res = self.db.decode_message(data.arbitration_id, data.data)
                print("LLC_LAMP_STAT(lamp):", res)

            if self.err_time:
                recovery_time = datetime.datetime.now() - self.err_time
                print(f"Recovered in {recovery_time.total_seconds()} seconds")
                self.err_time = None
        except cantools.database.errors.DecodeError as e:
            print(f"Decode error: {e}")
        except Exception as e:
            print(f"Unexpected error: {e}")

    def controller(self):
        commands = {
            99: self.enable_cmd,
            88: self.disable_cmd,
            1001: self.reset_cmd,
            -30: self.left_steer_cmd,
            30: self.right_steer_cmd,
            1111: self.left_turn_cmd,
            2222: self.right_turn_cmd,
            1000: exit
        }

        while True:
            try:
                cmd = int(input('99: enable, 88: disable , 1001: reset , accel:0~10 , brake:-1~-20 , left_steer:-30, right_steer:30, 1000: exit\nleft_light:1111, right_light:2222\n'))
                if cmd in commands:
                    commands[cmd]()
                elif 0 <= cmd <= 10:
                    self.set_accel_cmd(cmd)
                elif -20 <= cmd <= -1:
                    self.set_brake_cmd(cmd)
                else:
                    print("Invalid command")
            except ValueError:
                print("Please enter a valid integer command")

    def enable_cmd(self):
        self.brake = 0
        self.accel = 0
        self.steering = 0
        self.enable = 1

    def disable_cmd(self):
        self.enable = 0

    def reset_cmd(self):
        self.reset = 0

    def left_steer_cmd(self):
        self.steering = -360

    def right_steer_cmd(self):
        self.steering = 360

    def left_turn_cmd(self):
        self.left_turn = 1
        self.right_turn = 0

    def right_turn_cmd(self):
        self.left_turn = 0
        self.right_turn = 1

    def set_accel_cmd(self, cmd):
        self.accel = float(cmd) * 5
        self.brake = 0

    def set_brake_cmd(self, cmd):
        self.brake = -float(cmd) * 5
        self.accel = 0

if __name__ == '__main__':
    jiat = JIAT()
    t1 = threading.Thread(target=jiat.daemon)
    t2 = threading.Thread(target=jiat.controller)

    t1.start()
    t2.start()

    t1.join()
    t2.join()
