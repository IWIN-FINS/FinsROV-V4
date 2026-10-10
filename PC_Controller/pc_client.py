# -*- coding: utf-8 -*-
import os
import socket
import time
import pygame
import struct
import threading
from collections import deque

PI_IP = "192.168.138.2"
PI_PORT = 5000
DIRECTION_THRESHOLD = 0.1
DEPTH_THRESHOLD = 0.2
ROTATE_THRESHOLD = 0.2
HEARTBEAT_INTERVAL = 0.5
FPS = 30

# 深度
DEPTH_MIN = 0
DEPTH_MAX = 1000
DEPTH_STEP = 10
DEPTH_REPEAT_INTERVAL = 0.1

# 舵机
PWM_MIN, PWM_MAX = 500, 2500
SERVO_STEP = 20
SERVO_REPEAT_INTERVAL = 0.10
# 夹爪按住按钮时每 0.10 秒补发一次方向命令。
# 松手后只发一次 0，不会一直占用通信。
GRIPPER_REPEAT_INTERVAL = 0.10
# pygame 里的按钮编号不一定等于手柄外壳印的编号。
# 界面会显示 Pressed buttons，用它确认实际编号后改这里。
GRIPPER_CLOSE_BUTTON = 4
GRIPPER_OPEN_BUTTON = 5
aaaa = 1500
bbbb = 1500
cccc = 1500
dddd = 1500
target_depth = 300

MAX_SPEED = 99  # 最大速度值

# 新协议格式：
# 帧头(2B) + 地址(1B) + 命令(1B) + 长度(1B) + 数据(nB) + CRC16(2B)
# CRC16 使用 CRC-16/MODBUS，计算范围为：地址 + 命令 + 长度 + 数据，低字节在前。
FRAME_HEADER = b"\xAA\x55"
PROPELLER_ADDR = 0x02
SERVO_ADDR = 0x03
TELEMETRY_ADDR = 0xF0
TELEMETRY_CMD_STATE = 0x01
TELEMETRY_BASE_FORMAT = "<HIfff"
TELEMETRY_BASE_SIZE = struct.calcsize(TELEMETRY_BASE_FORMAT)
TELEMETRY_PWM_COUNT = 12
TELEMETRY_PWM_PACKED_SIZE = 15
TELEMETRY_PAYLOAD_SIZE = TELEMETRY_BASE_SIZE + TELEMETRY_PWM_PACKED_SIZE
# 下位机新增的夹爪设备地址。设备数组里追加在 LED 后面，所以地址是 5。
GRIPPER_ADDR = 0x05

CMD_SET_DEPTH_VALUE = 0x00  # [int16 depth_x10]
CMD_SET_DEPTH_INCRE = 0x01  # [int16 depth_x10]
CMD_SET_DEPTH_FORCE = 0x02  # [int16 force]
CMD_TOGGLE_FLOAT = 0x10
CMD_TOGGLE_YAW_LOOP = 0x20
# 夹爪命令：
# 0x00 是方向控制，payload 为 int8，+1 闭合，-1 张开，0 停止保持。
# 0x01 是调试用绝对 PWM，正常手柄控制不用它。
CMD_GRIPPER_SPEED = 0x00
CMD_GRIPPER_PWM = 0x01
CMD_SET_SERVO_PWM = 0x03
CMD_TOGGLE_YAW_DOWN = 0x01
CMD_SET_YAW_VALUE = 0x21  # [int16 deg]
CMD_INC_YAW_INCRE = 0x22  # [int16 deg]
CMD_SET_YAW_RATE = 0x23  # [int16 deg_per_s]，0=停转并锁角
CMD_SET_PLANAR_VEL = 0x30  # [int8 x, int8 y]，-99..99
# 组合运动命令：[int8 x, int8 y, int16 yaw_rate, int16 depth_force]
# 这样一个控制周期里，水平/垂直/旋转/深度可以合并成一帧发送。
# 注意：下位机也需要新增对这个命令的解析逻辑。
CMD_SET_COMBINED_MOTION = 0x31
isHovering = False
isYawLooping = False
SERVO_X_INDEX = 1
SERVO_Y_INDEX = 2

pygame.init()
screen = pygame.display.set_mode((600, 340))  # 适当增加高度显示速度信息
pygame.display.set_caption("Joystick TCP Controller")
clock = pygame.time.Clock()
font_small = pygame.font.Font(None, 20)
font_norm = pygame.font.Font(None, 24)


def draw_text(x, y, text, font=font_small, color=(0, 0, 0)):
    surface = font.render(text, True, color)
    screen.blit(surface, (x, y))
    return y + surface.get_height() + 2


def pwm_to_deg(pwm, pwm_min=PWM_MIN, pwm_max=PWM_MAX, full_deg=270.0):
    val = (pwm - pwm_min) / (pwm_max - pwm_min)
    val = max(0.0, min(1.0, val))
    return round(val * full_deg, 1)


recv_log = deque(maxlen=20)
rx_thread = None
rx_stop = threading.Event()
log_file_path = "recv_log.txt"
latest_gripper_pwm = None
latest_telemetry = None


def log_recv(msg):
    global latest_gripper_pwm
    ts = time.strftime("%H:%M:%S")
    line = f"{ts}  {msg}"
    recv_log.appendleft(line)

    # 兼容旧版CSV遥测；新版二进制遥测由handle_received_frame解析。
    try:
        parts = [p.strip() for p in msg.split(",")]
        if len(parts) >= 4:
            latest_gripper_pwm = int(float(parts[-1]))
    except ValueError:
        pass

    # 同时写入文件
    try:
        with open(log_file_path, "a", encoding="utf-8") as f:
            f.write(line + "\n")
    except Exception as e:
        print(f"[FILE] 写入日志失败: {e}")


def unpack_telemetry_pwm(packed):
    if len(packed) != TELEMETRY_PWM_PACKED_SIZE:
        raise ValueError(
            f"packed PWM length must be {TELEMETRY_PWM_PACKED_SIZE}"
        )

    packed_value = int.from_bytes(packed, byteorder="little", signed=False)
    return tuple(
        1000 + ((packed_value >> (index * 10)) & 0x03FF)
        for index in range(TELEMETRY_PWM_COUNT)
    )


def handle_received_frame(addr, cmd, payload):
    global latest_gripper_pwm, latest_telemetry

    if addr == TELEMETRY_ADDR and cmd == TELEMETRY_CMD_STATE:
        if len(payload) != TELEMETRY_PAYLOAD_SIZE:
            log_recv(
                f"# 遥测长度错误: expected={TELEMETRY_PAYLOAD_SIZE}, "
                f"actual={len(payload)}"
            )
            return

        seq, uptime_ms, roll_rate, pitch_rate, yaw_rate = struct.unpack(
            TELEMETRY_BASE_FORMAT,
            payload[:TELEMETRY_BASE_SIZE],
        )
        pwm_values = unpack_telemetry_pwm(payload[TELEMETRY_BASE_SIZE:])
        propeller_pwm = pwm_values[:8]
        servo_pwm = pwm_values[8:]
        gripper_pwm = servo_pwm[3]
        latest_gripper_pwm = gripper_pwm
        latest_telemetry = {
            "seq": seq,
            "uptime_ms": uptime_ms,
            "roll_rate": roll_rate,
            "pitch_rate": pitch_rate,
            "yaw_rate": yaw_rate,
            "gripper_pwm": gripper_pwm,
            "propeller_pwm": propeller_pwm,
            "servo_pwm": servo_pwm,
        }
        log_recv(
            f"TEL seq={seq} t={uptime_ms}ms "
            f"rate=({roll_rate:+.2f},{pitch_rate:+.2f},{yaw_rate:+.2f}) "
            f"P={propeller_pwm} S={servo_pwm}"
        )
        return

    log_recv(
        f"RX addr=0x{addr:02X} cmd=0x{cmd:02X} "
        f"payload={payload.hex(' ')}"
    )


def process_recv_buffer(buf):
    """Parse binary AA55 frames while retaining legacy newline text support."""
    while buf:
        if buf.startswith(FRAME_HEADER):
            if len(buf) < 5:
                return

            payload_len = buf[4]
            frame_len = 7 + payload_len
            if len(buf) < frame_len:
                return

            frame = bytes(buf[:frame_len])
            body = frame[2:-2]
            received_crc = struct.unpack("<H", frame[-2:])[0]
            if received_crc != crc16_modbus(body):
                log_recv("# 接收帧 CRC 错误")
                del buf[0]
                continue

            del buf[:frame_len]
            handle_received_frame(frame[2], frame[3], frame[5:-2])
            continue

        header_index = buf.find(FRAME_HEADER)
        newline_index = buf.find(b"\n")

        if newline_index >= 0 and (
            header_index < 0 or newline_index < header_index
        ):
            line = bytes(buf[:newline_index])
            del buf[:newline_index + 1]
            text = line.decode(errors="ignore").strip("\r")
            if text:
                log_recv(text)
            continue

        if header_index > 0:
            del buf[:header_index]
            continue

        if len(buf) > 4096:
            log_recv("# 接收缓冲区溢出，已清空")
            buf.clear()
        return


def start_rx_thread():
    global rx_thread, rx_stop
    rx_stop.clear()
    rx_thread = threading.Thread(target=recv_worker, daemon=True)
    rx_thread.start()


def recv_worker():
    """后台线程：接收并解析下位机二进制遥测或旧版文本。"""
    buf = bytearray()
    while not rx_stop.is_set():
        try:
            data = sock.recv(1024)
            if not data:
                log_recv("# 连接丢失，正在重连...")
                connect_to_pi()
                continue
            buf.extend(data)
            process_recv_buffer(buf)
        except Exception as e:
            log_recv(f"# 接收错误: {e}")
            time.sleep(1)


sock = None
send_log = deque(maxlen=12)


def connect_to_pi():
    global sock
    while True:
        try:
            if sock:
                try:
                    sock.close()
                except Exception:
                    pass
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.connect((PI_IP, PI_PORT))
            sock = s
            print(f"[NET] Connected to {PI_IP}:{PI_PORT}")
            log_send(f"# Connected {PI_IP}:{PI_PORT}")
            start_rx_thread()
            return
        except Exception as e:
            print(f"[NET] Connect failed: {e}, retrying...")
            time.sleep(3)


def log_send(msg):
    ts = time.strftime("%H:%M:%S")
    send_log.appendleft(f"{ts}  {msg}")


def send_tcp(msg):
    global sock
    data = (msg + "\n").encode()
    try:
        sock.sendall(data)
        log_send(msg)
    except Exception as e:
        print(f"[NET] Send failed: {e}")
        connect_to_pi()


def crc16_modbus(data):
    """CRC-16/MODBUS，多项式 0xA001，初值 0xFFFF。"""
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
            crc &= 0xFFFF
    return crc


def build_frame(addr, cmd, payload=b""):
    """组帧：AA55 + 地址 + 命令 + 长度 + 数据 + CRC16。"""
    payload = bytes(payload)
    if len(payload) > 255:
        raise ValueError("payload length must be <= 255")

    body = bytes([addr & 0xFF, cmd & 0xFF, len(payload) & 0xFF]) + payload
    crc = crc16_modbus(body)
    return FRAME_HEADER + body + struct.pack("<H", crc)


def send_pkt(addr, cmd, payload=b""):
    """发送一帧二进制协议。"""
    global sock
    frame = build_frame(addr, cmd, payload)
    try:
        sock.sendall(frame)
        log_send(f"TX data={frame.hex(' ')}")
    except Exception as e:
        print(f"[NET] Send failed: {e}")
        connect_to_pi()


def send_servo_pwm(index, pwm):
    payload = bytes([index & 0xFF]) + i16le(pwm)
    send_pkt(SERVO_ADDR, CMD_SET_SERVO_PWM, payload)


def send_gripper_speed(direction):
    """发送夹爪方向命令。

    direction > 0：往闭合方向缓慢增加 PWM
    direction < 0：往张开方向缓慢减少 PWM
    direction = 0：停止并保持当前角度
    """
    if direction > 0:
        direction = 1
    elif direction < 0:
        direction = -1
    else:
        direction = 0
    send_pkt(GRIPPER_ADDR, CMD_GRIPPER_SPEED, i8(direction))


def send_combined_motion(x_pct, y_pct, yaw_rate_cmd, depth_force_cmd):
    """发送合并后的平面运动 + 偏航角速度 + 深度命令。"""
    payload = i8(x_pct) + i8(y_pct) + i16le(yaw_rate_cmd) + i16le(depth_force_cmd)
    send_pkt(PROPELLER_ADDR, CMD_SET_COMBINED_MOTION, payload)


def is_button_pressed(js, index):
    """安全读取按钮状态，避免不同手柄按钮数量不一致时报错。"""
    return 0 <= index < js.get_numbuttons() and js.get_button(index)


def i16le(val):
    return struct.pack("<h", int(val))


def i8(val):
    # 截断到 int8 范围
    v = int(max(-128, min(127, int(val))))
    return struct.pack("b", v)


def calculate_speed(axis_value, threshold):
    """根据摇杆轴值计算速度（0-MAX_SPEED）"""
    speed = 0
    if not -threshold < axis_value < threshold:
        if axis_value > 0:
            speed = int((axis_value - threshold) * MAX_SPEED / (1 - threshold))
        else:
            speed = -int((-axis_value - threshold) * MAX_SPEED / (1 - threshold))
    return speed


def main():
    global aaaa, bbbb, cccc, dddd, target_depth, isHovering, isYawLooping
    global latest_gripper_pwm, latest_telemetry
    connect_to_pi()

    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("No joystick found.")
        return
    js = pygame.joystick.Joystick(0)
    js.init()
    print(f"Using joystick: {js.get_name()}")

    last_cmd_hex = None
    last_depth_tick = 0.0
    last_servo_tick = 0.0
    hat_state = (0, 0)

    # 最近一次已发送的组合运动命令：(x_pct, y_pct, yaw_rate_cmd, depth_force_cmd)
    # 上电默认就是全 0，所以这里也从全 0 开始，避免首次进入悬浮时补发一帧全 0。
    last_combined_motion = (0, 0, 0, 0)
    # 记录上一次发给夹爪的方向。
    # 用它判断“松手时是否需要发一次 0”，以及“按住时是否到了补发时间”。
    # 下位机初始化时夹爪本来就是停止状态，因此启动时不需要先补发一次 0。
    last_gripper_direction = 0
    last_gripper_tick = 0.0

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if event.type == pygame.JOYHATMOTION:
                hat_state = js.get_hat(0)
            if event.type == pygame.JOYBUTTONDOWN:
                # 调试用：显示 pygame 实际识别到的按钮编号。
                log_send(f"JOY button down: {event.button}")
                if event.button == 7:  # 悬浮开关
                    next_hovering = not isHovering
                    send_pkt(PROPELLER_ADDR, CMD_TOGGLE_FLOAT, bytes([1 if next_hovering else 0]))
                    isHovering = next_hovering
                if event.button == 0:
                    next_yaw_looping = not isYawLooping
                    send_pkt(PROPELLER_ADDR, CMD_TOGGLE_YAW_LOOP, bytes([1 if next_yaw_looping else 0]))
                    isYawLooping = next_yaw_looping
            if event.type == pygame.JOYBUTTONUP:
                # 松手也记录一次，确认按键事件是否正常进入程序。
                log_send(f"JOY button up: {event.button}")

        try:
            leftH = js.get_axis(0)
            leftV = js.get_axis(1)
            rightH = js.get_axis(2)
            rightV = js.get_axis(3)
        except Exception:
            leftH = leftV = rightH = rightV = 0.0

        # —— 平面推力百分比 + 偏航角速度：在一个周期末尾合并发送 ——
        # 你的 MCU 会把 -99..99 的百分比映射到 PWM 增量
        x_pct = calculate_speed(leftH, DIRECTION_THRESHOLD)
        y_pct = -calculate_speed(leftV, DIRECTION_THRESHOLD)
        # y_pct = calculate_speed(leftV, DIRECTION_THRESHOLD)
        # —— 右摇杆水平控制“偏航角速度” ——
        # 右杆>0：顺时针为正；右杆<0：逆时针为负（与之前 E/Q 的方向保持一致）
        yaw_rate_cmd = 0
        if not (-ROTATE_THRESHOLD < rightH < ROTATE_THRESHOLD):
            yaw_rate_cmd = calculate_speed(rightH, ROTATE_THRESHOLD)

        depth_force_cmd = 0
        if not (-DEPTH_THRESHOLD < rightV < DEPTH_THRESHOLD):
            depth_force_cmd = calculate_speed(rightV, DEPTH_THRESHOLD)

        # 一个周期里统一发一条总消息，包含 x / y / yaw / depth
        combined_motion = (x_pct, y_pct, yaw_rate_cmd, depth_force_cmd)
        if isHovering and combined_motion != last_combined_motion:
            send_combined_motion(*combined_motion)
            last_combined_motion = combined_motion

        now = time.time()

        # 夹爪控制使用 pygame 的按钮编号：
        # GRIPPER_CLOSE_BUTTON：闭合
        # GRIPPER_OPEN_BUTTON：张开
        # 如果你的手柄外壳标号和 pygame 编号不一致，用 joy_test.py 看实际输出后改这里。
        close_pressed = is_button_pressed(js, GRIPPER_CLOSE_BUTTON)
        open_pressed = is_button_pressed(js, GRIPPER_OPEN_BUTTON)
        if close_pressed and not open_pressed:
            gripper_direction = 1
        elif open_pressed and not close_pressed:
            gripper_direction = -1
        else:
            gripper_direction = 0

        # 发送策略：
        # 1. 方向变化时立刻发，比如按下、松手、换方向。
        # 2. 按住非零方向时，每 0.10 秒补发一次，防止链路偶发丢包。
        # 3. 松手后 gripper_direction=0 只会发送一次，后面不会持续刷 0。
        if gripper_direction != last_gripper_direction or (
            gripper_direction != 0 and now - last_gripper_tick >= GRIPPER_REPEAT_INTERVAL
        ):
            send_gripper_speed(gripper_direction)
            last_gripper_direction = gripper_direction
            last_gripper_tick = now

        # 舵机控制 (bbbb, cccc)
        if hat_state != (0, 0) and (now - last_servo_tick >= SERVO_REPEAT_INTERVAL):
            hx, hy = hat_state
            changed = False
            if hy == 1:
                bbbb = min(bbbb + SERVO_STEP, PWM_MAX)
                changed = True
            elif hy == -1:
                bbbb = max(bbbb - SERVO_STEP, PWM_MIN)
                changed = True
            if hx == 1:
                cccc = min(cccc + SERVO_STEP, PWM_MAX)
                changed = True
            elif hx == -1:
                cccc = max(cccc - SERVO_STEP, PWM_MIN)
                changed = True
            if changed:
                send_servo_pwm(SERVO_X_INDEX, bbbb)
                send_servo_pwm(SERVO_Y_INDEX, cccc)
            last_servo_tick = now

        # 心跳包保持原逻辑
        # if now - last_hb >= HEARTBEAT_INTERVAL:
        #     send_tcp(HEARTBEAT_MSG)
        #     last_hb = now

        # UI显示新增速度信息
        screen.fill((255, 255, 255))
        y = 8
        y = draw_text(10, y, f"Left(H,V)=({leftH:+.2f},{leftV:+.2f})  Right(H,V)=({rightH:+.2f},{rightV:+.2f})")
        y = draw_text(10, y,
                      f"Planar: x={x_pct}, y={y_pct}   Yaw rate cmd: {yaw_rate_cmd:+d} deg/s")
        if latest_telemetry is not None:
            y = draw_text(
                10,
                y,
                "Telemetry: "
                f"seq={latest_telemetry['seq']} "
                f"rates=({latest_telemetry['roll_rate']:+.2f},"
                f"{latest_telemetry['pitch_rate']:+.2f},"
                f"{latest_telemetry['yaw_rate']:+.2f}) deg/s"
            )
        pressed_buttons = [i for i in range(js.get_numbuttons()) if js.get_button(i)]
        y = draw_text(10, y, f"Pressed buttons: {pressed_buttons}")
        gripper_pwm_text = latest_gripper_pwm if latest_gripper_pwm is not None else "--"
        if gripper_direction > 0:
            gripper_state = "closing"
        elif gripper_direction < 0:
            gripper_state = "opening"
        else:
            gripper_state = "hold"
        y = draw_text(
            10,
            y,
            f"Gripper: {gripper_state}  PWM: {gripper_pwm_text}"
        )
        y = draw_text(10, y, f"Depth target: {target_depth} (0.1 m units)")
        deg_b = pwm_to_deg(bbbb)
        deg_c = pwm_to_deg(cccc)
        y = draw_text(10, y, f"Servos: x={deg_b:.1f}°  y={deg_c:.1f}°")

        # 绘制发送日志
        y = draw_text(10, y + 8, "Sent commands (latest first):")
        for line in list(send_log):
            y = draw_text(14, y, line)

        # 绘制接收日志
        y = draw_text(320, 8, "Received messages:")
        for line in list(recv_log):
            y = draw_text(324, y, line)
        pygame.display.flip()
        clock.tick(FPS)

    try:
        sock.close()
    except Exception:
        pass
    pygame.quit()


if __name__ == "__main__":
    main()
