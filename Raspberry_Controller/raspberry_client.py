# -*- coding: utf-8 -*-
import glob, os, queue, re, shlex, socket, subprocess, threading, time, traceback
import serial, serial.tools.list_ports as list_ports
import fcntl, sys, signal

try:
    sys.stdout.reconfigure(line_buffering=True)
except Exception:
    pass

# ===================== 基本网络配置（TCP-Serial 桥） =====================
TCP_IP   = "0.0.0.0"
TCP_PORT = 5000

# ===================== 串口配置 =====================
SERIAL_PORT = "/dev/ttyUSB0"   # 只连接这里指定的串口；断开后等待它重新出现
BAUD_RATE   = 115200
SER_TIMEOUT = 0.02             # 串口读超时（仅在读线程使用）

# ===================== 相机原生 H.264 直通（GStreamer RTP/UDP） =====================
# 留空时自动扫描支持 H.264 的 USB 相机节点；也可以手动填写 /dev/videoX。
VIDEO_DEVICE   = ""
VIDEO_W        = 1280
VIDEO_H        = 720
VIDEO_FPS      = 30
PC_VIDEO_PORT  = 5600          # QGroundControl 默认也是 5600
RTP_PT         = 96            # H.264 常用 96

# ===================== 发送端控制开关 =====================
# 若你希望始终推向某一 PC，而不是“最近连上 TCP:5000 的 IP”，在此填写：
# 强烈建议：如果你想“程序一启动就一直推流”，这里设置成上位机的固定 IP。
FORCE_TARGET_IP   = "192.168.138.10"          # 例如 "192.168.138.10"；留空则等待有客户端连上后来决定 IP
VERBOSE_GST_LOG   = True        # 打印 GStreamer 日志，便于排查

# ===================== 终端调试输出 =====================
LOG_PC_RX     = True            # 打印从 PC 收到的完整控制帧
LOG_SERIAL_TX = True            # 打印写入串口的数据
LOG_SERIAL_RX = True            # 打印从串口收到的数据
LOG_PC_TX     = True            # 打印发回 PC 的数据

# ===================== 控制协议：AA55 帧 =====================
#   AA 55 + ADDRESS + COMMAND + LENGTH + DATA... + CRC16
# CRC16 is Modbus CRC over ADDRESS+COMMAND+LENGTH+DATA, little-endian.
FRAME_HEADER = b"\xAA\x55"
FRAME_HEADER_LEN = len(FRAME_HEADER)
ADDRESS_LEN = 1
COMMAND_LEN = 1
LENGTH_LEN = 1
CRC_LEN = 2
MAX_DATA_LEN = 126
MIN_FRAME_LEN = FRAME_HEADER_LEN + ADDRESS_LEN + COMMAND_LEN + LENGTH_LEN + CRC_LEN
VERIFY_FRAME_CRC = True

# ===================== 全局状态 =====================
ser = None
serial_ready = threading.Event()
serial_lock  = threading.Lock()
running      = True

active_clients = set()
video_proc   = None
video_lock   = threading.Lock()
current_target_ip = None

# PC->Serial 高优先级队列（有界，丢旧保新）
pc_send_queue = queue.Queue(maxsize=512)
serial_writer_started = threading.Event()

# ===================== 工具函数 =====================
def hexs(b: bytes) -> str:
    return ' '.join(f'{x:02X}' for x in b)

def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF

def frame_crc_bytes(body: bytes) -> bytes:
    crc = crc16_modbus(body)
    return bytes((crc & 0xFF, (crc >> 8) & 0xFF))

def is_valid_frame_crc(frame: bytes) -> bool:
    body = frame[FRAME_HEADER_LEN:-CRC_LEN]
    expected = frame_crc_bytes(body)
    actual = frame[-CRC_LEN:]
    return actual == expected

def debug_print_video_state(prefix: str = ""):
    """打印当前视频推流状态，便于调试。"""
    print(
        "[VID-STATE]{} serial_ready={} active_clients={} FORCE_TARGET_IP={} "
        "current_target_ip={} video_proc_alive={}".format(
            prefix,
            serial_ready.is_set(),
            list(active_clients),
            FORCE_TARGET_IP or '""',
            current_target_ip,
            (video_proc is not None and video_proc.poll() is None),
        )
    )

def mark_serial_disconnected(reason: str):
    global ser
    should_notify = False
    with serial_lock:
        old_ser = ser
        ser = None
        if serial_ready.is_set():
            serial_ready.clear()
            should_notify = True

    if old_ser is not None:
        try:
            old_ser.close()
        except Exception:
            pass

    if should_notify:
        print(f"[SER] 串口断开，等待重新连接：{reason}")
        on_state_change_maybe_toggle_stream()

# ===================== 单实例锁，防止上次进程没退出干净 =====================
def ensure_single_instance(lock_file="/tmp/tcp_serial_bridge.lock"):
    fd = os.open(lock_file, os.O_RDWR | os.O_CREAT, 0o644)
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        print("[BOOT] 检测到已有实例在运行，请先结束旧进程。")
        sys.exit(1)
    os.write(fd, str(os.getpid()).encode("ascii"))
    # 不要关闭 fd，进程结束后锁自动释放

# ===================== 组帧器：按 AA55 + 长度 + CRC16 解析 =====================
def extract_frames_from_stream(buf: bytearray, source=""):
    frames = []
    i = 0
    n = len(buf)
    log_prefix = f"[PROTO][{source}]" if source else "[PROTO]"

    while True:
        header_index = buf.find(FRAME_HEADER, i)
        if header_index < 0:
            remaining = buf[i:]
            if remaining.endswith(FRAME_HEADER[:1]):
                return frames, bytearray(FRAME_HEADER[:1])
            return frames, bytearray()

        if header_index > i:
            i = header_index

        if n - i < MIN_FRAME_LEN:
            break

        length_index = i + FRAME_HEADER_LEN + ADDRESS_LEN + COMMAND_LEN
        data_len = buf[length_index]
        if data_len > MAX_DATA_LEN:
            print(f"{log_prefix}[DROP] invalid length={data_len}")
            i += 1
            continue

        total_len = MIN_FRAME_LEN + data_len
        if n - i < total_len:
            break

        frame = bytes(buf[i:i + total_len])
        if VERIFY_FRAME_CRC and not is_valid_frame_crc(frame):
            body = frame[FRAME_HEADER_LEN:-CRC_LEN]
            print(
                f"{log_prefix}[DROP] CRC mismatch "
                f"frame={hexs(frame)} expected={hexs(frame_crc_bytes(body))}"
            )
            i += 1
            continue

        frames.append(frame)
        i += total_len

    remaining = bytearray()
    if i < n:
        remaining.extend(buf[i:])
    return frames, remaining

def _video_device_index(path: str) -> int:
    match = re.search(r"(\d+)$", path)
    return int(match.group(1)) if match else 1 << 30


def _video_device_name(path: str) -> str:
    name_path = os.path.join(
        "/sys/class/video4linux",
        os.path.basename(path),
        "name",
    )
    try:
        with open(name_path, "r", encoding="utf-8") as name_file:
            return name_file.read().strip()
    except OSError:
        return ""


def _advertises_h264(path: str) -> bool:
    try:
        result = subprocess.run(
            ["v4l2-ctl", "-d", path, "--list-formats-ext"],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            universal_newlines=True,
            timeout=3,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired):
        return False

    formats = result.stdout.upper()
    return result.returncode == 0 and ("'H264'" in formats or "H.264" in formats)


def find_h264_video_device() -> str:
    if VIDEO_DEVICE:
        if not os.path.exists(VIDEO_DEVICE):
            raise RuntimeError(
                "configured H.264 camera node does not exist: " + VIDEO_DEVICE
            )
        if not _advertises_h264(VIDEO_DEVICE):
            raise RuntimeError(
                "configured camera node does not advertise H.264: " + VIDEO_DEVICE
            )
        return VIDEO_DEVICE

    candidates = sorted(glob.glob("/dev/video*"), key=_video_device_index)
    camera_candidates = []
    for path in candidates:
        name = _video_device_name(path).lower()
        if (
            _video_device_index(path) < 10
            or "camera" in name
            or "usb" in name
            or "uvc" in name
        ):
            camera_candidates.append(path)

    for path in camera_candidates:
        if _advertises_h264(path):
            print(
                "[VID-SOURCE] native H.264 camera: {} ({})".format(
                    path,
                    _video_device_name(path) or "unknown",
                )
            )
            return path

    raise RuntimeError(
        "no USB camera node advertising H.264 was found; "
        "set VIDEO_DEVICE to the correct /dev/videoX node"
    )


def build_gst_cmd(target_ip: str) -> list:
    """Forward camera-provided H.264 without decoding or re-encoding it."""
    video_device = find_h264_video_device()
    cmd = [
        "gst-launch-1.0",
        "-e",
        "-v",
        "v4l2src",
        f"device={video_device}",
        "do-timestamp=true",
        "!",
        (
            f"video/x-h264,width={VIDEO_W},height={VIDEO_H},"
            f"framerate={VIDEO_FPS}/1"
        ),
        "!",
        "h264parse",
        "config-interval=1",
        "!",
        "video/x-h264,stream-format=byte-stream,alignment=au",
        "!",
        "rtph264pay",
        f"pt={RTP_PT}",
        "config-interval=1",
        "mtu=1400",
        "!",
        "udpsink",
        f"host={target_ip}",
        f"port={PC_VIDEO_PORT}",
        "buffer-size=4194304",
        "sync=false",
        "async=false",
        "qos=false",
    ]

    if VERBOSE_GST_LOG:
        print("[GST-H264-DIRECT-CMD]", " ".join(shlex.quote(arg) for arg in cmd))

    return cmd


def _drain_gst_logs(proc: subprocess.Popen):
    try:
        for line in proc.stdout:
            print("[GST]", line.rstrip())
    except Exception:
        pass

def stop_video_stream_locked():
    """停止当前 GStreamer 管线，并清理状态。"""
    global video_proc, current_target_ip

    if video_proc is None:
        debug_print_video_state(" stop (no proc)")
        current_target_ip = None
        return

    try:
        print("[VID] 停止 GStreamer 推流...")
        try:
            pgid = os.getpgid(video_proc.pid)
            os.killpg(pgid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        except Exception as e:
            print(f"[VID][WARN] SIGTERM 进程组失败: {e}")

        try:
            video_proc.wait(timeout=1.0)
        except subprocess.TimeoutExpired:
            print("[VID][WARN] GStreamer 1 秒内未退出，发送 SIGKILL...")
            try:
                pgid = os.getpgid(video_proc.pid)
                os.killpg(pgid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            except Exception as e:
                print(f"[VID][WARN] SIGKILL 进程组失败: {e}")
        except Exception as e:
            print(f"[VID][WARN] wait() 异常: {e}")

    except Exception as e:
        print(f"[VID][ERR] 停止视频管线时出错: {e}")
    finally:
        video_proc = None
        current_target_ip = None
        debug_print_video_state(" stopped")

def stop_video_stream():
    """对外接口：停止视频推流。"""
    with video_lock:
        stop_video_stream_locked()

def start_video_stream(target_ip: str):
    """
    启动 GStreamer 推流。
    注意：根据你当前需求，一旦启动，就不会因为 PC 断开自动停流；
    只会在程序退出时调用 stop_video_stream() 统一杀干净。
    """
    global video_proc, current_target_ip

    with video_lock:
        # 如果设置了 FORCE_TARGET_IP，则优先使用
        if FORCE_TARGET_IP:
            target_ip = FORCE_TARGET_IP

        # 如果之前有进程但是已经死亡，则清理句柄
        if video_proc is not None and video_proc.poll() is not None:
            print("[VID] 检测到已死亡的视频进程，清理句柄。")
            video_proc = None
            current_target_ip = None

        # 如果已经在向同一个 IP 推流且进程存活，则不重复启动
        if (
            video_proc is not None
            and current_target_ip == target_ip
            and video_proc.poll() is None
        ):
            debug_print_video_state(" already-on")
            return

        # 启动前确保旧进程彻底停掉
        stop_video_stream_locked()

        debug_print_video_state(f" start→{target_ip}")
        try:
            video_proc = subprocess.Popen(
                build_gst_cmd(target_ip),
                stdout=(subprocess.PIPE if VERBOSE_GST_LOG else subprocess.DEVNULL),
                stderr=(subprocess.STDOUT if VERBOSE_GST_LOG else subprocess.DEVNULL),
                text=True,
                bufsize=1,
                preexec_fn=os.setsid,  # 新建进程组，方便整体杀死
            )

            if VERBOSE_GST_LOG and video_proc.stdout:
                threading.Thread(
                    target=_drain_gst_logs,
                    args=(video_proc,),
                    daemon=True
                ).start()

            current_target_ip = target_ip
            print(f"[VID] GStreamer 推流到 {target_ip}:{PC_VIDEO_PORT}（payload={RTP_PT}）")
            debug_print_video_state(" started")

        except Exception as e:
            print(f"[VID][ERR] 启动 GStreamer 失败：{e}")
            video_proc = None
            current_target_ip = None
            debug_print_video_state(" start-failed")

def on_state_change_maybe_toggle_stream():
    """
    状态变更后的钩子。
    现在的策略：一旦满足条件就尝试“启动”推流，但不在断开时自动停止。
    只有在程序退出时统一 stop_video_stream()。
    """
    # 条件：串口已就绪，并且有 FORCE_TARGET_IP 或至少有一个客户端
    with video_lock:
        need_start = serial_ready.is_set() and (bool(FORCE_TARGET_IP) or bool(active_clients))
        debug_print_video_state(f" state-change need_start={need_start}")
        if not need_start:
            # 不再自动关闭推流，只是记录状态
            return

        target_ip = FORCE_TARGET_IP or list(active_clients)[-1]

    # 在锁外真正启动（避免死锁）
    start_video_stream(target_ip)

# ===================== 串口管理（开关） =====================
def pick_serial_port():
    if not SERIAL_PORT:
        raise RuntimeError("SERIAL_PORT 未配置，请指定要连接的串口路径。")
    return SERIAL_PORT if os.path.exists(SERIAL_PORT) else None

def serial_manager():
    global ser
    next_wait_log = 0.0
    while running:
        try:
            port = pick_serial_port()
            if not port:
                if serial_ready.is_set():
                    serial_ready.clear()
                    print(f"[SER] 指定串口 {SERIAL_PORT} 不存在，标记未就绪。")
                    on_state_change_maybe_toggle_stream()
                now = time.time()
                if now >= next_wait_log:
                    print(f"[SER] 等待指定串口出现：{SERIAL_PORT}")
                    next_wait_log = now + 2.0
                time.sleep(0.3)
                continue
            if ser is not None:
                time.sleep(0.3)
                continue

            print(f"[SER] 打开串口：{port} @ {BAUD_RATE}")
            s = serial.Serial(
                port=port, baudrate=BAUD_RATE,
                parity=serial.PARITY_EVEN, stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS, timeout=SER_TIMEOUT, write_timeout=0.05
            )

            try:
                # 先用 False（很多板子 DTR/RTS 翻转会触发复位/进入下载）
                s.setDTR(False)
                s.setRTS(False)
                time.sleep(0.05)
                s.reset_input_buffer()
                s.reset_output_buffer()
            except Exception as _e:
                print(f"[SER][WARN] setDTR/RTS/reset_buffer failed: {_e}")

            with serial_lock:
                ser = s
                serial_ready.set()
            print("[SER] 串口已就绪。")
            on_state_change_maybe_toggle_stream()

            while running and ser is s:
                if not s.is_open:
                    break
                if not os.path.exists(port):
                    print(f"[SER] 指定串口消失：{port}")
                    break
                time.sleep(0.3)
        except Exception as e:
            print(f"[SER] 管理异常：{e}")
            traceback.print_exc()
        finally:
            mark_serial_disconnected("串口管理线程检测到连接不可用")

# ===================== 高优先级：串口写线程（只写不读） =====================
def serial_writer():
    serial_writer_started.set()
    while running:
        if not (serial_ready.is_set() and ser is not None):
            time.sleep(0.001)
            continue
        try:
            data = pc_send_queue.get(timeout=0.02)
        except queue.Empty:
            continue
        try:
            with serial_lock:
                sent = 0
                mv = memoryview(data)
                L = len(data)
                while sent < L:
                    n = ser.write(mv[sent:])
                    if not n:
                        # time.sleep(0.001)
                        continue
                    sent += n
            if LOG_SERIAL_TX:
                print("[TX-SER]", hexs(data))
        except serial.SerialTimeoutException:
            print("[SER][WARN] 写串口超时。")
        except serial.SerialException as e:
            print(f"[SER] 写串口失败：{e}")
            mark_serial_disconnected(f"写串口失败：{e}")
            time.sleep(0.001)
        except Exception as e:
            print(f"[SER] 写串口异常：{e}")
            time.sleep(0.001)

# ===================== TCP 桥（每连接一个线程） =====================
def handle_connection(conn, addr):
    pc_ip = addr[0]
    print(f"[NET] PC 已连接：{addr}")
    try:
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    except Exception:
        pass

    active_clients.add(pc_ip)
    on_state_change_maybe_toggle_stream()

    try:
        with serial_lock:
            if ser is not None and serial_ready.is_set():
                # 可选调试：观察积压量
                # print("[DBG] in_waiting(before) =", ser.in_waiting)
                ser.reset_input_buffer()
                # print("[DBG] in_waiting(after)  =", ser.in_waiting)
        print("[SER] 已在 PC 连接时清空串口输入缓冲。")
    except Exception as e:
        print(f"[SER][WARN] 清空串口输入缓冲失败：{e}")

    stop_evt = threading.Event()

    def pc_to_queue():
        stream_buf = bytearray()
        while not stop_evt.is_set():
            try:
                data = conn.recv(4096)
                if not data:
                    break
                stream_buf.extend(data)

                frames, new_buf = extract_frames_from_stream(stream_buf, "PC")
                stream_buf = new_buf
                for frm in frames:
                    if LOG_PC_RX:
                        print("[RX-PC]", hexs(frm))
                    while True:
                        try:
                            pc_send_queue.put_nowait(frm)
                            break
                        except queue.Full:
                            try:
                                pc_send_queue.get_nowait()
                            except queue.Empty:
                                break

            except (ConnectionResetError, BrokenPipeError):
                break
            except Exception as e:
                print(f"[NET] PC->Queue 异常：{e}")
                break

    def serial_to_pc():
        read_buf = bytearray(4096)
        stream_buf = bytearray()
        while not stop_evt.is_set():
            if not (serial_ready.is_set() and ser is not None):
                stream_buf.clear()
                time.sleep(0.001)
                continue
            try:
                with serial_lock:
                    n = ser.readinto(read_buf)
                if n and n > 0:
                    stream_buf.extend(memoryview(read_buf)[:n])
                    frames, new_buf = extract_frames_from_stream(
                        stream_buf,
                        "SER",
                    )
                    stream_buf = new_buf
                    for frm in frames:
                        if LOG_SERIAL_RX:
                            print("[RX-SER]", hexs(frm))
                        conn.sendall(frm)
                        if LOG_PC_TX:
                            print("[TX-PC]", hexs(frm))
                else:
                    time.sleep(0.001)
            except (ConnectionResetError, BrokenPipeError):
                break
            except serial.SerialException as e:
                print(f"[SER] 读串口失败：{e}")
                mark_serial_disconnected(f"读串口失败：{e}")
                time.sleep(0.001)
            except Exception as e:
                print(f"[NET] SER->PC 异常：{e}")
                time.sleep(0.001)

    t1 = threading.Thread(target=pc_to_queue, daemon=True)
    t2 = threading.Thread(target=serial_to_pc, daemon=True)
    t1.start()
    t2.start()

    try:
        while t1.is_alive() and t2.is_alive() and running:
            time.sleep(0.001)
    finally:
        stop_evt.set()
        try:
            conn.shutdown(socket.SHUT_RDWR)
        except Exception:
            pass
        try:
            conn.close()
        except Exception:
            pass
        active_clients.discard(pc_ip)
        print(f"[NET] PC 断开：{addr}，现有客户端：{len(active_clients)}")
        on_state_change_maybe_toggle_stream()

def serial_bridge_server():
    print(f"[BOOT] 串口桥监听 {TCP_IP}:{TCP_PORT}")
    if not serial_writer_started.is_set():
        threading.Thread(target=serial_writer, daemon=True).start()

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((TCP_IP, TCP_PORT))
        s.listen(3)
        s.settimeout(0.5)

        while running:
            try:
                conn, addr = s.accept()
            except socket.timeout:
                continue
            except KeyboardInterrupt:
                break
            except OSError:
                # 很可能是正在关闭
                if not running:
                    break
                time.sleep(0.2)
                continue
            except Exception as e:
                print(f"[NET] accept 失败：{e}")
                time.sleep(0.2)
                continue

            threading.Thread(target=handle_connection, args=(conn, addr), daemon=True).start()

# ===================== 主入口 =====================
def main():
    ensure_single_instance()

    # 启动串口管理线程
    threading.Thread(target=serial_manager, daemon=True).start()

    # 提示：如果 FORCE_TARGET_IP 已配置，这里可以立刻尝试启动推流
    if FORCE_TARGET_IP:
        print(f"[BOOT] FORCE_TARGET_IP={FORCE_TARGET_IP}，尝试立即启动推流。")
        start_video_stream(FORCE_TARGET_IP)
    else:
        print("[BOOT] 未设置 FORCE_TARGET_IP，将在串口就绪且有上位机连接时启动推流。")

    try:
        serial_bridge_server()
    except KeyboardInterrupt:
        pass
    finally:
        global running
        running = False

        # 确保 GStreamer 管线被杀干净
        print("[EXIT] 停止视频推流...")
        stop_video_stream()

        # 关闭串口
        with serial_lock:
            if ser:
                try:
                    ser.close()
                except Exception:
                    pass

        print("[EXIT] 已退出。")

if __name__ == "__main__":
    main()
