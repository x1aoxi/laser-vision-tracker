import serial
import threading
import queue
import select
import configparser
import time

# 创建队列用于线程间通信
send_queue = queue.Queue()
receive_queue = queue.Queue()


def configure_serial_port():
    """配置并打开串口"""
    uart_config = configparser.ConfigParser()
    uart_config.read("./uart.ini")
    ser = serial.Serial(
        port=uart_config["com1"]["port"],
        baudrate=int(uart_config["com1"]["baudrate"]),
        bytesize=int(uart_config["com1"]["bytesize"]),    # 8 数据位
        parity=uart_config["com1"]["parity"],     # 奇校验
        stopbits=int(uart_config["com1"]["stopbits"]),  # 1 停止位
        timeout=float(uart_config["com1"]["timeout"])                # 读取超时时间（秒）
    )

    if ser.is_open:
        print(f"串口 {ser.port} 已打开，波特率: {ser.baudrate}")
        return ser

def send_data(ser, w_data_list=None):
    """发送数据到串口"""
    data_len = 0x0
    if w_data_list is not None:
        data_len = len(w_data_list)

    bytes_e = b''

    if data_len:
        for w_d in w_data_list:
            bytes_e = bytes_e + w_d.to_bytes(1, "big")

    w_frame = bytes_e 

    ser.write(w_frame)
    # print(f"发送数据: {w_frame}")

def receive_data(ser, buffer_size=1024):
    """从串口接收数据"""
    rec_len = ser.inWaiting()
    r_frame = b''
    while rec_len > 0:
        r_frame += ser.read(rec_len)
        time.sleep(0.01)
        rec_len = ser.inWaiting()
        
    return r_frame

# 接收线程函数
def receive_thread(ser):
    fd = ser.fileno()
    while True:
        readable, _, _ = select.select([fd], [], [], 1.0)
        if fd in readable:
            r_frame = receive_data(ser)
            receive_queue.put(r_frame)
            # print(f"接收数据:{r_frame}")

# 发送线程函数
def send_thread(ser):
    while True:
        w_data = send_queue.get(block=True)
        send_data(ser, w_data)

def uart_send(data):
    send_queue.put(data)

def uart_receive():
    try:
        r_data = receive_queue.get(timeout=0.2)
    except queue.Empty:
        r_data = b''

    return r_data

def uart_init():
    ser = configure_serial_port()
    if not ser:
        return

    # 启动线程
    receive_t = threading.Thread(target=receive_thread, args=(ser,), daemon=True)
    send_t = threading.Thread(target=send_thread, args=(ser,), daemon=True)
    receive_t.start()
    send_t.start()

