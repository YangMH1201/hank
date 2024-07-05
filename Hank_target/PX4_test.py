from pymavlink import mavutil
import time

def connect_pixhawk(port="/dev/ttyUSB0", baudrate=921600):
    # 建立與Pixhawk的連接
    master = mavutil.mavlink_connection(port, baud=baudrate)
    print(f"連接到Pixhawk在 {port} 以 {baudrate} 波特率")

    # 等待心跳以確認連接
    print("等待心跳...")
    master.wait_heartbeat()
    print("心跳接收，Pixhawk連接正常！")

    # 嘗試讀取一些基本信息
    try:
        while True:
            # 獲取消息
            msg = master.recv_match(blocking=True)
            if msg:
                print(f"接收到消息: {msg}")
                break  # 獲取到第一條消息後退出循環
    except KeyboardInterrupt:
        print("程序被用戶中斷")

if __name__ == "__main__":
    connect_pixhawk()
