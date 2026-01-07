"""
EtherSpin-ESP: Python UDPクライアントサンプル

PC側から速度プロファイルをリアルタイムでストリーミングして
ステッピングモータを制御するサンプルコード
"""

import socket
import json
import time
import math
import sys

# ESP32の設定
ESP32_IP = "motor.local"  # または実際のIPアドレス (例: "192.168.1.100")
UDP_PORT = 8888

class MotorController:
    """モータコントローラクラス"""
    
    def __init__(self, ip=ESP32_IP, port=UDP_PORT):
        self.ip = ip
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        print(f"[MotorController] Initialized: {ip}:{port}")
    
    def set_speed(self, speed):
        """
        速度を設定（step/s）
        
        Args:
            speed (float): 速度 (step/s)
                         正: 正回転、負: 逆回転、0: 停止
        """
        message = json.dumps({"v": float(speed)})
        self.sock.sendto(message.encode(), (self.ip, self.port))
    
    def stop(self):
        """モータを停止"""
        self.set_speed(0)
    
    def close(self):
        """ソケットを閉じる"""
        self.sock.close()


def example_constant_speed():
    """例1: 定速回転"""
    print("\n=== Example 1: Constant Speed ===")
    controller = MotorController()
    
    try:
        print("500 step/s で5秒間回転")
        controller.set_speed(500)
        time.sleep(5)
        
        print("停止")
        controller.stop()
        time.sleep(1)
        
    finally:
        controller.close()


def example_acceleration():
    """例2: 加速・減速プロファイル"""
    print("\n=== Example 2: Acceleration Profile ===")
    controller = MotorController()
    
    try:
        duration = 10  # 秒
        max_speed = 2000  # step/s
        dt = 0.05  # 更新間隔（秒）
        
        print(f"0 → {max_speed} step/s まで加速・減速")
        
        t = 0
        while t < duration:
            # 台形加速プロファイル
            if t < duration / 3:
                # 加速
                speed = max_speed * (t / (duration / 3))
            elif t < 2 * duration / 3:
                # 定速
                speed = max_speed
            else:
                # 減速
                speed = max_speed * (1 - (t - 2 * duration / 3) / (duration / 3))
            
            controller.set_speed(speed)
            print(f"Time: {t:.2f}s, Speed: {speed:.1f} step/s")
            
            time.sleep(dt)
            t += dt
        
        controller.stop()
        
    finally:
        controller.close()


def example_sine_wave():
    """例3: サイン波速度プロファイル"""
    print("\n=== Example 3: Sine Wave Profile ===")
    controller = MotorController()
    
    try:
        duration = 20  # 秒
        amplitude = 1500  # step/s
        frequency = 0.2  # Hz
        dt = 0.02  # 更新間隔（秒）
        
        print(f"サイン波で振動（振幅: {amplitude} step/s, 周波数: {frequency} Hz）")
        
        t = 0
        while t < duration:
            speed = amplitude * math.sin(2 * math.pi * frequency * t)
            controller.set_speed(speed)
            
            if int(t * 10) % 10 == 0:  # 1秒ごとに表示
                print(f"Time: {t:.1f}s, Speed: {speed:.1f} step/s")
            
            time.sleep(dt)
            t += dt
        
        controller.stop()
        
    finally:
        controller.close()


def example_s_curve():
    """例4: S字カーブ加速"""
    print("\n=== Example 4: S-Curve Acceleration ===")
    controller = MotorController()
    
    try:
        duration = 8  # 秒
        max_speed = 2500  # step/s
        dt = 0.05  # 更新間隔（秒）
        
        print(f"S字カーブで滑らかに加速・停止")
        
        t = 0
        while t < duration:
            # S字カーブ（シグモイド関数）
            x = (t / duration - 0.5) * 10
            sigmoid = 1 / (1 + math.exp(-x))
            speed = max_speed * sigmoid
            
            controller.set_speed(speed)
            print(f"Time: {t:.2f}s, Speed: {speed:.1f} step/s")
            
            time.sleep(dt)
            t += dt
        
        controller.stop()
        
    finally:
        controller.close()


def example_interactive():
    """例5: インタラクティブ制御"""
    print("\n=== Example 5: Interactive Control ===")
    print("キーボードで速度を制御します")
    print("  w: 加速")
    print("  s: 減速")
    print("  d: 右回転方向")
    print("  a: 左回転方向")
    print("  space: 停止")
    print("  q: 終了")
    
    controller = MotorController()
    
    try:
        speed = 0
        step = 100
        
        # 注: この例はLinux/Mac環境では動作します
        # Windowsではmsvcrtモジュールを使用する必要があります
        import tty
        import termios
        import sys
        
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            
            while True:
                char = sys.stdin.read(1)
                
                if char == 'w':
                    speed += step
                elif char == 's':
                    speed -= step
                elif char == 'd':
                    speed = abs(speed)
                elif char == 'a':
                    speed = -abs(speed)
                elif char == ' ':
                    speed = 0
                elif char == 'q':
                    break
                
                # 速度制限
                speed = max(-3000, min(3000, speed))
                
                controller.set_speed(speed)
                print(f"\rCurrent speed: {speed:6.1f} step/s", end='', flush=True)
        
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            print()
            controller.stop()
    
    except ImportError:
        print("対話モードはLinux/Macでのみサポートされています")
    
    finally:
        controller.close()


def main():
    """メイン関数"""
    print("=" * 50)
    print("EtherSpin-ESP: Python UDP Client Sample")
    print("=" * 50)
    
    if len(sys.argv) > 1:
        example = sys.argv[1]
        
        if example == "1":
            example_constant_speed()
        elif example == "2":
            example_acceleration()
        elif example == "3":
            example_sine_wave()
        elif example == "4":
            example_s_curve()
        elif example == "5":
            example_interactive()
        else:
            print(f"Unknown example: {example}")
    else:
        print("\n使用方法:")
        print("  python udp_client.py [example_number]")
        print("\n利用可能な例:")
        print("  1: 定速回転")
        print("  2: 台形加速プロファイル")
        print("  3: サイン波速度プロファイル")
        print("  4: S字カーブ加速")
        print("  5: インタラクティブ制御")
        print("\n例:")
        print("  python udp_client.py 1")
        print()


if __name__ == "__main__":
    main()
