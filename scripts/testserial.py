import serial
import time

def test_serial_port(port='/dev/ttyACM0', baudrate=921600, timeout=0.1):
    """简化版串口测试：只验证是否能打开串口，不依赖复杂配置"""
    ser = None
    try:
        # 简化配置：只传核心参数，避免版本兼容问题
        ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=timeout  # 仅保留超时参数，其他用默认值
        )

        # 检查串口是否成功打开
        if ser.is_open:
            print(f"✅ 成功打开串口: {port}")
            print(f"📌 当前波特率: {baudrate}")
            print("💡 串口状态正常，可进行后续通信测试")
            return True
        else:
            print(f"❌ 串口已创建但未打开: {port}")
            return False

    except serial.SerialException as e:
        # 捕获串口相关错误（如权限、设备不存在、波特率不支持）
        print(f"❌ 串口错误: {str(e)}")
        print("💡 可能原因：1.设备路径错误 2.权限不足 3.波特率不支持 4.串口被占用")
        return False
    except Exception as e:
        # 捕获其他未知错误
        print(f"❌ 其他错误: {str(e)}")
        return False
    finally:
        # 确保程序退出时关闭串口，避免占用
        if ser and ser.is_open:
            ser.close()
            print("🔌 程序退出，已关闭串口")

if __name__ == "__main__":
    # 你的串口配置（根据实际情况修改）
    TEST_PORT = '/dev/ttyACM0'    # 你的串口设备路径
    TEST_BAUDRATE = 921600        # 你的波特率（与电机驱动一致）
    # 执行测试
    test_serial_port(TEST_PORT, TEST_BAUDRATE)