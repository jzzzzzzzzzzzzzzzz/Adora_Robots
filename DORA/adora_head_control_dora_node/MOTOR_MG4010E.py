import serial
import time
import struct

class MOTOR_MG4010E: 
    def __init__(self):
        self.ser = None  # 串口对象
        self.serial_opened = False  # 串口状态标志

        self.init_serial("/dev/ttyACM0",19200)
        self.motor_init()

 
        # 配置串口参数
    def init_serial(self, port, baudrate=115200):
        try:
                # 初始化串口，设置串口参数
            self.ser = serial.Serial(
                port=port,  # 串口名称
                baudrate=baudrate,  # 波特率
                bytesize=serial.EIGHTBITS,  # 数据位8位
                parity=serial.PARITY_NONE,  # 校验位，无校验位
                stopbits=serial.STOPBITS_ONE,  # 停止位1位
                timeout=0.1  # 超时时间
        )
        except Exception as e:
            print("串口初始化失败:", e)

    def run(self):
 
        #while True:
 
        self.motor_position_read()
        #time.sleep(0.1)

        uart_buffer_data = self.ser.read_all()  

        if(len(uart_buffer_data) < 7):
            print("uart_buffer_data < 7")
            return
        
        tem_bytes = bytearray()
        tem_bytes.extend(uart_buffer_data[0:7]) #高位存低地址
        tem_bytes_str = ' '.join(f"{b:02X}" for b in tem_bytes)
        print(f"HEtem_bytes_strX: [{tem_bytes_str}]")

        recives_crc = self.calculate_crc16(tem_bytes)
        if recives_crc:
            # 打印十六进制和ASCII格式
            hex_str = ' '.join(f"{b:02X}" for b in uart_buffer_data)
            crc_str = ' '.join(f"{b:02X}" for b in recives_crc)
            ascii_str = ''.join(chr(b) if 32 <= b <= 126 else '.' for b in recives_crc)
            print(f"HEX: [{hex_str}] CRC: [{crc_str}]  ASCII: [{ascii_str}]")
        #print("daying:")


        if(uart_buffer_data[0] == 0x01  
            and uart_buffer_data[1] == 0x03 
            and uart_buffer_data[2] == 0x04):

            position_bytes = bytearray()
            position_bytes.extend([uart_buffer_data[5], uart_buffer_data[6],uart_buffer_data[3], uart_buffer_data[4]]) #高位存低地址
            self.motor_positon_read = int.from_bytes(position_bytes,'big',signed = True)
            print("position:", self.motor_positon_read)


    def motor_init(self):
        data_array_1 = bytearray()
        data_array_1.extend([0x01, 0x06, 0x00, 0x00, 0x00, 0x01, 0x48, 0x0A])  # enable modbus

        data_array_2 = bytearray()
        data_array_2.extend([0x01, 0x06, 0x00, 0x01, 0x00, 0x01, 0x48, 0xCA])  # enable motor

        data_array_3 = bytearray()
        data_array_3.extend([0x01, 0x06, 0x00, 0x03, 0x13, 0x88, 0x74, 0x9C])  # accel speed 5000

        data_array_4 = bytearray()
        data_array_4.extend([0x01, 0x06, 0x00, 0x02, 0x05, 0xDC, 0x2A, 0xC3])  # speed 1500
        
        if not self.ser.is_open:
            self.ser.open()
            
        # 发送数据
        self.ser.write(data_array_1)
        time.sleep(0.2)
        self.ser.write(data_array_2)
        time.sleep(0.2)
        self.ser.write(data_array_3)
        time.sleep(0.2)
        self.ser.write(data_array_4)
        time.sleep(0.2)

    def calculate_crc(self,data: bytes) -> bytes:
        crc_value = 0x00  #初始化
        for byte in data:
            # 计算索引（修正原代码中的 *puchMsgg++ 错误）
            crc_value = crc_value + byte
        # 返回组合后的CRC值
        return bytes([crc_value & 0xff])
    

    def motor_mg4010e_read_state1(self,motor_id):
        data_array_1 = bytearray()
        data_array_1.extend([0x3E, 0x9A, 0x00, 0x00])  # read motor global position
        data_array_1[2] = motor_id
        crc_values = self.calculate_crc(data_array_1) # calculate crc
        data_array_1.extend([crc_values[0]])
        if not self.ser.is_open:
            self.ser.open()
        # 发送数据
        self.ser.write(data_array_1)

    def motor_mg4010e_clear_error_code(self,motor_id):
        data_array_1 = bytearray()
        data_array_1.extend([0x3E, 0x9B, 0x00])  # read motor global position
        data_array_1[2] = motor_id
        crc_values = self.calculate_crc(data_array_1) # calculate crc
        data_array_1.extend([crc_values[0]])
        if not self.ser.is_open:
            self.ser.open()
        # 发送数据
        self.ser.write(data_array_1)
     
    def motor_mg4010e_read_state2(self,motor_id):
        data_array_1 = bytearray()
        data_array_1.extend([0x3E, 0x9C, 0x00, 0x00])  # read motor global position
        data_array_1[2] = motor_id
        crc_values = self.calculate_crc(data_array_1) # calculate crc
        data_array_1.extend([crc_values[0]])
        if not self.ser.is_open:
            self.ser.open()
        # 发送数据
        self.ser.write(data_array_1)

    def motor_mg4010e_read_state3(self,motor_id):
        data_array_1 = bytearray()
        data_array_1.extend([0x3E, 0x9D, 0x00, 0x00])  # read motor global position
        data_array_1[2] = motor_id
        crc_values = self.calculate_crc(data_array_1) # calculate crc
        data_array_1.extend([crc_values[0]])
        if not self.ser.is_open:
            self.ser.open()
        # 发送数据
        self.ser.write(data_array_1)

    def motor_mg4010e_close(self,motor_id):
        data_array_1 = bytearray()
        data_array_1.extend([0x3E, 0x80, 0x00, 0x00])  # read motor global position
        data_array_1[2] = motor_id
        crc_values = self.calculate_crc(data_array_1) # calculate crc
        data_array_1.extend([crc_values[0]])
        if not self.ser.is_open:
            self.ser.open()
        # 发送数据
        self.ser.write(data_array_1)

    def motor_mg4010e_start(self,motor_id):
        data_array_1 = bytearray()
        data_array_1.extend([0x3E, 0x88, 0x00, 0x00])  # read motor global position
        data_array_1[2] = motor_id
        crc_values = self.calculate_crc(data_array_1) # calculate crc
        data_array_1.extend([crc_values[0]])
        if not self.ser.is_open:
            self.ser.open()
        # 发送数据
        self.ser.write(data_array_1)

    def motor_mg4010e_stop(self,motor_id):
        data_array_1 = bytearray()
        data_array_1.extend([0x3E, 0x81, 0x00, 0x00])  # read motor global position
        data_array_1[2] = motor_id
        crc_values = self.calculate_crc(data_array_1) # calculate crc
        data_array_1.extend([crc_values[0]])
        if not self.ser.is_open:
            self.ser.open()
        # 发送数据
        self.ser.write(data_array_1)

    # 控制抱闸器的开合，或者读取当前抱闸器的状态。
    def motor_mg4010e_set_brake(self,motor_id):
        data_array_1 = bytearray()
        data_array_1.extend([0x3E, 0x81, 0x00, 0x00])  # read motor global position
        data_array_1[2] = motor_id
        crc_values = self.calculate_crc(data_array_1) # calculate crc
        data_array_1.extend([crc_values[0]])
        if not self.ser.is_open:
            self.ser.open()
        # 发送数据
        self.ser.write(data_array_1)


    # 主机发送该命令以控制电机的位置（多圈角度）， 控制值 angleControl 为 int64_t 类型，
    # 对应实际位置为 0.01degree/LSB，即 36000 代表 360°，
    # 电机转动方向由目标位置和当前位置的差值决定
    def motor_mg4010e_set_position(self,motor_id,angleControl):
        data_array_1 = bytearray()
        data_array_1.extend([0x3E, 0x81, 0x00, 0x00])  # read motor global position
        data_array_1[2] = motor_id
        crc_values = self.calculate_crc(data_array_1) # calculate crc
        data_array_1.extend([crc_values[0]])
        data_array_1.extend(self.int64_to_bytearray_with_checksum(angleControl))
        if not self.ser.is_open:
            self.ser.open()
        # 发送数据
        self.ser.write(data_array_1)

    def int64_to_bytearray_with_checksum(num):
        """
        - 将int64_t整数转换为9字节数组:
        - 前8字节:int64_t的原始字节 小端序
        - 第9字节 前8字节的校验和 求和后取低8位 
        
        参数:
            num: int64_t 整数
        返回:
            bytearray 对象,长度9
        """
        # Step 1: 将int64_t打包为8字节（小端序）[6,7](@ref)
        packed_bytes = struct.pack('<q', num)
        
        # Step 2: 创建长度为9的bytearray，前8字节填充数据
        data_array = bytearray(packed_bytes)  # 前8字节
        
        # Step 3: 计算前8字节的校验和（求和后取低8位）[1](@ref)
        checksum = sum(data_array) & 0xFF
        data_array.append(checksum)  # 第9字节填充校验和
        
        return data_array
    
 



if __name__ == "__main__":
    app = MOTOR_MG4010E()
    app.run()
    while True:
        app.motor_mg4010e_set_position(200*100)
        time.sleep(20)   
        app.motor_mg4010e_set_position(0)
        time.sleep(20)  
