import os
from pymodbus.client.sync import ModbusTcpClient
from dotenv import load_dotenv

load_dotenv()

MODBUS_SERVER_IP = os.getenv("MODBUS_SERVER_IP", "127.0.0.1")
MODBUS_SERVER_PORT = int(os.getenv("MODBUS_SERVER_PORT", "502"))
MODBUS_UNIT_ID = int(os.getenv("MODBUS_UNIT_ID", "1"))

client = ModbusTcpClient(MODBUS_SERVER_IP, port=MODBUS_SERVER_PORT)

def connect_client():
    if not client.is_socket_open():
        client.connect()

def write_holding_register(register, value):
    connect_client()
    print("Holding register. Регистр: " + str(register) + ", значение: " + str(value))
    return client.write_register(register, value, unit=MODBUS_UNIT_ID)

def write_coil(register, value):
    connect_client()
    print("Write coil. Регистр: " + str(register) + ", значение: " + str(value))
    return client.write_coil(register, value, unit=MODBUS_UNIT_ID)

def read_holding_register(register):
    connect_client()
    result = client.read_holding_registers(register, 1, slave=MODBUS_UNIT_ID)
    if result.isError():
        return None
    return result.registers[0]

def read_coil(register):
    connect_client()
    result = client.read_coils(register, 1, unit=MODBUS_UNIT_ID)
    if result.isError():
        return None
    return result.bits[0]

def reset_errors():
    connect_client()
    res1 = client.write_coil(2, 0, unit=MODBUS_UNIT_ID)  # RobotWarning reset
    res2 = client.write_coil(3, 0, unit=MODBUS_UNIT_ID)  # RobotError reset
    return not (res1.isError() or res2.isError())
