from fastapi import FastAPI
from modbus_client import write_holding_register, read_holding_register, write_coil, read_coil, reset_errors

app = FastAPI()

@app.post("/CellN/{cell_number}", description="Выбор номера ячейки с которой будет происходить работа")
def cell_n(cell_number: int):
    res = write_holding_register(100, cell_number)
    if res.isError():
        return {"status": 0, "message": "Ошибка записи"}
    return {"status": 1, "message": f"CellN={cell_number} успешно записано"}

@app.post("/SocketN/{socket_number}", description="Выбор Номера порта подключения к анализатору")
def socket_n(socket_number: int):
    res = write_holding_register(101, socket_number)
    if res.isError():
        return {"status": 0, "message": "Ошибка записи"}
    return {"status": 1, "message": f"SocketN={socket_number} успешно записано"}

@app.post("/SetCell/{value}", description="Запуск движения переноса пробы от магазина проб к анализатору")
def set_execution(value: int):
    res = write_coil(100, bool(value))
    if res.isError():
        return {"status": 0}
    return {"status": 1}

@app.post("/UnsetCell/{value}", description="Запуск движения переноса пробы от анализатора к магазину проб")
def set_execution(value: int):
    res = write_coil(101, bool(value))
    if res.isError():
        return {"status": 0}
    return {"status": 1}

@app.get("/TaskStatus", description="Текущее состояние раюоты робота: 0 - робот в базовой точке; 1 - робот переносит ячейку от магазина проб к анализатору;  2 - робот переносит ячейку от анализатора к магазину проб")
def task_status():
    value = read_holding_register(102)
    return {"TaskStatus": value}

@app.get("/RobotWarning", description="Ошибка робота не требующегая вмещательства программистов")
def robot_warning():
    val = read_coil(103)
    return {"warning": val} if val is not None else {"warning": "error"}

@app.get("/RobotError", description="Ошибка робота требующая вмещательства программистов")
def robot_error():
    val = read_coil(104)
    return {"error": val} if val is not None else {"error": "error"}

@app.post("/ResetErrors", description="Сбросить все ошибки роботов")
def reset_errs():
    success = reset_errors()
    return {"status": 1 if success else 0}
