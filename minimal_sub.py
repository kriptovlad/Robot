import time
import sys
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_ # Используем тот же тип данных

# --- НАСТРОЙКИ ---
# УБЕДИТЕСЬ, ЧТО ОНИ СОВПАДАЮТ С ВАШИМ config.py / demo.py
DOMAIN_ID = 1
INTERFACE = "Беспроводная сеть" # <-- Укажите тот же интерфейс, что и в config.py
TOPIC_LOWCMD = "rt/lowcmd"      # <-- Тот же топик, что и в demo/bridge
# --- ---

received_count = 0

# Функция обратного вызова, которая будет вызываться при получении сообщения
def MessageHandler(msg: LowCmd_):
    global received_count
    received_count += 1
    # Считываем данные из сообщения для проверки
    head0 = msg.head[0]
    head1 = msg.head[1]
    level_flag = msg.level_flag
    motor_q0 = msg.motor_cmd[0].q # Читаем счетчик
    print(f"---> [Subscriber] Получено сообщение #{received_count}! flag={hex(level_flag)}, q0={motor_q0:.1f}")

print(f"[Subscriber] Инициализация DDS: Domain ID={DOMAIN_ID}, Interface='{INTERFACE}'")
try:
    # Инициализация фабрики DDS (один раз на процесс)
    ChannelFactoryInitialize(DOMAIN_ID, INTERFACE)
    print("[Subscriber] DDS инициализирован.")

    # Создание подписчика
    sub = ChannelSubscriber(TOPIC_LOWCMD, LowCmd_)
    # Передаем нашу функцию-обработчик MessageHandler
    if not sub.Init(MessageHandler, 10): # Размер буфера 10, как в bridge
         print("[Subscriber] Ошибка инициализации подписчика.")
         # sys.exit(1)
    print(f"[Subscriber] Подписчик для топика '{TOPIC_LOWCMD}' создан. Ожидание сообщений...")

    # Просто поддерживаем работу скрипта
    while True:
        time.sleep(5) # Просто ждем, обработка происходит в отдельном потоке DDS

except Exception as e:
    print(f"[Subscriber] Ошибка: {e}")
except KeyboardInterrupt:
    print("\n[Subscriber] Завершение работы.")