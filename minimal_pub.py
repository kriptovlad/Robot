import time
import sys
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_  # Используем тот же тип данных
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_  # Используем конструктор по умолчанию

# --- НАСТРОЙКИ ---
# УБЕДИТЕСЬ, ЧТО ОНИ СОВПАДАЮТ С ВАШИМ config.py / demo.py
DOMAIN_ID = 1
INTERFACE = "Беспроводная сеть"  # <-- Укажите тот же интерфейс, что и в config.py
TOPIC_LOWCMD = "rt/lowcmd"  # <-- Тот же топик, что и в demo/bridge
# --- ---

print(f"[Publisher] Инициализация DDS: Domain ID={DOMAIN_ID}, Interface='{INTERFACE}'")
try:
    # Инициализация фабрики DDS (один раз на процесс)
    ChannelFactoryInitialize(DOMAIN_ID, INTERFACE)
    print("[Publisher] DDS инициализирован.")

    # Создание издателя
    pub = ChannelPublisher(TOPIC_LOWCMD, LowCmd_)
    if not pub.Init():
        print("[Publisher] Ошибка инициализации издателя.")
        # sys.exit(1)
    print(f"[Publisher] Издатель для топика '{TOPIC_LOWCMD}' создан.")

    # Создаем пустое сообщение LowCmd_
    cmd = unitree_go_msg_dds__LowCmd_()
    cmd.head[0] = 0xFE  # Стандартные заголовки
    cmd.head[1] = 0xEF
    cmd.level_flag = 0xAA  # Используем другой флаг для отличия

    count = 0
    while True:
        # Записываем счетчик в какое-нибудь поле для проверки
        cmd.motor_cmd[0].q = float(count)

        print(f"[Publisher] Отправка сообщения #{count} (q0={cmd.motor_cmd[0].q:.1f})")
        if not pub.Write(cmd):
            # Эта проверка может не сработать, если нет подписчиков
            print(f"[Publisher] Ошибка отправки сообщения #{count} (возможно, нет подписчиков)")

        count += 1
        time.sleep(1)  # Отправляем раз в секунду

except Exception as e:
    print(f"[Publisher] Ошибка: {e}")
except KeyboardInterrupt:
    print("\n[Publisher] Завершение работы.")