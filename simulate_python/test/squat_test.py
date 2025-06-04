import time
import sys
import threading
import simulate_python.config as config

from unitree_sdk2py.core.channel import ChannelPublisher
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
from unitree_sdk2py.utils.crc import CRC
import numpy as np

# Константы для упрощения работы с углами (в радианах)
kPi = 3.141592654
kPi_2 = 1.57079632


# Класс, определяющий индексы суставов робота G1 для управления
class G1JointIndex:
    # Суставы левой ноги
    LeftHipPitch = 0      # Наклон бедра вперёд-назад
    LeftHipRoll = 1       # Боковой наклон бедра
    LeftHipYaw = 2        # Поворот бедра
    LeftKnee = 3          # Сгибание колена
    LeftAnklePitch = 4    # Наклон лодыжки вперёд-назад
    LeftAnkleRoll = 5     # Боковой наклон лодыжки

    # Суставы правой ноги
    RightHipPitch = 6     # Наклон бедра вперёд-назад
    RightHipRoll = 7      # Боковой наклон бедра
    RightHipYaw = 8       # Поворот бедра
    RightKnee = 9         # Сгибание колена
    RightAnklePitch = 10  # Наклон лодыжки вперёд-назад
    RightAnkleRoll = 11   # Боковой наклон лодыжки

    # Суставы талии
    WaistYaw = 12         # Поворот талии
    WaistRoll = 13        # Боковой наклон талии
    WaistPitch = 14       # Наклон талии вперёд-назад

    # Суставы левой руки
    LeftShoulderPitch = 15  # Наклон плеча вперёд-назад
    LeftShoulderRoll = 16   # Боковой наклон плеча
    LeftShoulderYaw = 17    # Поворот плеча
    LeftElbow = 18          # Сгибание локтя
    LeftWristRoll = 19      # Поворот запястья
    LeftWristPitch = 20     # Наклон запястья вперёд-назад
    LeftWristYaw = 21       # Поворот запястья

    # Суставы правой руки
    RightShoulderPitch = 22  # Наклон плеча вперёд-назад
    RightShoulderRoll = 23   # Боковой наклон плеча
    RightShoulderYaw = 24    # Поворот плеча
    RightElbow = 25          # Сгибание локтя
    RightWristRoll = 26      # Поворот запястья
    RightWristPitch = 27     # Наклон запястья вперёд-назад
    RightWristYaw = 28       # Поворот запястья

    kNotUsedJoint = 29       # Специальный индекс для управления SDK (включение/выключение)


# Основной класс для управления роботом (приседание)
class Custom:
    def __init__(self):
        # Инициализация переменных для управления временем
        self.time_ = 0.0               # Текущее время выполнения программы
        self.control_dt_ = 0.02        # Частота обновления (50 Гц, каждые 0.02 секунды)
        self.duration_ = 2.0           # Длительность каждой стадии движения (в секундах)

        # Параметры управления суставами
        self.kp = 80.0                 # Жёсткость для управления позицией (увеличена для стабильности)
        self.kd = 1.5                  # Коэффициент демпфирования (для плавности движения)

        # Объекты для взаимодействия с SDK
        self.low_cmd = unitree_hg_msg_dds__LowCmd_()  # Команда для управления роботом
        self.low_state = None                        # Состояние робота (будет обновляться)
        self.first_update_low_state = False          # Флаг первого получения состояния
        self.crc = CRC()                             # Объект для вычисления контрольной суммы
        self.done = False                            # Флаг завершения задачи

        # Переменные для управления потоком выполнения
        self._running = False                        # Флаг активности цикла управления
        self.control_thread = None                   # Поток для цикла управления
        self.state_lock = threading.Lock()           # Блокировка для безопасного доступа к состоянию

        # Целевые позиции суставов для глубокого приседания (в радианах)
        self.target_pos_squat = [
            -0.8,  # LeftHipPitch: наклон бедра вперёд для глубокого приседания
            0.0,   # LeftHipRoll: без бокового наклона
            0.0,   # LeftHipYaw: без поворота бедра
            1.4,   # LeftKnee: сильное сгибание колена для приседания
            -0.7,  # LeftAnklePitch: наклон лодыжки для сохранения баланса
            0.0,   # LeftAnkleRoll: без бокового наклона лодыжки
            -0.8,  # RightHipPitch: наклон бедра вперёд для глубокого приседания
            0.0,   # RightHipRoll: без бокового наклона
            0.0,   # RightHipYaw: без поворота бедра
            1.4,   # RightKnee: сильное сгибание колена для приседания
            -0.7,  # RightAnklePitch: наклон лодыжки для сохранения баланса
            0.0    # RightAnkleRoll: без бокового наклона лодыжки
        ]

        # Список суставов ног, участвующих в движении
        self.leg_joints = [
            G1JointIndex.LeftHipPitch,
            G1JointIndex.LeftHipRoll,
            G1JointIndex.LeftHipYaw,
            G1JointIndex.LeftKnee,
            G1JointIndex.LeftAnklePitch,
            G1JointIndex.LeftAnkleRoll,
            G1JointIndex.RightHipPitch,
            G1JointIndex.RightHipRoll,
            G1JointIndex.RightHipYaw,
            G1JointIndex.RightKnee,
            G1JointIndex.RightAnklePitch,
            G1JointIndex.RightAnkleRoll
        ]

        # Проверка на соответствие длины массивов целевых позиций и суставов
        assert len(self.target_pos_squat) == len(self.leg_joints), "Target positions length mismatch with leg joints length"

    # Инициализация каналов связи с роботом через DDS
    def Init(self):
        # Создание издателя для отправки команд роботу
        self.arm_sdk_publisher = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.arm_sdk_publisher.Init()

        # Создание подписчика для получения состояния робота
        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_subscriber.Init(self.LowStateHandler, 10)

        return True

    # Запуск цикла управления роботом
    def Start(self):
        # Ожидание первого сообщения о состоянии робота
        while not self.first_update_low_state:
            time.sleep(0.1)

        # Запуск потока управления, если состояние получено
        if self.first_update_low_state:
            self._running = True
            self.control_thread = threading.Thread(target=self._control_loop, name="control_loop")
            self.control_thread.daemon = True
            self.control_thread.start()

    # Остановка цикла управления
    def Stop(self):
        self._running = False
        if self.control_thread and self.control_thread.is_alive():
            self.control_thread.join(timeout=1.0)

    # Обработчик состояния робота (обновляет self.low_state)
    def LowStateHandler(self, msg: LowState_):
        with self.state_lock:
            self.low_state = msg
            if not self.first_update_low_state:
                self.first_update_low_state = True

    # Основной цикл управления роботом
    def _control_loop(self):
        last_execution_time = time.perf_counter()

        while self._running:
            # Обеспечение равномерного времени между итерациями цикла (50 Гц)
            current_time = time.perf_counter()
            time_since_last = current_time - last_execution_time
            wait_time = self.control_dt_ - time_since_last

            if wait_time > 0:
                time.sleep(wait_time)
            last_execution_time = time.perf_counter()

            # Вызов функции управления суставами
            self.LowCmdWrite()

            # Прерывание цикла, если задача завершена
            if self.done:
                self._running = False

        # Отключение SDK после завершения
        self._ensure_sdk_disabled()

    # Функция управления суставами робота
    def LowCmdWrite(self):
        # Получение текущего состояния робота с защитой от одновременного доступа
        with self.state_lock:
            if self.low_state is None:
                return
            current_low_state = self.low_state

        # Увеличение времени выполнения
        self.time_ += self.control_dt_

        # Логика движения в 4 стадиях:
        # 1. Переход в начальную позу (все суставы в 0) за duration_
        # 2. Приседание до целевых позиций за duration_
        # 3. Возврат в начальную позу за duration_
        # 4. Выключение SDK за duration_

        if self.time_ < self.duration_:
            # Стадия 1: Переход в начальную позу (все суставы в нуле)
            self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = 1.0  # Включение SDK
            ratio = np.clip(self.time_ / self.duration_, 0.0, 1.0)     # Линейная интерполяция

            for joint_index in self.leg_joints:
                current_q = current_low_state.motor_state[joint_index].q  # Текущая позиция сустава
                target_q = 0.0                                           # Целевая позиция (нулевая)
                self.low_cmd.motor_cmd[joint_index].q = (1.0 - ratio) * current_q + ratio * target_q
                self.low_cmd.motor_cmd[joint_index].dq = 0.0             # Скорость = 0
                self.low_cmd.motor_cmd[joint_index].kp = self.kp         # Жёсткость
                self.low_cmd.motor_cmd[joint_index].kd = self.kd         # Демпфирование
                self.low_cmd.motor_cmd[joint_index].tau = 0.0            # Момент = 0

        elif self.time_ < self.duration_ * 2:
            # Стадия 2: Приседание до целевых позиций
            self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = 1.0
            ratio = np.clip((self.time_ - self.duration_) / self.duration_, 0.0, 1.0)

            for i, joint_index in enumerate(self.leg_joints):
                start_q = 0.0  # Начальная позиция (из предыдущей стадии)
                target_q = self.target_pos_squat[i]  # Целевая позиция для приседания
                self.low_cmd.motor_cmd[joint_index].q = (1.0 - ratio) * start_q + ratio * target_q
                self.low_cmd.motor_cmd[joint_index].dq = 0.0
                self.low_cmd.motor_cmd[joint_index].kp = self.kp
                self.low_cmd.motor_cmd[joint_index].kd = self.kd
                self.low_cmd.motor_cmd[joint_index].tau = 0.0

        elif self.time_ < self.duration_ * 3:
            # Стадия 3: Возврат в начальную позу (стоя)
            self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = 1.0
            ratio = np.clip((self.time_ - self.duration_ * 2) / self.duration_, 0.0, 1.0)

            for i, joint_index in enumerate(self.leg_joints):
                start_q = self.target_pos_squat[i]  # Начальная позиция (из приседания)
                target_q = 0.0                     # Целевая позиция (нулевая)
                self.low_cmd.motor_cmd[joint_index].q = (1.0 - ratio) * start_q + ratio * target_q
                self.low_cmd.motor_cmd[joint_index].dq = 0.0
                self.low_cmd.motor_cmd[joint_index].kp = self.kp
                self.low_cmd.motor_cmd[joint_index].kd = self.kd
                self.low_cmd.motor_cmd[joint_index].tau = 0.0

        elif self.time_ < self.duration_ * 4:
            # Стадия 4: Выключение SDK
            ratio = np.clip((self.time_ - self.duration_ * 3) / self.duration_, 0.0, 1.0)
            self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = 1.0 - ratio  # Постепенное выключение

            for joint_index in self.leg_joints:
                self.low_cmd.motor_cmd[joint_index].q = 0.0
                self.low_cmd.motor_cmd[joint_index].dq = 0.0
                self.low_cmd.motor_cmd[joint_index].kp = 10.0  # Уменьшенная жёсткость
                self.low_cmd.motor_cmd[joint_index].kd = 0.5   # Уменьшенное демпфирование
                self.low_cmd.motor_cmd[joint_index].tau = 0.0

        else:
            # Завершение задачи
            if not self.done:
                self.done = True

        # Отправка команды роботу, если задача не завершена или время превысило 4 стадии
        if not self.done or self.time_ >= self.duration_ * 4:
            self.low_cmd.crc = self.crc.Crc(self.low_cmd)  # Вычисление контрольной суммы
            self.arm_sdk_publisher.Write(self.low_cmd)     # Отправка команды

    # Функция для безопасного выключения SDK и моторов
    def _ensure_sdk_disabled(self):
        self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = 0.0  # Выключение SDK
        for joint_id in range(len(self.low_cmd.motor_cmd)):
            if joint_id != G1JointIndex.kNotUsedJoint:
                self.low_cmd.motor_cmd[joint_id].q = 0.0    # Нулевые позиции
                self.low_cmd.motor_cmd[joint_id].dq = 0.0   # Нулевые скорости
                self.low_cmd.motor_cmd[joint_id].kp = 0.0   # Нулевая жёсткость
                self.low_cmd.motor_cmd[joint_id].kd = 0.0   # Нулевое демпфирование
                self.low_cmd.motor_cmd[joint_id].tau = 0.0  # Нулевой момент
        self.low_cmd.crc = self.crc.Crc(self.low_cmd)       # Вычисление контрольной суммы
        self.arm_sdk_publisher.Write(self.low_cmd)          # Отправка команды


# Основная часть программы
if __name__ == '__main__':
    custom = None
    try:
        # Инициализация DDS для связи с роботом
        ChannelFactoryInitialize(config.DOMAIN_ID, config.INTERFACE)

        # Создание и инициализация объекта управления
        custom = Custom()
        custom.Init()

        # Запуск цикла управления
        custom.Start()

        # Ожидание завершения цикла управления
        while custom._running and custom.control_thread and custom.control_thread.is_alive():
            try:
                custom.control_thread.join(0.5)
            except KeyboardInterrupt:
                custom.Stop()
                break

    except KeyboardInterrupt:
        if custom:
            custom.Stop()
    except Exception:
        if custom:
            custom.Stop()
    finally:
        sys.exit(0 if custom and custom.done else 1)