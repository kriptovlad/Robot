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

kPi = 3.141592654
kPi_2 = 1.57079632


class G1JointIndex:
    # Левая нога
    LeftHipPitch = 0
    LeftHipRoll = 1
    LeftHipYaw = 2
    LeftKnee = 3
    LeftAnklePitch = 4
    LeftAnkleRoll = 5

    # Правая нога
    RightHipPitch = 6
    RightHipRoll = 7
    RightHipYaw = 8
    RightKnee = 9
    RightAnklePitch = 10
    RightAnkleRoll = 11

    WaistYaw = 12
    WaistRoll = 13
    WaistPitch = 14

    # Левая рука
    LeftShoulderPitch = 15
    LeftShoulderRoll = 16
    LeftShoulderYaw = 17
    LeftElbow = 18
    LeftWristRoll = 19
    LeftWristPitch = 20
    LeftWristYaw = 21

    # Правая рука
    RightShoulderPitch = 22
    RightShoulderRoll = 23
    RightShoulderYaw = 24
    RightElbow = 25
    RightWristRoll = 26
    RightWristPitch = 27
    RightWristYaw = 28

    kNotUsedJoint = 29


class Custom:
    def __init__(self):
        self.time_ = 0.0
        self.control_dt_ = 0.02  # Частота управления 50 Гц
        self.gait_cycle_ = 1.2  # Длительность одного цикла походки (увеличена для стабильности)
        self.kp = 60.0  # Жёсткость для управления позицией
        self.kd = 1.5  # Демпфирование
        self.kp_ankle = 80.0  # Увеличенная жёсткость для суставов стоп
        self.kd_ankle = 2.0  # Увеличенное демпфирование для суставов стоп
        self.low_cmd = unitree_hg_msg_dds__LowCmd_()
        self.low_state = None
        self.first_update_low_state = False
        self.crc = CRC()
        self.done = False

        self._running = False
        self.control_thread = None
        self.state_lock = threading.Lock()

        # Начальная устойчивая поза для стояния
        self.standing_pos = {
            'legs': [
                -0.2,  # LeftHipPitch
                0.0,   # LeftHipRoll
                0.0,   # LeftHipYaw
                0.3,   # LeftKnee
                -0.1,  # LeftAnklePitch
                0.0,   # LeftAnkleRoll
                -0.2,  # RightHipPitch
                0.0,   # RightHipRoll
                0.0,   # RightHipYaw
                0.3,   # RightKnee
                -0.1,  # RightAnklePitch
                0.0    # RightAnkleRoll
            ],
            'waist': [0.0, 0.0, 0.0],  # WaistYaw, WaistRoll, WaistPitch
            'arms': [
                -0.1,  # LeftShoulderPitch
                0.2,   # LeftShoulderRoll
                0.0,   # LeftShoulderYaw
                0.0,   # LeftElbow
                -0.1,  # RightShoulderPitch
                -0.2,  # RightShoulderRoll
                0.0,   # RightShoulderYaw
                0.0    # RightElbow
            ]
        }

        # Параметры походки (уменьшены для сдержанности)
        self.step_length = 0.25  # Радианы для движения бедра
        self.step_height = 0.15  # Радианы для подъёма колена
        self.waist_sway = 0.05  # Радианы для наклона таза
        self.arm_swing = 0.1  # Радианы для взмаха руки
        self.double_support_ratio = 0.3  # Доля цикла для фазы двойной опоры

        # Индексы суставов
        self.leg_joints = [
            G1JointIndex.LeftHipPitch, G1JointIndex.LeftHipRoll, G1JointIndex.LeftHipYaw,
            G1JointIndex.LeftKnee, G1JointIndex.LeftAnklePitch, G1JointIndex.LeftAnkleRoll,
            G1JointIndex.RightHipPitch, G1JointIndex.RightHipRoll, G1JointIndex.RightHipYaw,
            G1JointIndex.RightKnee, G1JointIndex.RightAnklePitch, G1JointIndex.RightAnkleRoll
        ]
        self.waist_joints = [G1JointIndex.WaistYaw, G1JointIndex.WaistRoll, G1JointIndex.WaistPitch]
        self.arm_joints = [
            G1JointIndex.LeftShoulderPitch, G1JointIndex.LeftShoulderRoll,
            G1JointIndex.RightShoulderPitch, G1JointIndex.RightShoulderRoll
        ]

    def Init(self):
        print("Инициализация DDS Factory...")
        self.arm_sdk_publisher = ChannelPublisher("rt/lowcmd", LowCmd_)
        try:
            self.arm_sdk_publisher.Init()
            print("Публикатор rt/lowcmd инициализирован.")
        except Exception as e:
            print(f"Ошибка инициализации публикатора: {e}")
            return False

        print("Создание подписчика rt/lowstate...")
        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        try:
            self.lowstate_subscriber.Init(self.LowStateHandler, 10)
            print("Подписчик rt/lowstate инициализирован.")
        except Exception as e:
            print(f"Ошибка инициализации подписчика: {e}")
            return False

        print("Инициализация завершена.")
        return True

    def Start(self):
        print("Ожидание первого сообщения LowState...")
        while not self.first_update_low_state:
            time.sleep(0.1)
        print("Первое сообщение LowState получено.")

        if self.first_update_low_state:
            print("Запуск потока управления...")
            self._running = True
            self.control_thread = threading.Thread(target=self._control_loop, name="control_loop")
            self.control_thread.daemon = True
            self.control_thread.start()
            print("Поток управления запущен.")
        else:
            print("Ошибка: невозможно запустить поток управления без LowState.")

    def Stop(self):
        print("Остановка потока управления...")
        self._running = False
        if self.control_thread and self.control_thread.is_alive():
            self.control_thread.join(timeout=1.0)
            if self.control_thread.is_alive():
                print("Предупреждение: поток управления не завершился корректно.")
        print("Поток управления остановлен.")

    def LowStateHandler(self, msg: LowState_):
        with self.state_lock:
            self.low_state = msg
            if not self.first_update_low_state:
                self.first_update_low_state = True

    def _control_loop(self):
        print("Вход в цикл управления...")
        last_execution_time = time.perf_counter()

        while self._running:
            current_time = time.perf_counter()
            time_since_last = current_time - last_execution_time
            wait_time = self.control_dt_ - time_since_last

            if wait_time > 0:
                time.sleep(wait_time)
            last_execution_time = time.perf_counter()

            try:
                self.LowCmdWrite()
                if self.done:
                    print("Задача завершена. Выход из цикла управления.")
                    self._running = False
            except Exception as e:
                print(f"Ошибка в цикле управления: {e}")
                import traceback
                traceback.print_exc()
                self._running = False

        print("Цикл управления завершён.")
        self._ensure_sdk_disabled()

    def LowCmdWrite(self):
        with self.state_lock:
            if self.low_state is None:
                return
            current_low_state = self.low_state

        self.time_ += self.control_dt_

        # Логика походки: непрерывный цикл движения
        if self.time_ < 2.0:
            # Фаза 1: Переход в начальную позу (стояние, 2 секунды)
            ratio = np.clip(self.time_ / 2.0, 0.0, 1.0)
            smooth_ratio = 0.5 * (1.0 - np.cos(np.pi * ratio))  # Синусоидальная интерполяция

            for i, joint_index in enumerate(self.leg_joints):
                current_q = current_low_state.motor_state[joint_index].q
                target_q = self.standing_pos['legs'][i]
                self.low_cmd.motor_cmd[joint_index].q = (1.0 - smooth_ratio) * current_q + smooth_ratio * target_q
                self.low_cmd.motor_cmd[joint_index].dq = 0.0
                self.low_cmd.motor_cmd[joint_index].kp = self.kp_ankle if joint_index in [G1JointIndex.LeftAnklePitch, G1JointIndex.RightAnklePitch] else self.kp
                self.low_cmd.motor_cmd[joint_index].kd = self.kd_ankle if joint_index in [G1JointIndex.LeftAnklePitch, G1JointIndex.RightAnklePitch] else self.kd
                self.low_cmd.motor_cmd[joint_index].tau = 0.0

            for i, joint_index in enumerate(self.waist_joints):
                self.low_cmd.motor_cmd[joint_index].q = self.standing_pos['waist'][i]
                self.low_cmd.motor_cmd[joint_index].dq = 0.0
                self.low_cmd.motor_cmd[joint_index].kp = self.kp
                self.low_cmd.motor_cmd[joint_index].kd = self.kd
                self.low_cmd.motor_cmd[joint_index].tau = 0.0

            for i, joint_index in enumerate(self.arm_joints):
                current_q = current_low_state.motor_state[joint_index].q
                target_q = self.standing_pos['arms'][i]
                self.low_cmd.motor_cmd[joint_index].q = (1.0 - smooth_ratio) * current_q + smooth_ratio * target_q
                self.low_cmd.motor_cmd[joint_index].dq = 0.0
                self.low_cmd.motor_cmd[joint_index].kp = self.kp
                self.low_cmd.motor_cmd[joint_index].kd = self.kd
                self.low_cmd.motor_cmd[joint_index].tau = 0.0

        else:
            # Фаза 2: Непрерывная походка с фазой двойной опоры
            gait_time = (self.time_ - 2.0) % self.gait_cycle_
            phase = (gait_time / self.gait_cycle_) * 2 * np.pi  # Нормализация в диапазон [0, 2pi]

            # Определение фазы маха с учётом двойной опоры
            swing_phase = np.sin(phase)
            if abs(swing_phase) < self.double_support_ratio:
                left_swing = 0.0
                right_swing = 0.0  # Двойная опора: обе ноги на земле
            else:
                left_swing = np.sign(swing_phase) * (abs(swing_phase) - self.double_support_ratio) / (1.0 - self.double_support_ratio)
                right_swing = -left_swing  # Фазовый сдвиг на 180 градусов

            leg_targets = self.standing_pos['legs'].copy()
            # Левая нога
            if left_swing > 0:  # Фаза маха
                leg_targets[G1JointIndex.LeftHipPitch - G1JointIndex.LeftHipPitch] += self.step_length * left_swing
                leg_targets[G1JointIndex.LeftKnee - G1JointIndex.LeftHipPitch] += self.step_height * left_swing
                leg_targets[G1JointIndex.LeftAnklePitch - G1JointIndex.LeftHipPitch] = -0.1 * left_swing
            else:  # Фаза опоры
                leg_targets[G1JointIndex.LeftHipPitch - G1JointIndex.LeftHipPitch] += 0.1 * left_swing
                leg_targets[G1JointIndex.LeftKnee - G1JointIndex.LeftHipPitch] = 0.3
                leg_targets[G1JointIndex.LeftAnklePitch - G1JointIndex.LeftHipPitch] = -0.1

            # Правая нога
            if right_swing > 0:  # Фаза маха
                leg_targets[G1JointIndex.RightHipPitch - G1JointIndex.LeftHipPitch] += self.step_length * right_swing
                leg_targets[G1JointIndex.RightKnee - G1JointIndex.LeftHipPitch] += self.step_height * right_swing
                leg_targets[G1JointIndex.RightAnklePitch - G1JointIndex.LeftHipPitch] = -0.1 * right_swing
            else:  # Фаза опоры
                leg_targets[G1JointIndex.RightHipPitch - G1JointIndex.LeftHipPitch] += 0.1 * right_swing
                leg_targets[G1JointIndex.RightKnee - G1JointIndex.LeftHipPitch] = 0.3
                leg_targets[G1JointIndex.RightAnklePitch - G1JointIndex.LeftHipPitch] = -0.1

            for i, joint_index in enumerate(self.leg_joints):
                self.low_cmd.motor_cmd[joint_index].q = leg_targets[i]
                self.low_cmd.motor_cmd[joint_index].dq = 0.0
                self.low_cmd.motor_cmd[joint_index].kp = self.kp_ankle if joint_index in [G1JointIndex.LeftAnklePitch, G1JointIndex.RightAnklePitch] else self.kp
                self.low_cmd.motor_cmd[joint_index].kd = self.kd_ankle if joint_index in [G1JointIndex.LeftAnklePitch, G1JointIndex.RightAnklePitch] else self.kd
                self.low_cmd.motor_cmd[joint_index].tau = 0.0

            # Движение таза: наклон и вращение
            waist_targets = [
                self.waist_sway * swing_phase,  # WaistYaw
                self.waist_sway * np.cos(phase),  # WaistRoll
                0.0  # WaistPitch
            ]
            for i, joint_index in enumerate(self.waist_joints):
                self.low_cmd.motor_cmd[joint_index].q = waist_targets[i]
                self.low_cmd.motor_cmd[joint_index].dq = 0.0
                self.low_cmd.motor_cmd[joint_index].kp = self.kp
                self.low_cmd.motor_cmd[joint_index].kd = self.kd
                self.low_cmd.motor_cmd[joint_index].tau = 0.0

            # Движение рук: уравновешивающий мах
            arm_targets = [
                -self.arm_swing * right_swing,  # LeftShoulderPitch (противоположно правой ноге)
                0.2,
                -self.arm_swing * left_swing,   # RightShoulderPitch (противоположно левой ноге)
                -0.2
            ]
            for i, joint_index in enumerate(self.arm_joints):
                self.low_cmd.motor_cmd[joint_index].q = arm_targets[i]
                self.low_cmd.motor_cmd[joint_index].dq = 0.0
                self.low_cmd.motor_cmd[joint_index].kp = self.kp
                self.low_cmd.motor_cmd[joint_index].kd = self.kd
                self.low_cmd.motor_cmd[joint_index].tau = 0.0

        # Завершение после 10 циклов походки
        if self.time_ > 2.0 + 10 * self.gait_cycle_:
            print("Естественная устойчивая походка завершена.")
            self.done = True
            self._ensure_sdk_disabled()

        try:
            self.low_cmd.crc = self.crc.Crc(self.low_cmd)
            self.arm_sdk_publisher.Write(self.low_cmd)
        except Exception as e:
            print(f"Ошибка записи LowCmd: {e}")
            self._running = False

    def _ensure_sdk_disabled(self):
        print("Обеспечение отключения SDK и безопасности моторов.")
        self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = 0.0
        for joint_id in range(len(self.low_cmd.motor_cmd)):
            if joint_id != G1JointIndex.kNotUsedJoint:
                self.low_cmd.motor_cmd[joint_id].q = 0.0
                self.low_cmd.motor_cmd[joint_id].dq = 0.0
                self.low_cmd.motor_cmd[joint_id].kp = 0.0
                self.low_cmd.motor_cmd[joint_id].kd = 0.0
                self.low_cmd.motor_cmd[joint_id].tau = 0.0
        try:
            self.low_cmd.crc = self.crc.Crc(self.low_cmd)
            self.arm_sdk_publisher.Write(self.low_cmd)
            print("Команда финального отключения отправлена.")
        except Exception as e:
            print(f"Ошибка отправки команды финального отключения: {e}")


if __name__ == '__main__':
    custom = None
    try:
        print("Инициализация DDS Factory...")
        print(f"Используемый интерфейс DDS: {config.INTERFACE}")
        if not config.INTERFACE:
            print("Предупреждение: интерфейс DDS не указан в конфигурации. Используется значение по умолчанию.")
        ChannelFactoryInitialize(config.DOMAIN_ID, config.INTERFACE)
        print(f"DDS инициализирован с Domain ID: {config.DOMAIN_ID}")

        custom = Custom()
        print("Инициализация объекта Custom...")
        if not custom.Init():
            print("Ошибка инициализации объекта Custom. Выход.")
            sys.exit(1)
        print("Объект Custom успешно инициализирован.")

        custom.Start()

        print("Главный поток ожидает завершения цикла управления...")
        while custom._running and custom.control_thread and custom.control_thread.is_alive():
            try:
                custom.control_thread.join(0.5)
            except KeyboardInterrupt:
                print("\nПолучен KeyboardInterrupt в главном цикле. Остановка...")
                custom.Stop()
                break

        if custom and custom.done:
            print("Главный поток: задача успешно завершена.")
        else:
            print("Главный поток: цикл управления завершён до завершения задачи (или был прерван).")

    except KeyboardInterrupt:
        print("\nПолучен KeyboardInterrupt во время настройки. Остановка...")
        if custom:
            custom.Stop()
    except Exception as e:
        print(f"\nПроизошла непредвиденная ошибка в главном блоке: {e}")
        import traceback
        traceback.print_exc()
        if custom:
            custom.Stop()
    finally:
        print("Выход из программы.")
        sys.exit(0 if custom and custom.done else 1)