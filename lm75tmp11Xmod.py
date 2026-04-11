# MIT license
# lm75tmp11Xmod.py — Драйвер для TMP117/TMP119 (строгий контракт, твой стиль преобразований)

from micropython import const
from collections import namedtuple
from sensor_pack_2.bus_service import I2cAdapter
from sensor_pack_2.base_sensor import DeviceEx, IBaseSensorEx, Iterator, check_value
from sensor_pack_2.comp_interface import ICompInterface, CompMode
from lm75mod import ILM75Sensor, PerformanceMode, ISensorPowerControl


# ========================================================================
# Константы и твои функции преобразования (вставлены дословно)
# ========================================================================
_SCALE: float = const(0.0078125)
_SCALE_INV: float = const(128.0)
_HEX_FFFF: int = const(0xFFFF)
# ========================================================================
# Register Addresses (TMP117/TMP119)
# ========================================================================
_REG_TEMP: int = const(0x00)
_REG_CONFIG: int = const(0x01)
_REG_THIGH: int = const(0x02)
_REG_TLOW: int = const(0x03)
_REG_DEVICE_ID: int = const(0x0F)

# Битовые маски и сдвиги (Table 7-6 SBOS725C)
_MASK_MOD: int = const(0x0C00)    # 11:10 — режим конвертации
_SHIFT_MOD: int = const(10)
_MASK_CONV: int = const(0x0380)   # 9:7 — время цикла
_SHIFT_CONV: int = const(7)
_MASK_AVG: int = const(0x0060)    # 6:5 — усреднение
_SHIFT_AVG: int = const(5)
_MASK_TNA: int = const(0x0010)    # 4 — Therm/Alert mode select
_SHIFT_TNA: int = const(4)
_MASK_POL: int = const(0x0008)    # 3 — ALERT pin polarity
_SHIFT_POL: int = const(3)

# карта режимов производительности: (AVG, CONV)
# Плавный рост времени цикла [мс]: 16, 125, 250, 1000, 4000
_PERF_MAP: tuple = ((0, 0), (0, 1), (1, 2), (3, 3), (3, 5))
# Базовое время цикла преобразования для CONV[2:0] при AVG=00 (в мс)
# Индексы: 0 1 2 3 4 5 6 7
_CONV_BASE_TIME_MS: tuple[int, ...] = const((16, 125, 250, 500, 1000, 4000, 8000, 16000))
# Минимальное время цикла, требуемое режимом усреднения AVG[1:0] (в мс)
# Если время усреднения больше базового CONV, цикл удлиняется (standby = 0)
# Индексы: b00 b01 b10 b11
_AVG_MIN_CYCLE_MS: tuple[int, ...] = const((0, 125, 500, 1000))
# форматы для unpack
_str_h = const("h")
_str_H = const("H")
# режим работы - отключен (пониженное потребление тока)
_SHUTDOWN_MODE = const(0x0400)  # 0b01 << 10
# режимы измерений
_MODE_CONTINUOUS = const(0b00)  # непрерывный режим измерений
_MODE_ONE_SHOT = const(0b11)    # однократный режим измерений
#
id_tmp11X = namedtuple("id_tmp11X", "revision_number device_id")

class TMP11X(ILM75Sensor, IBaseSensorEx, ICompInterface, Iterator, ISensorPowerControl):
    """
    Универсальный драйвер для семейства TMP117/TMP119.
    Реализует ВСЕ абстрактные методы интерфейсов.
    Использует твой стиль преобразований и единственный метод set_reg.
    """
    CONFIG_REG_WIDTH = const(2)
    COMP_MODE_INVERTED = True

    def __init__(self, adapter: I2cAdapter, address: int):
        self._connection = DeviceEx(adapter=adapter, address=address, big_byte_order=True)
        self._buf_2 = bytearray(2)
        self._cfg_cache: int = 0
        self._one_shot_mode_support = True
        self._resol_changeable = False
        # Сохранение кэша конфигурации
        self._cfg_cache = self.set_reg(_REG_CONFIG, _str_H)

    # =========================================================================
    # ЕДИНСТВЕННЫЙ метод доступа к регистрам (зафиксирован без изменений)
    # =========================================================================
    def set_reg(self, addr: int, format_value: str | None, value: int | None = None) -> int:
        """Возвращает (при value is None)/устанавливает (при not value is None) содержимое регистра с адресом addr.
        разрядность регистра 16 бит!"""
        buf = self._buf_2
        _conn = self._connection
        if value is None:
            # читаю из Register устройства в буфер два байта
            if format_value is None:
                raise ValueError("При чтении из регистра не задан формат его значения!")
            _conn.read_buf_from_mem(address=addr, buf=buf, address_size=1)
            return _conn.unpack(fmt_char=format_value, source=buf)[0]
        #
        return self._connection.write_reg(reg_addr=addr, value=value, bytes_count=len(buf))

    # =========================================================================
    # ILM75Sensor — контракт температурного датчика
    # =========================================================================
    def get_typical_accuracy(self) -> float: return 0.1
    def get_current_lsb(self) -> float: return _SCALE
    def get_threshold_lsb(self) -> float: return _SCALE
    # def get_resolution_code(self) -> int: return 0  # Фиксированное 16-бит

    def celsius_to_raw(self, celsius: float, threshold: bool = False) -> int:
        return int(_SCALE_INV * celsius) & _HEX_FFFF

    def raw_to_celsius(self, raw: int, threshold: bool = False) -> float:
        return _SCALE * raw

    def get_measurement_value(self, index: int = 0) -> float:
        # 'h' распаковывает signed 16-bit, знак уже обработан
        return self.raw_to_celsius(self.set_reg(_REG_TEMP, _str_h))

    def __next__(self) -> float:
        return self.get_measurement_value(0)

    # ICompInterface — управление компаратором
    def set_comp_mode(self, mode: int | None = None, active_alarm_level: bool = False) -> int:
        cfg = self.set_reg(_REG_CONFIG, _str_H)
        if mode is not None:
            if mode == CompMode.COMPARATOR:
                cfg = (cfg & ~_MASK_TNA) | (1 << _SHIFT_TNA)
            elif mode == CompMode.INTERRUPT:
                cfg = (cfg & ~_MASK_TNA) | (0 << _SHIFT_TNA)
            else:
                raise ValueError("Режим: 0=COMPARATOR, 1=INTERRUPT")

        pol = 1 if active_alarm_level else 0
        cfg = (cfg & ~_MASK_POL) | (pol << _SHIFT_POL)
        self.set_reg(_REG_CONFIG, _str_H, cfg)
        self._cfg_cache = cfg

        return CompMode.COMPARATOR if ((cfg & _MASK_TNA) >> _SHIFT_TNA) else CompMode.INTERRUPT

    # TMP11X
    def get_supported_thresholds(self) -> tuple[float, float]:
        return -55.0, 150.0

    def set_thresholds(self, thresholds: tuple[float, float] | None = None) -> tuple[float, float]:
        if thresholds is None:
            raw_low = self.set_reg(_REG_TLOW, _str_h)
            raw_high = self.set_reg(_REG_THIGH, _str_h)
            return self.raw_to_celsius(raw_low), self.raw_to_celsius(raw_high)

        t_low, t_high = thresholds
        self._validate_thresholds(t_low, t_high)  # проверка
        self.set_reg(_REG_TLOW, _str_h, self.celsius_to_raw(t_low))
        self.set_reg(_REG_THIGH, _str_h, self.celsius_to_raw(t_high))
        return t_low, t_high

    def is_over_threshold(self) -> bool:
        cfg = self.set_reg(_REG_CONFIG, _str_H)
        return bool(((cfg >> 15) & 1) | ((cfg >> 14) & 1))

    # ISensorPowerControl — управление питанием
    def set_shutdown(self, value: bool | None = None, read_from_cache: bool = False) -> bool:
        """Реализация ISensorPowerControl.set_shutdown"""
        cfg = self._cfg_cache if read_from_cache else self.set_reg(_REG_CONFIG, _str_H)
        if value is None:
            return (cfg & _MASK_MOD) == _SHUTDOWN_MODE

        # 2-битное значение режима: 0b01 = Shutdown, 0b00 = Continuous
        mode_bits = _SHUTDOWN_MODE if value else 0b00
        _cfg = (cfg & ~_MASK_MOD) | mode_bits

        self.set_reg(_REG_CONFIG, _str_H, _cfg)
        self._cfg_cache = _cfg
        return value

    def start_measurement(self, one_shot: bool = False):
        """
        Запуск процесса измерений.

        Args:
            one_shot (bool): True: One-Shot (измерение -> авто-Sleep).
                             False: Continuous Conversion Mode.
        """
        mode = _MODE_ONE_SHOT if one_shot else _MODE_CONTINUOUS
        cfg = (self._cfg_cache & ~_MASK_MOD) | (mode << _SHIFT_MOD)
        self.set_reg(_REG_CONFIG, _str_H, cfg)
        self._cfg_cache = cfg

    # TMP11X-специфичные методы
    def get_conversion_cycle_time(self) -> int:
        """
        Возвращает время преобразования температуры датчиком в миллисекундах
        в зависимости от его настроек (Table 7-7, Section 7.4.3).
        """
        conv = (self._cfg_cache & _MASK_CONV) >> _SHIFT_CONV
        check_value(conv, range(8), f"Invalid conversion cycle time value: {conv}")

        avg = (self._cfg_cache & _MASK_AVG) >> _SHIFT_AVG
        check_value(avg, range(4), f"Invalid conversion averaging mode value: {avg}")

        mode = (self._cfg_cache & _MASK_MOD) >> _SHIFT_MOD

        # В One-Shot режиме CONV игнорируется (раздел 7.4.3)
        if mode == 3:
            return _AVG_MIN_CYCLE_MS[avg]

        # Запланированное время цикла из настроек CONV
        base_time = _CONV_BASE_TIME_MS[conv]
        # Минимально необходимое время для выбранного усреднения AVG
        min_required_time = _AVG_MIN_CYCLE_MS[avg]

        # Реальное время = максимум из двух (общее время цикла не может быть меньше
        # времени, которое физически требуется датчику на выполнение всех измерений)
        return base_time if base_time > min_required_time else min_required_time

    def set_performance_mode(self, mode: int | None = None) -> int:
        if mode is None:
            # режим чтения
            avg = (self._cfg_cache & _MASK_AVG) >> _SHIFT_AVG
            conv = (self._cfg_cache & _MASK_CONV) >> _SHIFT_CONV
            for m, (a, c) in enumerate(_PERF_MAP):
                if avg == a and conv == c:
                    return m
            return 0  # при неизвестной комбинации битов

        valid_modes = range(1 + PerformanceMode.MAX_ACCURACY)
        check_value(mode, valid_modes, f"PerformanceMode: допустимо {valid_modes}")
        avg, conv = _PERF_MAP[mode]
        cfg = (self._cfg_cache & ~_MASK_AVG & ~_MASK_CONV) | (avg << _SHIFT_AVG) | (conv << _SHIFT_CONV)
        self.set_reg(_REG_CONFIG, _str_H, cfg)
        self._cfg_cache = cfg
        return mode # возвращаю установленный режим

    def get_supported_threshold_range(self) -> tuple[float, float]:
        return -55.0, 150.0

    def get_device_id(self) -> id_tmp11X:
        raw = self.set_reg(_REG_DEVICE_ID, _str_H)  # 16-битное чтение
        rev = (raw >> 12) & 0x0F  # rev
        did = raw & 0x0FFF  # Device ID
        return id_tmp11X(rev, did)

    def soft_reset(self):
        cfg = self.set_reg(_REG_CONFIG, _str_H) | 0x0002
        self.set_reg(_REG_CONFIG, _str_H, cfg)
        self._cfg_cache = cfg


class TMP119(TMP11X):
    """Псевдоним. Электрически и программно идентичен TMP11X."""
    pass