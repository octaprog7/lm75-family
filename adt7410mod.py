"""Модуль для Analog Devices ADT7410 (+/-0.5 гр. Ц., 13/16 бит).
Совместим с архитектурой LM75LikeBase."""

from lm75_family_constants import (
    ADT7410_ACCURACY_TYP,
    ADT7410_TEMP_MIN, ADT7410_TEMP_MAX,
    ADT7410_LSB_13, ADT7410_LSB_16
)
from lm75mod import LM75LikeBase
from sensor_pack_2.bus_service import I2cAdapter
from sensor_pack_2.bitfield import bit_field_info, BitFields
from sensor_pack_2.base_sensor import check_value
from micropython import const

# Постоянные
_ADDR_TEMP_MS = const(0x00)
_ADDR_TEMP_LS = const(0x01)
_ADDR_CONFIG  = const(0x03)
_ADDR_THIGH   = const(0x04)
_ADDR_TLOW    = const(0x06)
_ADDR_THYST   = const(0x0A)

# Режимы работы (Биты [6:5] регистра конфигурации)
_MODE_CONTINUOUS = const(0x00)
_MODE_ONE_SHOT   = const(0x01)
_MODE_SHUTDOWN   = const(0x03)

# Маски для битовых операций
_MASK_MODE_BITS  = const(0x9F)  # 1001 1111 (очищает биты 6 и 5)
_MASK_RES_BIT    = const(0x7F)  # 0111 1111 (очищает бит 7)


class ADT7410(LM75LikeBase):
    """Модуль для ADT7410.
    Отличия от LM75: адрес конфига 0x03, OP_MODE [6:5] вместо SHUTDOWN,
    13-битный формат по умолчанию (биты 2-0 — флаги), THYST в отдельном регистре 0x0A."""

    # Константы имен битовых полей регистра конфигурации (0x03)
    BF_NAME_CT_POLARITY = const("CT_POLARITY")
    BF_NAME_OP_MODE     = const("OP_MODE")
    # ADT7410 использует инверсную логику компаратора (0=Int, 1=Comp)
    COMP_MODE_INVERTED: bool = True

    # Маппинг логических индексов на физические адреса
    ADDR_CONFIG = _ADDR_CONFIG
    ADDR_TEMP_HYST = _ADDR_THYST
    ADDR_TOS = _ADDR_THIGH
    ADDR_TEMPERATURE = _ADDR_TEMP_MS

    def __init__(self, adapter: I2cAdapter, address: int):
        super().__init__(adapter, address)

        # Описание битовых полей регистра конфигурации (адрес 0x03)
        self._config_fields = BitFields((
            bit_field_info(name=self.BF_NAME_FAULT_QUEUE, position=range(2), valid_values=range(4), description=None),
            bit_field_info(name=self.BF_NAME_CT_POLARITY, position=range(2, 3), valid_values=None, description=None),
            bit_field_info(name=self.BF_NAME_OS_POLARITY, position=range(3, 4), valid_values=None, description=None),
            bit_field_info(name=self.BF_NAME_COMP_MODE, position=range(4, 5), valid_values=None, description=None),
            bit_field_info(name=self.BF_NAME_OP_MODE, position=range(5, 7), valid_values=None, description=None),
            bit_field_info(name=self.BF_NAME_CONV_RESOL, position=range(7, 8), valid_values=None, description=None),
        ))

        self._one_shot_mode_support = True
        self._resol_changeable = True
        self.refresh_config_cache()

    # карта регистров
    def get_hw_reg_addr(self, reg_index: int) -> int:
        if reg_index == self.ADDR_CONFIG: return _ADDR_CONFIG
        if reg_index == self.ADDR_TEMP_HYST: return _ADDR_THYST
        if reg_index == self.ADDR_TOS: return _ADDR_THIGH
        if reg_index == self.ADDR_TEMPERATURE: return _ADDR_TEMP_MS
        return reg_index

    # Управление питанием (переопределение для обхода BF_NAME_SHUTDOWN)
    def set_shutdown(self, value: bool | None = None, read_from_cache: bool = False) -> bool:
        """Управление питанием через OP_MODE [6:5]."""
        if value is None:
            # get_config_field уже возвращает сдвинутое логическое значение (0..3)
            mode = self.get_config_field(self.BF_NAME_OP_MODE, read_from_cache=read_from_cache)
            return mode == _MODE_SHUTDOWN

        self.refresh_config_cache()
        target = _MODE_SHUTDOWN if value else _MODE_CONTINUOUS
        # BitFields сама сделает (target << 5) & mask
        self.set_config_field(value=target, field_name=self.BF_NAME_OP_MODE)
        self.set_config(value=self.get_config_field())
        return value

    def start_measurement(self, one_shot: bool = False):
        """Запуск измерений. one_shot=True -> OP_MODE=01, иначе OP_MODE=00."""
        self.refresh_config_cache()
        mode_val = _MODE_ONE_SHOT if one_shot else _MODE_CONTINUOUS
        self.set_config_field(value=mode_val, field_name=self.BF_NAME_OP_MODE)
        self.set_config(value=self.get_config_field())

    # Разрешение (13 бит / 16 бит)
    def get_resolution_code(self) -> int:
        """0 = 13 бит, 1 = 16 бит."""
        return self.get_config_field(self.BF_NAME_CONV_RESOL, read_from_cache=False)

    def set_resolution(self, code: int | None = None) -> int:
        """Устанавливает разрешение. 0=13 бит, >0=16 бит."""
        if code is None:
            return self.get_resolution_code()

        # Адаптация под универсальный тест
        adt_code = 1 if code > 0 else 0
        self.refresh_config_cache()
        self.set_config_field(value=adt_code, field_name=self.BF_NAME_CONV_RESOL)
        self.set_config(value=self.get_config_field())
        return self.get_resolution_code()


    # пороги и метрология
    def get_typical_accuracy(self) -> float: return ADT7410_ACCURACY_TYP
    def get_supported_thresholds(self) -> tuple[float, float]: return ADT7410_TEMP_MIN, ADT7410_TEMP_MAX
    def get_current_lsb(self) -> float: return ADT7410_LSB_16 if self.get_resolution_code() else ADT7410_LSB_13
    def get_threshold_lsb(self) -> float: return ADT7410_LSB_13

    def set_thresholds(self, thresholds: tuple[float, float] | None = None) -> tuple[float, float]:
        if thresholds is not None:
            low, high = thresholds
            self._validate_thresholds(low, high)

            # THYST (0x0A) — 8 бит, шаг 1 гр. Ц. Фиксируем 5 гр. Ц. по даташиту
            self.set_reg_val(addr=_ADDR_THYST, value=5, bytes_count=1)

            # THIGH (0x04/0x05) — 16 бит
            th_raw = self.celsius_to_raw(high, threshold=True)
            self.set_reg_val(_ADDR_THIGH, (th_raw >> 8) & 0xFF, bytes_count=1)
            self.set_reg_val(_ADDR_THIGH + 1, th_raw & 0xFF, bytes_count=1)

            # TLOW (0x06/0x07) — 16 бит
            tl_raw = self.celsius_to_raw(low, threshold=True)
            self.set_reg_val(_ADDR_TLOW, (tl_raw >> 8) & 0xFF, bytes_count=1)
            self.set_reg_val(_ADDR_TLOW + 1, tl_raw & 0xFF, bytes_count=1)

        # Чтение обратно для подтверждения
        h_msb = self.get_reg_val(_ADDR_THIGH, 1, signed=True)
        h_lsb = self.get_reg_val(_ADDR_THIGH + 1, 1)
        high_val = self.raw_to_celsius((h_msb << 8) | h_lsb, threshold=True)

        l_msb = self.get_reg_val(_ADDR_TLOW, 1, signed=True)
        l_lsb = self.get_reg_val(_ADDR_TLOW + 1, 1)
        low_val = self.raw_to_celsius((l_msb << 8) | l_lsb, threshold=True)

        return low_val, high_val

    # Преобразование температур
    def celsius_to_raw(self, celsius: float, threshold: bool = False) -> int:
        """Преобразование float в 16-битное two's complement."""
        lsb = self.get_threshold_lsb() if threshold else self.get_current_lsb()
        raw = int(round(celsius / lsb))

        # В 13-битном режиме температура хранится в битах [15:3]
        if not self.get_resolution_code():
            raw = raw << 3

        return raw & 0xFFFF

    def raw_to_celsius(self, raw: int, threshold: bool = False) -> float:
        """Преобразование raw (signed int от unpack) в float."""
        lsb = self.get_threshold_lsb() if threshold else self.get_current_lsb()

        # В 13-битном режиме отбрасываем флаги [2:0]
        if not self.get_resolution_code():
            raw = raw >> 3

        return raw * lsb

    def get_conversion_cycle_time(self) -> int:
        return 250