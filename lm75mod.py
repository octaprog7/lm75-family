# MIT license

# Иерархия
# LM75LikeBase          # Абстрактный контракт, I2C, probing, пороги
#    ├─ LM75            # Legacy: 9 бит фикс, 100 мс конвертация
#    └─ TMP75           # Modern: 9-12 бит динамически, One-Shot, 28-224 мс
#         ├─ TMP175, TMP275, TMP75A/B/C  # Пустые псевдонимы
#         └─ LM75A/B/C/D                 # Пустые псевдонимы

from micropython import const
from sensor_pack_2.bus_service import I2cAdapter
from sensor_pack_2.base_sensor import DeviceEx, IBaseSensorEx, Iterator, check_value, check_value_ex
from sensor_pack_2.bitfield import BitFields, bit_field_info, make_namedtuple
from sensor_pack_2.comp_interface import ICompInterface


class LM75LikeBase(IBaseSensorEx, ICompInterface, Iterator):
    """Класс для работы с датчиком температуры, подобным LM75."""
    # Индекс регистра температуры
    ADDR_TEMPERATURE = const(0x00)
    # Индекс регистра настройки/конфигурации
    ADDR_CONFIG = const(0x01)
    # Индекс регистра гистерезиса температуры в датчике.
    # Задает нижний температурный порог срабатывания выхода OS (Overtemperature Shutdown).
    ADDR_TEMP_HYST = const(0x02)
    # Индекс регистра порога температуры датчика, при превышении которого,
    # срабатывает выход OS (Over Setpoint - выше верхней уставки).
    ADDR_TOS = const(0x03)
    # разрядность регистра настройки/конфигурации датчика
    CONFIG_REG_WIDTH = const(1)  # по умолчанию — 1 байт (для LM75, MAX30205 и др.)

    # Имена битовых полей (константы для защиты от опечаток)
    BF_NAME_SHUTDOWN = const("SHUTDOWN")
    BF_NAME_COMP_MODE = const("COMP_MODE")
    BF_NAME_OS_POLARITY = const("OS_POLARITY")
    BF_NAME_FAULT_QUEUE = const("FAULT_QUEUE")
    # разрядность преобразователя
    BF_NAME_CONV_RESOL = const("CONV_RESOL")
    # имя поля режима однократного измерения
    BF_NAME_ONE_SHOT = const("ONE_SHOT")

    # ИНВЕРСИЯ ЗНАЧЕНИЙ РЕЖИМА КОМПАРАТОРА
    # Разные производители используют противоположные значения битов в регистре
    # конфигурации (Configuration Register):
    # LM75, MAX30205 (бит 1 регистра Config):
    #   Бит = 0 → Comparator mode (термостат)
    #   Бит = 1 → Interrupt mode (прерывание)
    #
    # TMP117, TMP119 (бит 4 регистра Config, поле T/N_A):
    #   Бит = 0 → Alert mode (прерывание) ИНВЕРСИЯ!
    #   Бит = 1 → Therm mode (термостат) ИНВЕРСИЯ!
    #
    # Этот флаг указывает, нужно ли инвертировать универсальные значения
    # CompMode.COMPARATOR/INTERRUPT при записи в регистр конкретного датчика.
    # Для LM75 и т. п.: CompMode.COMPARATOR (0) → регистр конфигурации, бит 0 = 0
    # Для TMP117, TMP119: CompMode.COMPARATOR (0) → регистр конфигурации, бит 4 = 1 ⚠(инверсия!)
    COMP_MODE_INVERTED: bool = False

    def get_hw_reg_addr(self, reg_index: int) -> int:
        """Возвращает аппаратный адрес соответствующего регистра датчика.
        reg_index           регистр датчика
        -------------------------------------
        ADDR_TEMPERATURE    температура
        ADDR_CONFIG         настройки
        ADDR_TEMP_HYST      нижний температурный порог срабатывания выхода OS (Overtemperature Shutdown).
        ADDR_TOS            порог температуры датчика, при превышении которого, срабатывает выход OS (Over Setpoint - выше верхней уставки).
        -------------------------------------
        Переопределить в наследниках при необходимости!
        """
        return reg_index

    # описание регистра конфигурации LM75 (TI/NXP)
    # та же самая структура регистра конфигурации у TCN75A (Microchip), LM75B (NXP), DS75 (Maxim Integrated/Analog Devices).
    config_reg_LM75 = (
        # Установите бит в 1, чтобы перевести устройство в режим выключения и снизить ток потребления
        bit_field_info(name=BF_NAME_SHUTDOWN, position=range(0, 1), valid_values=None, description=None),
        # 0 - comparator mode; 1 - interrupt mode
        bit_field_info(name=BF_NAME_COMP_MODE, position=range(1, 2), valid_values=None, description=None),
        # Установи бит в 0, чтобы активным уровнем OS стал низкий(0) уровень. Установи бит в 1,
        # чтобы активным уровнем OS стал активный высокий(1) уровень.
        # OS является выходом ИМС с открытым стоком, поэтому нужен подтягивающий резистор!
        bit_field_info(name=BF_NAME_OS_POLARITY, position=range(2, 3), valid_values=None, description=None),
        # Определяет количество событий over setpoint, необходимых для возникновения состояния ОS (over setpoint).
        # Для снижения влияния шума. Рекомендую максимальное значение - 3!
        bit_field_info(name=BF_NAME_FAULT_QUEUE, position=range(3, 5), valid_values=range(4), description=None),
    )

    def _convert_comp_mode(self, mode: int) -> int:
        """Конвертирует универсальное значение CompMode в значение для регистра датчика.
        Аргументы:
            mode (int): CompMode.COMPARATOR (0) или CompMode.INTERRUPT (1).
        Возвращает:
            int: Значение для записи в регистр датчика.
        Пример:
            LM75:  _convert_comp_mode(0) → 0 (без инверсии)
            TMP117: _convert_comp_mode(0) → 1 (инверсия!)
        """
        if self.COMP_MODE_INVERTED:
            return 1 - mode  # Инверсия для TMP117/TMP119
        return mode  # Без изменений для LM75, MAX30205

    def _validate_thresholds(self, low: float, high: float) -> None:
        """Проверяет, что оба порога находятся в допустимом диапазоне,
        поддерживаемом данным типом датчика.
        Выбрасывает ValueError при нарушении."""
        tpl_valid_rng = self.get_supported_threshold_range()
        def _get_err_str(val: float | int) -> str:
            """Возвращает строку сообщения об ошибке"""
            return f"Пороговое значение {val} находится вне пределов поддерживаемого диапазона: [{tpl_valid_rng[0]}, {tpl_valid_rng[1]}]"
        check_value_ex(low, tpl_valid_rng, _get_err_str(low))
        check_value_ex(high, tpl_valid_rng, _get_err_str(high))
        # Проверка: low < high
        if low >= high:
            raise ValueError(f"Tmin ({low}) должен быть строго меньше Tmax ({high})!")

    def __init__(self, adapter: I2cAdapter, address: int):
        # Создаем объект DeviceEx для I2C обмена
        self._connection = DeviceEx(adapter=adapter, address=address, big_byte_order=True)
        # Поле _config_fields присвойте в классе-наследнике
        self._config_fields = None  # BitFields(fields_info=_config_reg)
        # Флаг поддержки режима ONE_SHOT (устанавливается автоматически)
        self._one_shot_mode_support: bool = False
        # Истина, если датчик поддерживает изменение разрешения
        self._resol_changeable: bool = False

    def _probe_hardware_capabilities(self) -> None:
        """
        Аппаратно определяет возможности датчика методом Write-Read-Compare.
        Проверяет, записываются ли биты разрешения R1/R0 (биты 5-6).
        """
        conn = self._connection
        addr = self.ADDR_CONFIG  # 0x01

        # 1. Читаем текущее состояние (bytes[0] → int)
        original_cfg = conn.read_reg(addr, 1)[0]

        try:
            # 2. Тест: устанавливаем биты R1/R0 в 1 (код 3 → 12 бит)
            test_cfg = original_cfg | 0x60
            conn.write_reg(addr, test_cfg, 1)

            # 3. Читаем обратно и проверяем
            verify_cfg = conn.read_reg(addr, 1)[0]
            res_bits = (verify_cfg >> 5) & 0x03
            self._resol_changeable = (res_bits == 0x03)

            if self._resol_changeable:
                self._one_shot_mode_support = True

        finally:
            # Восстанавливаю исходное состояние
            conn.write_reg(addr, original_cfg, 1)
            self.refresh_config_cache()

    def get_typical_accuracy(self) -> float:
        """Возвращает типичную погрешность датчика (°C).
        Переопределить в наследниках!"""
        raise NotImplementedError()

    def is_one_shot_supported(self) -> bool:
        """Возвращает флаг поддержки режима однократного измерения.
        Вызывать только после выполнения кода метода _init_one_shot_support!"""
        return self._one_shot_mode_support

    def is_changeable_resol(self) -> bool:
        """Возвращает True, если датчик поддерживает программное изменение разрешения АЦП."""
        return self._resol_changeable

    def _is_field_exist(self, field_name: str) -> bool:
        """Проверяет наличие битового поля в конфигурации."""
        return self._config_fields is not None and field_name in self._config_fields

    def _init_one_shot_support(self) -> None:
        """Проверяет наличие поля ONE_SHOT через и устанавливает флаг."""
        self._one_shot_mode_support = self._is_field_exist(self.BF_NAME_ONE_SHOT)

    def _init_resolution_support(self) -> None:
        """Проверяет наличие поля CONV_RESOL в конфигурации и устанавливает флаг."""
        self._resol_changeable = self._is_field_exist(self.BF_NAME_CONV_RESOL)

    def get_reg_val(self, addr: int, bytes_count: int = 1, signed: bool = False) -> int:
        """Возвращает значение из регистра:
        addr - адрес регистра в адресном пространстве датчика;
        bytes_count определяется разрядностью регистра (1-8 бит/2-16 бит)."""
        conn = self._connection
        _raw = conn.read_reg(addr, bytes_count)
        fmt_char = 'b' if 1 == bytes_count else 'h'
        return conn.unpack(fmt_char if signed else fmt_char.upper(), _raw)[0]

    def set_reg_val(self, addr: int, value: int, bytes_count: int = 1):
        """Записывает значение в регистр датчика:
        addr - адрес регистра в адресном пространстве датчика;
        value - записываемое значение;
        bytes_count определяется разрядностью регистра (1-8 бит/2-16 бит)."""
        conn = self._connection
        _raw = conn.write_reg(addr, value, bytes_count)

    def set_get_config(self, value: int | None, bytes_count: int = None) -> int | None:
        """Чтение/запись регистра конфигурации.
        Если value is None — чтение, иначе — запись.
        bytes_count по умолчанию берётся из CONFIG_REG_WIDTH.
        """
        if bytes_count is None:
            bytes_count = self.CONFIG_REG_WIDTH

        hw_addr = self.get_hw_reg_addr(self.ADDR_CONFIG)

        if value is None:
            return self.get_reg_val(addr=hw_addr, bytes_count=bytes_count, signed=False)
        else:
            self.set_reg_val(addr=hw_addr, value=value, bytes_count=bytes_count)
            return None

    # IBaseSensorEx start
    def get_data_status(self, raw: bool = True):
        """Возвращает состояние готовности данных для считывания?
        Тип возвращаемого значения выбирайте сами!
        Если raw Истина, то возвращается сырое/не обработанное значение состояния!
        У датчиков, подобных LM75, данные всегда готовы для считывания!"""
        return True

    def get_measurement_value(self, value_index: int | None = 0) -> float:
        """Возвращает измеренное датчиком значение(значения) по его индексу/номеру.
        Возвращает измеренное значение температуры в градусах Цельсия."""
        hw_addr = self.get_hw_reg_addr(self.ADDR_TEMPERATURE)
        raw = self.set_get_temp_raw(addr=hw_addr)
        return self.raw_to_celsius(raw)

    def start_measurement(self, one_shot: bool = False,
                          mode: int | None = None,
                          thresholds: tuple[float, float] | None = None,
                          active_alarm_level: bool = False) -> None:
        """
        Настраивает датчик и управляет режимом измерения.

        Аргументы:
            one_shot (bool):
                - True: однократное измерение (ONE_SHOT).
                - False: непрерывные измерения (Continuous).
                - Игнорируется, если датчик не поддерживает ONE_SHOT (LM75).
                - Работает для: MAX30205, TMP117, TMP119.
                - После завершения ONE_SHOT датчик автоматически переходит в SHUTDOWN.

            mode (int | None): Режим работы компаратора.
                - CompMode.COMPARATOR (0) — термостатный режим.
                  ALERT активен, пока T вне диапазона [T_min, T_max].
                  Сбрасывается автоматически при T внутри диапазона [T_min, T_max].
                - CompMode.INTERRUPT (1) — режим прерывания.
                  ALERT активен при T вне диапазона [T_min, T_max].
                  Сбрасывается только чтением регистра.
                - Если None — не изменяется.

            thresholds (tuple[float, float] | None):
                - (T_min, T_max): диапазон в градусах Цельсия.
                - Если None — не изменяется.

            active_alarm_level (bool):
                - False: активный низкий уровень (0V при тревоге).
                - True: активный высокий уровень (V+ при тревоге).

        Заметка:
            - Для LM75 параметр one_shot игнорируется (нет аппаратной поддержки).
            - После ONE_SHOT датчик автоматически в SHUTDOWN — вызовите wakeup() для продолжения.
            - Проверить поддержку: `sensor._one_shot_mode_support`

        Пример:
           # Непрерывные измерения:
            sensor.start_measurement()

            # Однократное измерение (MAX30205, TMP117):
            sensor.start_measurement(one_shot=True)
            time.sleep_ms(sensor.get_conversion_cycle_time())
            # ...wait
            temp = sensor.get_measurement_value()

            # Настройка порогов и режима:
            sensor.start_measurement(
                mode=CompMode.COMPARATOR,
                thresholds=(20.0, 30.0)
            )
        """
        self.refresh_config_cache()

        # Настройка порогов (работает у всех)
        if thresholds is not None:
            self.set_thresholds(thresholds)

        # Настройка режима OS (работает у всех)
        if mode is not None:
            check_value(mode, range(2), f"Неверное значение mode: {mode}. Используйте CompMode.COMPARATOR или CompMode.INTERRUPT!")
            raw_mode = self._convert_comp_mode(mode)
            self.set_comp_mode(mode=raw_mode, active_alarm_level=active_alarm_level)

        # Управление режимом измерений
        if self.is_one_shot_supported() and one_shot:
            # ONE_SHOT + автоматический SHUTDOWN после измерения
            self._set_shutdown(value=True)
            self._set_one_shot(value=True)
        else:
            # Continuous mode (выход из SHUTDOWN)
            # Примечание: ONE_SHOT не нужно сбрасывать явно:
            # - Это trigger bit (1=запуск, 0=ничего)
            # - При SHUTDOWN=0 (Continuous mode) бит ONE_SHOT игнорируется
            # - После измерения бит сбрасывается аппаратно
            self._set_shutdown(value=False)

        # записываю настройки в регистр
        cfg_raw = self.get_config_field()
        self.set_get_config(value=cfg_raw)

    def is_single_shot_mode(self) -> bool:
        """Возвращает Истина, когда датчик находится в режиме однократных измерений,
        каждое из которых запускается методом start_measurement"""
        return False

    def is_continuously_mode(self) -> bool:
        """Возвращает Истина, когда датчик находится в режиме многократных измерений,
        производимых автоматически. Процесс запускается методом start_measurement"""
        return True
    # IBaseSensorEx stop

    def get_current_lsb(self) -> float:
        """Возвращает текущий 'вес' младшего бита сырого значения температуры в градусах Цельсия."""
        raise NotImplementedError()

    def get_threshold_lsb(self) -> float:
        """LSB для регистров TOS/THYST — всегда 0.5°C у LM75-совместимых."""
        raise NotImplementedError()

    def celsius_to_raw(self, celsius: float, threshold: bool = False) -> int:
        """Преобразует температуру (celsius) из градусов Цельсия в сырое значение, для датчика.
        Args:
            celsius: Температура в градусах Цельсия.
            threshold: Если True, используется LSB для порогов (0.5°C),
                   иначе для измерений (0.125/0.5°C).

            Returns:
                int: Сырое 16-bit значение для записи в регистр."""
        raise NotImplementedError()

    def raw_to_celsius(self, raw: int, threshold: bool = False) -> float:
        """Преобразует температуру (source) из градусов Цельсия в сырое значение, для датчика.
        Если threshold Истина, то используется LSB для пороговых значений температуры,
        иначе используется LSB для измеренной датчиком температуры."""
        raise NotImplementedError()

    def set_get_temp_raw(self, addr: int = 0, value: int = None) -> int | None:
        """Считывает/записывает сырое значение температуры из регистра / в регистр:
        addr - адрес 16-разрядного регистра температуры в адресном пространстве датчика.
        Возвращает значение, если value is None.
        Если value is not None, записывает value в регистр."""
        check_value(value=addr, valid_range=(0, 2, 3), error_msg=f"Неверное значение адреса регистра температуры: {addr}")
        if value is None:
            return self.get_reg_val(addr=addr, bytes_count=2, signed=True)
        self.set_reg_val(addr=addr, value=value, bytes_count=2)
        return None

    def get_supported_threshold_range(self) -> tuple[float, float]:
        """Возвращает допустимый диапазон температур (в градусах Цельсия),
        в пределах которого могут быть заданы пороги срабатывания:
        - нижний порог (THYST / T_LOW)
        - верхний порог (TOS / T_HIGH)

        Датчик	    Рабочий диапазон температуры	Рекомендуемый диапазон TOS / THYST      Примечание
        ---------------------------------------------------------------------------------------------------------------
        MAX30205	0…+50°C	                        0…+50°C                                 Строгое ограничение
        LM75	    –55…+125°C	                    –55…+125°C                              Поддерживает весь диапазон
        TMP117	    –55…+150°C	                    –55…+150°C                              Калибровка по NIST — только от -55 до +50°C. Вне этого диапазона точность видимо хуже
        TMP119	    –55…+150°C	                    –55…+150°C                              То же, что TMP117
        MCP9808	    –40…+125°C	                    –40…+125°C                              Вне диапазона — точность не гарантируется
        TMP102	    –40…+125°C	                    –40…+125°C                              Поддерживает весь диапазон

        Для переопределения в классах - наследниках.
        """
        raise NotImplementedError()


    # Iterator start
    def __next__(self) -> float | None:
        if self.is_single_shot_mode() and not self.is_shutdown():
            return None
        return self.get_measurement_value(0)
    # Iterator stop

    def get_current_config_hr(self) -> tuple:
        """Преобразует текущую конфигурацию датчика в человеко-читаемую (Human Readable).
        Для переопределения в классах-наследниках!"""
        self.refresh_config_cache()
        return make_namedtuple(source=self._config_fields)

    def get_config_field(self, field_name: str | None = None, read_from_cache: bool = True) -> int | bool:
        """Возвращает значение поля по его имени из конфигурации.
        Args:
            field_name: имя поля (например, 'SHUTDOWN'). Если None — возвращается весь регистр;
            read_from_cache: если False — кэш сначала обновляется из датчика (чтение по шине);

        Returns:
            Значение поля или всего регистра (int)."""
        if not read_from_cache:
            self.refresh_config_cache()
        bf = self._config_fields
        if field_name is None:
            return bf.source
        return bf[field_name]

    def set_config_field(self, value: int, field_name: str | None = None) -> int:
        """Устанавливает значение поля, value, по его имени, field_name, в сохраненной конфигурации.
        Если field_name is None, будут установлены значения всех полей конфигурации.
        Возвращает значение сырой конфигурации из полей класса!"""
        bf = self._config_fields
        if field_name is None:
            bf.source = value
        else:
            bf[field_name] = value
        #
        return bf.source

    def refresh_config_cache(self) -> int:
        """Читает регистр конфигурации, возвращает его значение, записывает это значение в кэш."""
        raw_cfg = self.set_get_config(value=None)
        # записываю в кэш сырой конфигурации новое значение
        self.set_config_field(value=raw_cfg)
        return raw_cfg

    def is_shutdown(self, read_from_cache: bool = False) -> bool:
        """Возвращает True, если датчик в режиме малого энергопотребления.
        Если read_from_cache Истина, то читает из кэша, иначе из датчика (после кеширования)."""
        return self.get_config_field(self.BF_NAME_SHUTDOWN, read_from_cache=read_from_cache)

    def _set_shutdown(self, value: bool):
        """Устанавливает битовое поле 'SHUTDOWN' в value"""
        self.set_config_field(value=value, field_name=self.BF_NAME_SHUTDOWN)

    def _set_one_shot(self, value: bool):
        """Устанавливает битовое поле 'ONE_SHOT' в value"""
        self.set_config_field(value=value, field_name=self.BF_NAME_ONE_SHOT)

    def shutdown(self, one_shot: bool = False) -> None:
        """
        Выключает датчик для энергосбережения.

        Аргументы:
            one_shot (bool):
                - True: выполнить однократное измерение перед выключением.
                - False: просто выключить без измерения.
                - Игнорируется, если датчик не поддерживает ONE_SHOT.

        Заметка:
            - После shutdown() датчик не измеряет температуру.
            - Для возобновления измерений вызовите start_measurement...)
            time.sleep_ms(sensor.get_conversion_cycle_time())
            temp = sensor.get_measurement_value()
        """
        self.refresh_config_cache()

        self._set_shutdown(value=True)

        # не является обязательным: ONE_SHOT перед выключением
        if one_shot and self._one_shot_mode_support:
            self._set_one_shot(value=True)

        # Запись настроек в регистр
        cfg_raw = self.get_config_field()
        self.set_get_config(value=cfg_raw)

    def set_fault_queue(self, faults: int | None = None) -> int:
        """
        Устанавливает или возвращает фильтр помех (Fault Queue).
        Определяет количество(!) последовательных событий over-setpoint до срабатывания OS/ALERT.

        Аргументы:
            faults: 0=одно событие (мгновенно), 1=два события, 2=четыре события, 3=шесть событий.
                    Если None — только чтение текущего значения.
        Возвращает:
            int: Текущий код Fault Queue (0..3).
        """
        if faults is not None:
            # check_value(faults, range(4), "Неверное значение Fault Queue! Допустимо 0..3.")
            self.refresh_config_cache()
            self.set_config_field(value=faults, field_name=self.BF_NAME_FAULT_QUEUE)
            self.set_get_config(value=self.get_config_field())

        return self.get_config_field(self.BF_NAME_FAULT_QUEUE, read_from_cache=True)

    # ICompInterface start
    def set_comp_mode(self, mode: int | None = None, active_alarm_level: bool = False) -> int:
        if mode is not None:
            check_value(mode, range(2), f"Неверное значение mode: {mode}")
            # Конвертация для TMP117/TMP119!
            raw_mode = self._convert_comp_mode(mode)
            self.set_config_field(value=raw_mode, field_name=self.BF_NAME_COMP_MODE)
            self.set_config_field(value=active_alarm_level, field_name=self.BF_NAME_OS_POLARITY)
            # запись настроек
            val = self.get_config_field()
            self.set_get_config(value=val)
            #
        return self.get_config_field('COMP_MODE')

    def set_thresholds(self, thresholds: tuple[float, float] | None = None) ->  tuple[float, float]:
        """Устанавливает/возвращает диапазон температур срабатывания. Смотри comp_interface.py.
           Заметка:
                - Минимальное окно: 3 × типичная точность датчика
                - Для TMP75: ≥ 0.75°C (3 × 0.25°C)
                - Для LM75: ≥ 6.0°C (3 × 2.0°C)

        Обоснование: Правило одной трети. Источник: ГОСТ Р 8.736–2011 (п. 5.3.3) + ISO 14253-1
        Суть:
            Погрешность средства измерений должна составлять не более 1/3 от допуска (ширины окна контроля).
            Меньшее окно может вызвать ложные срабатывания!
        """
        hw_temp_hyst = self.get_hw_reg_addr(self.ADDR_TEMP_HYST)
        hw_temp_os = self.get_hw_reg_addr(self.ADDR_TOS)
        #
        if thresholds is not None:
            if not isinstance(thresholds, tuple):
                raise TypeError("Ожидается экземпляр типа tuple")
            _low, _high = thresholds
            if _low >= _high:
                raise ValueError("low должен быть < high")

            # проверка на основе точности
            accuracy = self.get_typical_accuracy()  # Новый метод!
            min_window = 3 * accuracy
            actual_window = _high - _low

            if actual_window < min_window:
                raise ValueError(f"Окно температур ({actual_window:.3f}°C) слишком узкое! Увеличьте разницу между T_min и T_max!")

            # Проверка диапазона thresholds
            self._validate_thresholds(_low, _high)

            thyst_raw = self.celsius_to_raw(celsius=_low, threshold=True)
            tos_raw = self.celsius_to_raw(celsius=_high, threshold=True)
            self.set_get_temp_raw(hw_temp_hyst, value=thyst_raw)
            self.set_get_temp_raw(hw_temp_os, value=tos_raw)
        # читаю значения из регистров
        _val = self.set_get_temp_raw(addr=hw_temp_hyst)
        thyst_c = self.raw_to_celsius(raw=_val, threshold=True)
        _val = self.set_get_temp_raw(addr=hw_temp_os)
        tos_c = self.raw_to_celsius(raw=_val, threshold=True)
        #
        return thyst_c, tos_c

    def is_over_setpoint(self) -> bool:
        """Проверяет, превышена ли верхняя уставка (TOS).
        Чтение регистра температуры сбрасывает флаг OS в interrupt mode."""
        hw_temp_os = self.get_hw_reg_addr(self.ADDR_TOS)
        return self.set_get_temp_raw(0) > self.set_get_temp_raw(hw_temp_os)
    # ICompInterface stop


class LM75(LM75LikeBase):
    """Программное представление оригинального LM75 (Legacy).
    Фиксированное разрешение 9 бит (0.5°C), без поддержки One-Shot.
    Для современных LM75A/B/C/D используйте классы-псевдонимы, наследующиеся от TMP75.
    """

    def __init__(self, adapter: I2cAdapter, address: int):
        super().__init__(adapter, address)
        self._config_fields = BitFields(LM75LikeBase.config_reg_LM75)
        self._init_one_shot_support()
        self._init_resolution_support()
        self._probe_hardware_capabilities()

    def get_typical_accuracy(self) -> float:
        """Возвращает типичную погрешность датчика (°C)."""
        return 2.0

    def get_current_lsb(self) -> float:
        """Возвращает текущий 'вес' младшего бита сырого значения температуры в градусах Цельсия."""
        return 0.5  # 9 бит

    def celsius_to_raw(self, celsius: float, threshold: bool = False) -> int:
        """Преобразует температуру (source) из градусов Цельсия в сырое значение, для датчика.
        Если threshold Истина, то используется LSB для пороговых значений температуры,
        иначе используется LSB для измеренной датчиком температуры."""
        lsb = self.get_threshold_lsb() if threshold else self.get_current_lsb()
        return int(round(celsius / lsb)) << 7

    def raw_to_celsius(self, raw: int, threshold: bool = False) -> float:
        """Преобразует температуру (source) из градусов Цельсия в сырое значение, для датчика.
        Если threshold Истина, то используется LSB для пороговых значений температуры,
        иначе используется LSB для измеренной датчиком температуры."""
        lsb = self.get_threshold_lsb() if threshold else self.get_current_lsb()
        return lsb * (raw >> 7)  # 9 бит

    def get_threshold_lsb(self) -> float:
        """LSB для регистров TOS/THYST — всегда 0.5°C у LM75-совместимых."""
        # У всех LM75-совместимых датчиков (включая LM75, LM75A, TCN75A, DS75):
        #
        #     Регистры TOS и THYST — 9-битные,
        #     Младшие 7 бит всегда = 0,
        #     Старшие 9 бит (D15–D7) содержат значение в two’s complement,
        #     LSB = 0.5°C.
        return 0.5

    # IBaseSensorEx:
    def get_conversion_cycle_time(self) -> int:
        """Возвращает время в мс преобразования сигнала в цифровой код и готовности его для чтения по шине!
        Для текущих настроек датчика. При изменении настроек следует заново вызвать этот метод!"""
        return 100

    def get_supported_threshold_range(self) -> tuple[float, float]:
        return -55.0, 125.0

# =========================================================================
# Основная реализация (полная логика современных чипов)
# =========================================================================
class TMP75(LM75LikeBase):
    """Драйвер для TMP75/TMP175/TMP275 и современных LM75A/B/C/D.
    Регистровая карта, probing и динамическое разрешение реализованы здесь.
    """
    COMP_MODE_INVERTED: bool = False
    # Регистр конфигурации идентичен TMP75 (Table 8 datasheet)
    CONFIG_REG_TMP75 = (
        # разрядность преобразователя: 0 - 9 бит; 1 - 10 бит; 2 - 11 бит; 3 - 12 бит
        bit_field_info(name=LM75LikeBase.BF_NAME_CONV_RESOL, position=range(5, 7), valid_values=None, description=None),
        # 1 - режим однократного измерения температуры с переходом в состояние выключено
        bit_field_info(name=LM75LikeBase.BF_NAME_ONE_SHOT, position=range(7, 8), valid_values=None, description=None)
    )

    def __init__(self, adapter: I2cAdapter, address: int):
        super().__init__(adapter, address)
        self._config_fields = BitFields((LM75LikeBase.config_reg_LM75, self.CONFIG_REG_TMP75))
        self._probe_hardware_capabilities()  # Аппаратная проверка R1/R0 + OS

        # оригинальные LM75, без битов управления разрешением АЦП температуры, отбрасываю!
        if not self._resol_changeable and type(self) is TMP75:
            raise RuntimeError("Обнаружен устаревший LM75 (без битов R1/R0). Поддерживаются только современные LM75A/B/C/D и TMP75!")

    def get_resolution_code(self) -> int:
        """
        Возвращает текущий код разрешения АЦП (0..3).
        0 = 9 бит, 1 = 10 бит, 2 = 11 бит, 3 = 12 бит.
        Читаёт напрямую из железа, минуя кэш.
        """
        return (self.get_config_field(read_from_cache=False) >> 5) & 0x03

    def _get_shift(self) -> int:
        """Возвращает битовый сдвиг для текущего разрешения (7 для 9 бит, 4 для 12 бит)."""
        return 7 - self.get_resolution_code()

    def get_current_lsb(self) -> float:
        return 0.5 * 2 ** -self.get_resolution_code()

    def get_threshold_lsb(self) -> float:
        return self.get_current_lsb()

    def get_typical_accuracy(self) -> float:
        """Возвращает типичную погрешность датчика во всём рабочем диапазоне."""
        return 1.0  # согласно Table 6.5 TMP75

    def celsius_to_raw(self, celsius: float, threshold: bool = False) -> int:
        return int(round(celsius / self.get_current_lsb())) << self._get_shift()

    def raw_to_celsius(self, raw: int, threshold: bool = False) -> float:
        return (raw >> self._get_shift()) * self.get_current_lsb()

    def get_conversion_cycle_time(self) -> int:
        return 28 * (1 << self.get_resolution_code())

    def get_supported_threshold_range(self) -> tuple[float, float]:
        return -40.0, 125.0

    def set_resolution(self, code: int | None = None) -> int:
        """Устанавливает или возвращает разрешение АЦП (0=9бит, 1=10, 2=11, 3=12)."""
        _offs = 0
        if code is None:
            return _offs + self.get_resolution_code()
        check_value(code, range(4), "Неверное значение кода разрешения АЦП температуры!")
        self.refresh_config_cache()
        self.set_config_field(value=code, field_name=LM75LikeBase.BF_NAME_CONV_RESOL)
        self.set_get_config(value=self.get_config_field())
        return _offs + code

# =========================================================================
# пустые наследники для маркетинговых названий
# =========================================================================
class TMP175(TMP75): pass
class TMP275(TMP75): pass
class TMP75A(TMP75): pass
class TMP75B(TMP75): pass
class TMP75C(TMP75): pass

class LM75A(TMP75): pass
class LM75B(TMP75): pass
class LM75C(TMP75): pass
class LM75D(TMP75): pass


class TMP102(TMP75):
    """
    Программное представление для TMP102 — LM75-совместимый датчик с фиксированным 12-битным разрешением.

    Особенности:
        - Разрешение: 12 бит
        - Точность: +/- 0.5 °C в диапазоне(-40…+125 °C)
        - Время конвертации: ~26 мс
        - One-Shot: поддерживается (бит OS)
        - Биты R1/R0 в регистре конфиг.: отсутствуют (зарезервированы)
    """

    def __init__(self, adapter: I2cAdapter, address: int):
        super().__init__(adapter, address)
        # _resol_changeable = False
        # One-Shot поддерживается аппаратно
        self._one_shot_mode_support = True

    def get_typical_accuracy(self) -> float:
        return 0.5  # +/- 0.5 °C согласно даташиту

    def get_current_lsb(self) -> float:
        return 0.0625  # разрешение 12-бит

    def get_threshold_lsb(self) -> float:
        return self.get_current_lsb()  # пороги температуры в том же формате

    def get_conversion_cycle_time(self) -> int:
        return 26  # Фиксированное время конвертации (~26 мс)

    def get_resolution_code(self) -> int:
        """TMP102 всегда работает в 12-битном режиме."""
        return 3  # Код 12 бит

    def set_resolution(self, code: int | None = None) -> int:
        """TMP102 не поддерживает изменение разрешения."""
        if code is not None:
            raise NotImplementedError("TMP102 имеет фиксированное разрешение 12 бит")
        return self.get_resolution_code()


class PerformanceMode:
    """Режимы производительности для TMP11X/TMP119."""
    ULTRA_FAST      = const(0)  # ~16 мс   (1 отсчет,   15.5 мс)
    HIGH_SPEED      = const(1)  # ~125 мс  (1 отсчет,   125 мс)
    BALANCED        = const(2)  # ~1.0 с   (8 отсчетов, 125 мс)
    HIGH_ACCURACY   = const(3)  # ~8.0 с   (32 отсчет,  250 мс)
    MAX_ACCURACY    = const(4)  # ~32 с    (64 отсчет,  500 мс)