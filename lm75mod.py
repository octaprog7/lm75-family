# MIT license

from lm75_family_constants import (
    # Базовые коды и таблица LSB
    RES_9BIT, RES_10BIT, RES_11BIT, RES_12BIT,
    RES_LSB_TABLE,

    # LM75 (Legacy, фиксированный)
    LM75_RESOLUTION_CODE, LM75_LSB,
    LM75_ACCURACY_TYP, LM75_ACCURACY_MAX,
    LM75_TEMP_MIN, LM75_TEMP_MAX,

    # LM75A / TMP75 (Настраиваемый)
    TMP75_ACCURACY_TYP, TMP75_ACCURACY_MAX,
    TMP75_TEMP_MIN, TMP75_TEMP_MAX,

    # TMP102 (Фиксированный 12 бит)
    TMP102_RESOLUTION_CODE, TMP102_LSB,
    TMP102_ACCURACY_TYP, TMP102_ACCURACY_MAX,
    TMP102_TEMP_MIN, TMP102_TEMP_MAX,
    resolution_to_lsb
)

from micropython import const
from sensor_pack_2.bus_service import I2cAdapter
from sensor_pack_2.base_sensor import DeviceEx, IBaseSensorEx, Iterator, check_value # , check_value_ex
from sensor_pack_2.bitfield import BitFields, bit_field_info, make_namedtuple
from sensor_pack_2.comp_interface import ICompInterface # , CompMode


class ISensorPowerControl:
    """
    Интерфейс управления энергопотреблением датчика.
    """
    def set_shutdown(self, value: bool | None = None, read_from_cache: bool = False) -> bool:
        """
        Управляет режимом энергосбережения датчика.

        Args:
            value: True -> включить Shutdown. False -> активный режим.
                   Если None -> вернуть текущее состояние без изменений.
            read_from_cache: Если Истина, то возвращает значение из местного кэша, иначе читает из датчика
        Returns:
            bool: Текущее состояние (True = Shutdown, False = Active).
        """
        raise NotImplementedError


class ILM75Sensor:
    """
        Абстрактный интерфейс для датчиков семейства LM75/TMP75/TMP102/TMP117.
        Определяет базовый интерфейс для чтения температуры и конвертации raw-данных.
        Управление компаратором вынесено в ICompInterface
    """

    def get_typical_accuracy(self) -> float:
        """
            Возвращает типовую точность датчика в градусах Цельсия (±°C).
            Отражает максимальное ожидаемое отклонение от истинной температуры
            в номинальных условиях эксплуатации (согласно даташиту).

            Returns:
                float: Типовая точность (например, 0.1 для ±0.1°C).
        """
        raise NotImplementedError

    def get_current_lsb(self) -> float:
        """
            Возвращает вес младшего значащего бита (LSB) регистра температуры в °C.
            Определяет разрешающую способность преобразования АЦП датчика.

            Returns:
                float: Значение LSB в градусах Цельсия (например, 0.0078125).
        """
        raise NotImplementedError

    def get_threshold_lsb(self) -> float:
        """
            Возвращает вес LSB для регистров температурных порогов в °C.
            Обычно совпадает с get_current_lsb(), но вынесен отдельно для датчиков,
            у которых разрешение порогов отличается от разрешения измерений.

            Returns:
                float: Значение LSB порогов в градусах Цельсия.
        """
        raise NotImplementedError

    def celsius_to_raw(self, celsius: float, threshold: bool = False) -> int:
        """
            Преобразует температуру из градусов Цельсия в сырое целочисленное
            значение, готовое для записи в регистры датчика.

            Args:
                celsius (float): Температура в градусах Цельсия.
                threshold (bool): Если True, применяется форматирование/масштабирование,
                    специфичное для регистров порогов (если отличается от основного).

            Returns:
                int: Целочисленное raw-значение (обычно 16-битное дополнение до двух).
        """
        raise NotImplementedError

    def raw_to_celsius(self, raw: int, threshold: bool = False) -> float:
        """
            Преобразует сырое целочисленное значение из регистра датчика
            в температуру в градусах Цельсия.

            Args:
                raw (int): Сырое значение из регистра (может быть со знаком).
                threshold (bool): Указывает, что значение взято из регистра порогов.
                Влияет на логику расширения знака или масштабирования, если требуется.

            Returns:
                float: Температура в градусах Цельсия.
        """
        raise NotImplementedError

class LM75LikeBase(ILM75Sensor, IBaseSensorEx, ICompInterface, ISensorPowerControl, Iterator):
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
    #   Бит = 0 -> Comparator mode (термостат)
    #   Бит = 1 -> Interrupt mode (прерывание)
    #
    # TMP117, TMP119 (бит 4 регистра Config, поле T/N_A):
    #   Бит = 0 -> Alert mode (прерывание) ИНВЕРСИЯ!
    #   Бит = 1 -> Therm mode (термостат) ИНВЕРСИЯ!
    #
    # Этот флаг указывает, нужно ли инвертировать универсальные значения
    # CompMode.COMPARATOR/INTERRUPT при записи в регистр конкретного датчика.
    # Для LM75 и т. п.: CompMode.COMPARATOR (0) -> регистр конфигурации, бит 0 = 0
    # Для TMP117, TMP119: CompMode.COMPARATOR (0) -> регистр конфигурации, бит 4 = 1 ⚠(инверсия!)
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
            LM75:  _convert_comp_mode(0) -> 0 (без инверсии)
            TMP117: _convert_comp_mode(0) -> 1 (инверсия!)
        """
        if self.COMP_MODE_INVERTED:
            return 1 - mode  # Инверсия для TMP117/TMP119
        return mode  # Без изменений для LM75X, TMP75X

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

        # 1. Читаем текущее состояние (bytes[0] -> int)
        original_cfg = conn.read_reg(addr, 1)[0]

        try:
            # 2. Тест: устанавливаем биты R1/R0 в 1 (код 3 -> 12 бит)
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

    def set_config(self, value: int | None, bytes_count: int = None) -> int | None:
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

    def start_measurement(self, one_shot: bool = False):
        """
        Запуск процесса измерений.

        Args:
            one_shot (bool): Игнорируется в этом семействе.
                             LM75/TMP75 аппаратно не поддерживают One-Shot.
                             При True выводится предупреждение, запускается Continuous Mode.
        """
        if one_shot:
            print("One-Shot mode not supported! Starting Continuous Mode!")

        # Выход из Shutdown -> Continuous Mode
        self.set_shutdown(value=False)

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

    def get_supported_thresholds(self) -> tuple[float, float]:
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
        return LM75_TEMP_MIN, LM75_TEMP_MAX # уточни по своему даташиту, если нужно





    # Iterator start
    def __next__(self) -> float | None:
        if self.is_single_shot_mode() and not self.set_shutdown(value=None):
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
        raw_cfg = self.set_config(value=None)
        # записываю в кэш сырой конфигурации новое значение
        self.set_config_field(value=raw_cfg)
        return raw_cfg

    def _set_shutdown(self, value: bool):
        """Устанавливает битовое поле 'SHUTDOWN' в value"""
        self.set_config_field(value=value, field_name=self.BF_NAME_SHUTDOWN)

    def _set_one_shot(self, value: bool):
        """Устанавливает битовое поле 'ONE_SHOT' в value"""
        self.set_config_field(value=value, field_name=self.BF_NAME_ONE_SHOT)

    def set_shutdown(self, value: bool | None = None, read_from_cache: bool = False) -> bool:
        """
            Управляет режимом энергосбережения (Shutdown) датчика.

            Аргументы:
                value (bool | None):
                    - None: метод работает как геттер. Возвращает текущее состояние без изменений.
                    - True: включает режим пониженного энергопотребления (остановка АЦП и осциллятора).
                    - False: возвращает датчик в активный режим непрерывных измерений.
                read_from_cache (bool):
                    - True: читать состояние из локального кэша (метод автоматически обновляет кэш, изменяет только бит SHUTDOWN
                  и записывает итоговый конфиг обратно в регистр, не затрагивая остальные настройки.
                - В режиме Shutdown все регистры (пороги, конфигурация) сохраняют значения.
                - Выход из Shutdown происходит мгновенно, но первое измерение требует
                  полного времени конвертации (зависит от текущих настроек AVG/CONV).
                - Соответствует ISensorPowerControl.set_shutdown.
        """
        if value is None:
            # Чтение состояния (геттер)
            return self.get_config_field(self.BF_NAME_SHUTDOWN, read_from_cache=read_from_cache)

        # Запись состояния (сеттер)
        self.refresh_config_cache()
        self._set_shutdown(value=value)

        # Синхронная запись обновлённого конфига в регистр
        self.set_config(value=self.get_config_field())
        return value

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
            self.set_config(value=self.get_config_field())

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
            self.set_config(value=val)
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
            _low, _high = thresholds
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
        return LM75_ACCURACY_TYP

    def get_current_lsb(self) -> float:
        """Возвращает текущий 'вес' младшего бита сырого значения температуры в градусах Цельсия."""
        return LM75_LSB  # 9 бит

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
        return LM75_LSB

    # IBaseSensorEx:
    def get_conversion_cycle_time(self) -> int:
        """Возвращает время в мс преобразования сигнала в цифровой код и готовности его для чтения по шине!
        Для текущих настроек датчика. При изменении настроек следует заново вызвать этот метод!"""
        return 100

    def get_supported_thresholds(self) -> tuple[float, float]:
        return LM75_TEMP_MIN, LM75_TEMP_MAX

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
        """Возвращает текущий код разрешения АЦП (0..3).
        0 = 9 бит, 1 = 10 бит, 2 = 11 бит, 3 = 12 бит.
        Читаёт напрямую из железа, минуя кэш."""
        return self.get_config_field(self.BF_NAME_CONV_RESOL, read_from_cache=False)

    def _get_shift(self) -> int:
        """Возвращает битовый сдвиг для текущего разрешения (7 для 9 бит, 4 для 12 бит)."""
        return 7 - self.get_resolution_code()

    def get_current_lsb(self) -> float:
        return resolution_to_lsb(self.get_resolution_code())

    def get_threshold_lsb(self) -> float:
        return self.get_current_lsb()

    def get_typical_accuracy(self) -> float:
        """Возвращает типичную погрешность датчика во всём рабочем диапазоне."""
        return TMP75_ACCURACY_TYP  # согласно Table 6.5 TMP75

    def celsius_to_raw(self, celsius: float, threshold: bool = False) -> int:
        return int(round(celsius / self.get_current_lsb())) << self._get_shift()

    def raw_to_celsius(self, raw: int, threshold: bool = False) -> float:
        return (raw >> self._get_shift()) * self.get_current_lsb()

    def get_conversion_cycle_time(self) -> int:
        return 28 * (1 << self.get_resolution_code())

    def get_supported_thresholds(self) -> tuple[float, float]:
        return TMP75_TEMP_MIN, TMP75_TEMP_MAX

    def set_resolution(self, code: int | None = None) -> int:
        """Устанавливает или возвращает разрешение АЦП (0=9бит, 1=10, 2=11, 3=12)."""
        _offs = 0
        if code is None:
            return _offs + self.get_resolution_code()
        check_value(code, range(4), "Неверное значение кода разрешения АЦП температуры!")
        self.refresh_config_cache()
        self.set_config_field(value=code, field_name=LM75LikeBase.BF_NAME_CONV_RESOL)
        self.set_config(value=self.get_config_field())
        # после записи обновляю кэш
        # self.refresh_config_cache()
        return _offs + self.get_resolution_code()

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
        return TMP102_ACCURACY_TYP  # +/- 0.5 °C согласно даташиту

    def get_current_lsb(self) -> float:
        return TMP102_LSB  # разрешение 12-бит

    def get_threshold_lsb(self) -> float:
        return self.get_current_lsb()  # пороги температуры в том же формате

    def get_supported_thresholds(self) -> tuple[float, float]:
        return TMP102_TEMP_MIN, TMP102_TEMP_MAX

    def get_conversion_cycle_time(self) -> int:
        return 26  # Фиксированное время конвертации (~26 мс)

    def get_resolution_code(self) -> int:
        """TMP102 всегда работает в 12-битном режиме."""
        return RES_12BIT  # Код 12 бит

    def set_resolution(self, code: int | None = None) -> int:
        """TMP102 не поддерживает изменение разрешения."""
        # Игнорирую запрос на изменение (датчик не поддерживает переключение)
        # Возвращаю всегда 3 (12 бит), что соответствует даташиту TMP102
        return RES_12BIT


class PerformanceMode:
    """Режимы производительности для датчиков, подобных TMP11X."""
    ULTRA_FAST      = const(0)  # наивысшая частота измерений (наименьшая точность)
    HIGH_SPEED      = const(1)  #
    BALANCED        = const(2)  #
    HIGH_ACCURACY   = const(3)  #
    MAX_ACCURACY    = const(4)  # наивысшая точность (наименьшая частота измерений)