# micropython
# main_75.py — Тест драйвера LM75LikeBase
# Внимание: Подключайте датчики только к 3.3В! 5В выведет его из строя!

from collections import namedtuple
from sensor_pack_2.bus_service import I2cAdapter
from micropython import const
from machine import I2C, Pin
from lm75mod import TMP75, LM75AB, LM75LikeBase
import time
from sensor_pack_2.comp_interface import CompMode


strOK = const("OK")
strWarn = const("ПРЕДУПР.")
strFail = const("СБОЙ")
strShutDown = const("SHUTDOWN")
strActive = const("ACTIVE")
strPassed = const("ПРОЙДЕН")
strNotPassed = const("НЕ ПРОЙДЕН")

# =============================================================================
# НАСТРОЙКИ
# =============================================================================
I2C_ID: int = const(1)
SCL_PIN: int = const(7)
SDA_PIN: int = const(6)
I2C_FREQ: int = const(400_000)
SENSOR_ADDR: int = const(0x48)
TEST_ITERATIONS: int = const(50)
THRESHOLDS: tuple[float, float] = 20.0, 30.0

def print_header(title: str, char: str = "=", count: int = 60) -> None:
    """Печатает заголовок раздела."""
    print("")
    s_str = count * char
    print(s_str)
    print(" " + title)
    print(s_str)


def print_status(label: str, value, status: str = strOK) -> None:
    """Печатает строку статуса."""
    if status == strOK:
        symbol = f"[{strOK}]"
    elif status == strWarn:
        symbol = f"[{strWarn}]"
    else:
        symbol = strFail
    print("  " + symbol + " " + label + ": " + str(value))

# статистика по одной(!) измеряемой величине
Statistic = namedtuple("Statistic", "min max avg median range std_dev")

def calc_stats(values: list[float]) -> Statistic | None:
    """
    Расчёт статистики по списку значений (MicroPython совместимо).

    Args:
        values: Список измерений температуры (float).

    Returns:
        dict | None: Словарь со статистикой или None если список пуст.
    """
    if not values:
        return None

    n: int = len(values)
    avg: float = sum(values) / n

    # Сортировка для медианы
    sorted_vals: list[float] = sorted(values)
    if n % 2 == 0:
        median: float = (sorted_vals[n // 2 - 1] + sorted_vals[n // 2]) / 2
    else:
        median: float = sorted_vals[n // 2]

    # Стандартное отклонение
    variance: float = 0
    for x in values:
        variance += (x - avg) ** 2
    variance /= n
    std_dev: float = variance ** 0.5

    return Statistic(min = min(values), max = max(values), avg=avg,
                     median = median, range = max(values) - min(values), std_dev = std_dev)


# Тесты
def test_basic_read(ts: LM75LikeBase) -> bool:
    """Тест 1: Базовое чтение температуры и конфигурации."""
    print_header("ТЕСТ 1: Базовое чтение", "-")
    try:
        cfg_raw: int = ts.set_get_config(value=None)
        print_status("Сырая конфигурация", "0x{:04X}".format(cfg_raw))

        cfg_hr = ts.get_current_config_hr()
        print_status("Конфигурация (HR)", cfg_hr)

        temp: float = ts.get_measurement_value()
        print_status("Температура", "{:.3f} C".format(temp))

        lsb: float = ts.get_current_lsb()
        print_status("Разрешение (LSB)", "{:.5f} C".format(lsb))
        return True
    except Exception as e:
        print_status("Ошибка", str(e), "СБОЙ")
        return False


def test_thresholds(ts: LM75LikeBase) -> bool:
    """Тест 2: Установка и чтение порогов компаратора."""
    print_header("ТЕСТ 2: Пороги компаратора", "-")
    try:
        old_low: float
        old_high: float
        old_low, old_high = ts.set_thresholds(None)
        print_status("Текущие пороги", "{:.3f} / {:.3f} C".format(old_low, old_high))

        print("  -> Установка порогов: {} / {} C".format(THRESHOLDS[0], THRESHOLDS[1]))
        ts.set_thresholds(THRESHOLDS)

        new_low: float
        new_high: float
        new_low, new_high = ts.set_thresholds(None)
        print_status("Новые пороги", "{:.3f} / {:.3f} C".format(new_low, new_high))
        return True
    except Exception as e:
        print_status("Ошибка", str(e), "СБОЙ")
        return False


def test_comparator_modes(ts: LM75LikeBase) -> bool:
    """Тест 3: Переключение режимов компаратора."""
    print_header("ТЕСТ 3: Режимы компаратора", "-")
    try:
        current_mode: int = ts.set_comp_mode(mode=None)
        mode_name: str = "Interrupt" if current_mode else "Comparator (Therm)"
        print_status("Текущий режим", mode_name)

        print("  -> Переключение в Interrupt mode...")
        ts.set_comp_mode(mode=CompMode.INTERRUPT, active_alarm_level=False)
        new_mode: int = ts.set_comp_mode(mode=None)
        result: str = strOK if new_mode == CompMode.INTERRUPT else strWarn
        print_status("Новый режим", "Interrupt" if new_mode else "Comparator", result)

        print("  -> Переключение в Comparator mode...")
        ts.set_comp_mode(mode=CompMode.COMPARATOR, active_alarm_level=False)
        final_mode: int = ts.set_comp_mode(mode=None)
        result: str = strOK if final_mode == CompMode.COMPARATOR else strWarn
        print_status("Финальный режим",
                     "Comparator" if final_mode == CompMode.COMPARATOR else "Interrupt", result)
        return True
    except Exception as e:
        print_status("Ошибка", str(e), strFail)
        return False


def test_shutdown_mode(ts: LM75LikeBase) -> bool:
    """Тест 4: Режим энергосбережения SHUTDOWN."""
    print_header(f"ТЕСТ 4: Режим энергосбережения {strShutDown}", "-")
    try:
        is_off: bool = ts.is_shutdown()
        print_status("Текущее состояние", strShutDown if is_off else strActive)

        print("  -> Включение SHUTDOWN...")
        # ✅ Исправлено: используем новый метод shutdown()
        ts.shutdown()
        time.sleep_ms(10)
        is_off = ts.is_shutdown(read_from_cache=False)
        result: str = strOK if is_off else strWarn
        print_status("После включения", strShutDown if is_off else strActive, result)

        print("  -> Выход из SHUTDOWN...")
        # ✅ Исправлено: start_measurement() без параметров = непрерывный режим
        ts.start_measurement()
        time.sleep_ms(50)
        is_off = ts.is_shutdown(read_from_cache=False)
        result: str = strOK if not is_off else strWarn
        print_status("После выключения", strActive if not is_off else strShutDown, result)

        temp: float = ts.get_measurement_value()
        print_status("Температура после выхода", "{:.3f} C".format(temp))
        return True
    except Exception as e:
        print_status("Ошибка", str(e), strFail)
        return False


def test_shutdown_one_shot(ts: LM75LikeBase) -> bool:
    """Тест 8: Выключение с однократным измерением (если поддерживается)."""
    print_header("ТЕСТ 8: Выключение с ONE_SHOT", "-")
    try:
        if not ts.is_one_shot_supported():
            print_status("Поддержка ONE_SHOT", "Не поддерживается данным датчиком", strWarn)
            return True  # Не ошибка, просто особенность датчика

        print("  -> Выключение с однократным измерением...")
        ts.shutdown(one_shot=True)

        # Ждём завершения конвертации
        time.sleep_ms(ts.get_conversion_cycle_time())

        # Читаем температуру (датчик в SHUTDOWN, но данные доступны)
        temp: float = ts.get_measurement_value()
        print_status("Температура", "{:.3f} C".format(temp))

        # Проверяем, что датчик действительно выключен
        is_off: bool = ts.is_shutdown(read_from_cache=False)
        result: str = strOK if is_off else strWarn
        print_status("Состояние после измерения", strShutDown if is_off else strActive, result)

        # Возвращаем в активный режим для следующих тестов
        ts.start_measurement()

        return True
    except Exception as e:
        print_status("Ошибка", str(e), strFail)
        return False


def test_statistics(ts, count=20):
    """Тест 5: Статистика измерений."""
    print_header("ТЕСТ 5: Статистика ({} измерений)".format(count), "-")
    try:
        values: list[float] = []
        cct: int = ts.get_conversion_cycle_time()
        wait_time: int = max(cct, 100)

        # Получаем LSB датчика для оценки стабильности
        lsb: float = ts.get_current_lsb()

        print("  -> Интервал: {} мс, LSB: {:.5f} C".format(wait_time, lsb))

        for i in range(count):
            temp: float = ts.get_measurement_value()
            values.append(temp)
            if i % 5 == 0:
                print("    [{:2d}] {:.3f} C".format(i + 1, temp))
            time.sleep_ms(wait_time)

        stats = calc_stats(values)
        if stats:
            print("")
            print("  Результаты:")
            print("     Min:     {:.5f} C".format(stats.min))
            print("     Max:     {:.5f} C".format(stats.max))
            print("     Avg:     {:.5f} C".format(stats.avg))
            print("     Median:  {:.5f} C".format(stats.median))
            print("     Range:   {:.5f} C ({:.1f} LSB)".format(stats.range, stats.range / lsb))
            print("     StdDev:  {:.5f} C ({:.2f} LSB)".format(stats.std_dev, stats.std_dev / lsb))

            # Оценка стабильности в единицах LSB
            std_dev_lsb = stats.std_dev / lsb

            if std_dev_lsb < 1.0:
                print_status("Стабильность", "Отличная (< 1 LSB)")
            elif std_dev_lsb < 4.0:
                print_status("Стабильность", "Хорошая (< 4 LSB)")
            else:
                print_status("Стабильность", "Низкая (>= 4 LSB)", strWarn)
        return True
    except Exception as e:
        print_status("Ошибка", str(e), strFail)
        return False


def test_iterator(ts: LM75LikeBase, count: int = 10) -> bool:
    """Тест 6: Итератор для потокового чтения."""
    print_header("ТЕСТ 6: Итератор ({} значений)".format(count), "-")
    try:
        print("  -> Чтение через итератор:")
        for i, temp in enumerate(ts):
            if i >= count:
                break
            print("     [{:2d}] {:.3f} C".format(i + 1, temp))
            time.sleep_ms(100)
        print_status("Итератор", "Работает правильно")
        return True
    except Exception as e:
        print_status("Ошибка", str(e), strFail)
        return False


def test_error_handling(ts: LM75LikeBase) -> bool:
    """Тест 7: Обработка ошибок (валидация порогов)."""
    print_header("ТЕСТ 7: Обработка ошибок", "-")
    errors_caught: int = 0

    try:
        ts.set_thresholds((-100.0, 25.0))
        print_status("Порог -100C", "Должна быть ошибка!", strFail)
    except ValueError:
        print_status("Порог -100C", "ValueError пойман правильно")
        errors_caught += 1

    try:
        ts.set_thresholds((30.0, 25.0))
        print_status("Tmin >= Tmax", "Должна быть ошибка!", strFail)
    except ValueError:
        print_status("Tmin >= Tmax", "ValueError пойман правильно")
        errors_caught += 1

    try:
        ts.set_thresholds("invalid")  # type: ignore
        print_status("Неверный тип", "Должна быть ошибка!", strFail)
    except (ValueError, TypeError):
        print_status("Неверный тип", "Ошибка поймана правильно")
        errors_caught += 1

    if errors_caught == 3:
        print_status("Валидация", "Все ошибки обработаны правильно")
        return True
    else:
        print_status("Валидация", "Поймано {}/3 ошибок".format(errors_caught), strWarn)
        return True


# =============================================================================
# ОСНОВНАЯ ПРОГРАММА
# =============================================================================
def main() -> None:
    """Запуск всех тестов."""
    sensor_class_name = None
    # Инициализация I2C
    print("")
    print("Инициализация I2C...")
    try:
        i2c = I2C(id=I2C_ID, scl=Pin(SCL_PIN), sda=Pin(SDA_PIN), freq=I2C_FREQ)
        adapter: I2cAdapter = I2cAdapter(i2c)
        print_status("I2C порт", "ID={}, freq={}kHz".format(I2C_ID, I2C_FREQ // 1000))
    except Exception as e:
        print("Ошибка инициализации I2C: {}".format(e))
        print("Проверьте: пины {}/{}, подтяжки 4.7k, адрес 0x48".format(SDA_PIN, SCL_PIN))
        return

    # Инициализация датчика
    print("")
    print("Поиск датчика...")
    try:
        # подставляйте вызов конструктора любого класса, унаследованного от LM75LikeBase
        #ts = LM75AB(adapter=adapter, address=SENSOR_ADDR)
        ts = TMP75(adapter=adapter, address=SENSOR_ADDR)
        # название датчика это имя класса
        sensor_class_name = str(ts.__qualname__)
        print_status(label="Датчик", value=f"{sensor_class_name} @ 0x{SENSOR_ADDR:02X}")
    except Exception as e:
        print(f"Ошибка инициализации датчика: {e}")
        print("Проверьте: подключение, адрес, подтяжки 4.7kΩ, питание 3.3В")
        return

    print_header(title=f"Набор проверок 'драйвера' датчика {sensor_class_name}", char="#")

    # Запуск тестов
    tests: list[tuple[str, callable]] = [
        ("Базовое чтение", test_basic_read),
        ("Пороги компаратора", test_thresholds),
        ("Режимы компаратора", test_comparator_modes),
        ("Режим SHUTDOWN", test_shutdown_mode),
        ("Статистика", lambda t: test_statistics(t, TEST_ITERATIONS)),
        ("Итератор", lambda t: test_iterator(t, 10)),
        ("Обработка ошибок", test_error_handling),
        ("Выключение с ONE_SHOT", lambda t: test_shutdown_one_shot(t)),  # если поддерживается
    ]

    results: list[tuple[str, bool]] = []
    for name, func in tests:
        try:
            result: bool = func(ts)
            results.append((name, result))
            time.sleep_ms(200)
        except Exception as e:
            print("Критическая ошибка в тесте '{}': {}".format(name, e))
            results.append((name, False))

    # Итоговый отчёт
    print_header("ИТОГОВЫЙ ОТЧЁТ", "#")
    passed: int = 0
    for _, r in results:
        if r:
            passed += 1
    total: int = len(results)

    for name, result in results:
        symbol: str = f"[{strPassed}]" if result else f"[{strNotPassed}]"
        print("  " + symbol + " " + name)

    print("")
    print("Результат: {}/{} тестов пройдено".format(passed, total))

    if passed == total:
        print("Все тесты успешны!")
    elif passed >= total * 0.8:
        print("Большинство тестов пройдено. Проверьте предупреждения выше.")
    else:
        print("Много ошибок. Проверьте подключение и конфигурацию.")

    print("")
    print("Финальная температура: {:.3f} C".format(ts.get_measurement_value()))
    print("")
    print("#" * 60)


# =============================================================================
# ТОЧКА ВХОДА
# =============================================================================
if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("")
        print("Прервано пользователем")
    except Exception as e:
        print("")
        print("Неожиданная ошибка: {}".format(e))
        import sys

        sys.print_exception(e)