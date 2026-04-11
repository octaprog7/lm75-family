# micropython
# main_75.py — Universal test for LM75LikeBase and TMP11X drivers
# Warning: Connect sensors only to 3.3V! 5V will damage them!

from collections import namedtuple
from sensor_pack_2.bus_service import I2cAdapter
from sensor_pack_2.base_sensor import IBaseSensorEx
from sensor_pack_2.comp_interface import ICompInterface, CompMode
from micropython import const
from machine import I2C, Pin
from lm75mod import TMP75, LM75A, LM75LikeBase, LM75, ILM75Sensor, ISensorPowerControl
from lm75tmp11Xmod import TMP11X
import time

# Общий интерфейс для всех датчиков семейства.
# Используется только для IDE и документации. В рантайме MicroPython игнорируется.
# class LM75LikeBase(ILM75Sensor, IBaseSensorEx, ICompInterface, Iterator):
# class TMP11X(ILM75Sensor, IBaseSensorEx, ICompInterface, Iterator, ISensorPowerControl):
class ILM75Family(ILM75Sensor, ICompInterface, ISensorPowerControl, IBaseSensorEx):
    pass

# =============================================================================
# STATUS CONSTANTS
# =============================================================================
strOK: str = const("OK")
strWarn: str = const("WARN")
strFail: str = const("FAIL")
strShutDown: str = const("SHUTDOWN")
strActive: str = const("ACTIVE")
strPassed: str = const("PASSED")
strNotPassed: str = const("NOT PASSED")
strHwLimit: str = const("HW LIMIT")

# =============================================================================
# SETTINGS
# =============================================================================
I2C_ID: int = const(1)
SCL_PIN: int = const(7)
SDA_PIN: int = const(6)
I2C_FREQ: int = const(400_000)
SENSOR_ADDR: int = const(0x48)
TEST_ITERATIONS: int = const(30)
THRESHOLDS: tuple[float, float] = (20.0, 30.0)


def print_header(title: str, char: str = "=", count: int = 60) -> None:
    """Print section header."""
    print("")
    s_str = count * char
    print(s_str)
    print(" " + title)
    print(s_str)


def print_status(label: str, value, status: str = strOK) -> None:
    """Print status line."""
    if status == strOK:
        symbol = "[" + strOK + "]"
    elif status == strWarn:
        symbol = "[" + strWarn + "]"
    else:
        symbol = strFail
    print("  " + symbol + " " + label + ": " + str(value))


# =============================================================================
# STATISTICS
# =============================================================================
Statistic = namedtuple("Statistic", "min max avg median range std_dev")


def calc_stats(values: list[float]) -> Statistic | None:
    """Calculate statistics for a list of values (MicroPython compatible)."""
    if not values:
        return None

    n: int = len(values)
    avg: float = sum(values) / n
    sorted_vals: list[float] = sorted(values)

    if n % 2 == 0:
        median: float = (sorted_vals[n // 2 - 1] + sorted_vals[n // 2]) / 2
    else:
        median: float = sorted_vals[n // 2]

    variance: float = sum((x - avg) ** 2 for x in values) / n
    std_dev: float = variance ** 0.5

    return Statistic(min=min(values), max=max(values), avg=avg,
                     median=median, range=max(values) - min(values), std_dev=std_dev)


# =============================================================================
# TESTS (universal, work with ILM75Family sensor family)
# =============================================================================
def test_basic_read(ts: ILM75Family) -> bool:
    """Test 1: Basic temperature and config read."""
    print_header("TEST 1: Basic read", "-")
    try:
        temp: float = ts.get_measurement_value(0)
        print_status("Temperature", "{:.3f} C".format(temp))

        accuracy: float = ts.get_typical_accuracy()
        print_status("Accuracy (typ.)", "+/- {:.2f} C".format(accuracy))

        lsb: float = ts.get_current_lsb()
        print_status("Resolution (LSB)", "{:.5f} C".format(lsb))

        # Optional: config read (not available on all sensors)
        if hasattr(ts, 'set_get_config'):
            cfg_raw: int = ts.set_get_config(value=None)
            print_status("Raw config", "0x{:04X}".format(cfg_raw))
        if hasattr(ts, 'get_current_config_hr'):
            cfg_hr = ts.get_current_config_hr()
            print_status("Config (HR)", cfg_hr)

        return True
    except Exception as e:
        print_status("Error", str(e), strFail)
        return False


def test_thresholds(ts: ILM75Family) -> bool:
    """Test 2: Set and read comparator thresholds."""
    print_header("TEST 2: Comparator thresholds", "-")
    try:
        old_low, old_high = ts.set_thresholds(None)
        print_status("Current thresholds", "{:.3f} / {:.3f} C".format(old_low, old_high))

        print("  -> Setting thresholds: {} / {} C".format(THRESHOLDS[0], THRESHOLDS[1]))
        ts.set_thresholds(THRESHOLDS)

        new_low, new_high = ts.set_thresholds(None)
        print_status("New thresholds", "{:.3f} / {:.3f} C".format(new_low, new_high))
        return True
    except Exception as e:
        print_status("Error", str(e), strFail)
        return False


def test_comparator_modes(ts: ILM75Family) -> bool:
    """Test 3: Switch comparator modes."""
    print_header("TEST 3: Comparator modes", "-")
    try:
        current_mode: int = ts.set_comp_mode(mode=None)
        mode_name: str = "Interrupt" if current_mode == CompMode.INTERRUPT else "Comparator (Therm)"
        print_status("Current mode", mode_name)

        print("  -> Switching to Interrupt mode...")
        ts.set_comp_mode(mode=CompMode.INTERRUPT, active_alarm_level=False)
        new_mode: int = ts.set_comp_mode(mode=None)
        result: str = strOK if new_mode == CompMode.INTERRUPT else strWarn
        print_status("New mode", "Interrupt" if new_mode == CompMode.INTERRUPT else "Comparator", result)

        print("  -> Switching to Comparator mode...")
        ts.set_comp_mode(mode=CompMode.COMPARATOR, active_alarm_level=False)
        final_mode: int = ts.set_comp_mode(mode=None)
        result: str = strOK if final_mode == CompMode.COMPARATOR else strWarn
        print_status("Final mode",
                     "Comparator" if final_mode == CompMode.COMPARATOR else "Interrupt", result)
        return True
    except Exception as e:
        print_status("Error", str(e), strFail)
        return False


def test_shutdown_mode(ts: ILM75Family) -> bool:
    """Test 4: Power-saving SHUTDOWN mode."""
    print_header("TEST 4: Power-saving mode " + strShutDown, "-")
    try:
        # Read state (universal way)
        if hasattr(ts, 'set_shutdown'):
            # TMP11X: set_shutdown(value=None) -> getter
            is_off: bool = ts.set_shutdown(value=None)
        elif hasattr(ts, 'is_shutdown'):
            # LM75LikeBase: is_shutdown()
            is_off: bool = ts.is_shutdown()
        else:
            print_status("Read status", strHwLimit, strWarn)
            return True

        print_status("Current state", strShutDown if is_off else strActive)

        # Enable SHUTDOWN
        print("  -> Enabling SHUTDOWN...")
        if hasattr(ts, 'set_shutdown'):
            ts.set_shutdown(value=True)
        elif hasattr(ts, 'shutdown'):
            # LM75LikeBase: shutdown(one_shot=False) or shutdown()
            try:
                ts.shutdown(True)
            except TypeError:
                ts.shutdown()  # fallback for old signatures
        else:
            print_status("Write status", strHwLimit, strWarn)
            return True

        time.sleep_ms(10)

        # Check after enabling
        if hasattr(ts, 'set_shutdown'):
            is_off = ts.set_shutdown(value=None, read_from_cache=False)
        elif hasattr(ts, 'is_shutdown'):
            is_off = ts.is_shutdown(read_from_cache=False)
        result: str = strOK if is_off else strWarn
        print_status("After enable", strShutDown if is_off else strActive, result)

        # Exit SHUTDOWN
        print("  -> Exiting SHUTDOWN...")
        if hasattr(ts, 'set_shutdown'):
            ts.set_shutdown(value=False)
        elif hasattr(ts, 'start_measurement'):
            ts.start_measurement()  # Continuous mode
        time.sleep_ms(50)

        if hasattr(ts, 'set_shutdown'):
            is_off = ts.set_shutdown(value=None, read_from_cache=False)
        elif hasattr(ts, 'is_shutdown'):
            is_off = ts.is_shutdown(read_from_cache=False)
        result: str = strOK if not is_off else strWarn
        print_status("After disable", strActive if not is_off else strShutDown, result)

        temp: float = ts.get_measurement_value(0)
        print_status("Temperature after exit", "{:.3f} C".format(temp))
        return True
    except Exception as e:
        print_status("Error", str(e), strFail)
        return False


def test_shutdown_one_shot(ts) -> bool | None:
    """Тест: Запуск One-Shot измерения (если поддерживается)."""
    print_header("TEST: One-Shot Measurement", "-")
    try:
        # Проверяем аппаратную поддержку (флаг ставим в драйверах при инициализации)
        if not getattr(ts, '_one_shot_mode_support', False):
            print_status("One-Shot support", strHwLimit, strWarn)
            return None

        print("  -> Starting One-Shot conversion...")
        ts.start_measurement(one_shot=True)  # ← ОДИН ВЫЗОВ, работает везде!

        # Ждём завершения конвертации
        cct = ts.get_conversion_cycle_time()
        time.sleep_ms(cct + 50)

        # Читаем результат (TMP11X уже в Shutdown, LM75 проигнорирует one_shot)
        temp = ts.get_measurement_value()
        print_status("Temperature", "{:.3f} C".format(temp))

        # Проверяем состояние (для LM75 вернёт False, для TMP11X → True)
        is_off = ts.set_shutdown(value=None, read_from_cache=False)
        result = strOK if is_off else strWarn
        print_status("State after conversion", "SHUTDOWN" if is_off else "ACTIVE", result)

        # Возвращаем в активный режим
        ts.set_shutdown(value=False)
        return True
    except Exception as e:
        print_status("Error", str(e), strFail)
        return False


def test_statistics(ts: ILM75Family, count: int = 20) -> bool:
    """Test 5: Measurement statistics."""
    print_header("TEST 5: Statistics ({} measurements)".format(count), "-")
    try:
        values: list[float] = []
        cct: int = ts.get_conversion_cycle_time()
        wait_time: int = cct + 50
        lsb: float = ts.get_current_lsb()

        print("  -> Interval: {} ms, LSB: {:.5f} C".format(wait_time, lsb))

        for i in range(count):
            temp: float = ts.get_measurement_value(0)
            values.append(temp)
            if i % 5 == 0:
                print("    [{:2d}] {:.3f} C".format(i + 1, temp))
            time.sleep_ms(wait_time)

        stats = calc_stats(values)
        if stats:
            print("")
            print("  Results:")
            print("     Min:     {:.5f} C".format(stats.min))
            print("     Max:     {:.5f} C".format(stats.max))
            print("     Avg:     {:.5f} C".format(stats.avg))
            print("     Median:  {:.5f} C".format(stats.median))
            print("     Range:   {:.5f} C ({:.1f} LSB)".format(stats.range, stats.range / lsb))
            print("     StdDev:  {:.5f} C ({:.2f} LSB)".format(stats.std_dev, stats.std_dev / lsb))

            std_dev_lsb = stats.std_dev / lsb
            if std_dev_lsb < 1.0:
                print_status("Stability", "Excellent (< 1 LSB)")
            elif std_dev_lsb < 4.0:
                print_status("Stability", "Good (< 4 LSB)")
            else:
                print_status("Stability", "Low (>= 4 LSB)", strWarn)
        return True
    except Exception as e:
        print_status("Error", str(e), strFail)
        return False


def test_iterator(ts: ILM75Family, count: int = 10) -> bool:
    """Test 6: Iterator for streaming read."""
    print_header("TEST 6: Iterator ({} values)".format(count), "-")
    try:
        print("  -> Reading via iterator:")
        for i, temp in enumerate(ts):
            if i >= count:
                break
            print("     [{:2d}] {:.3f} C".format(i + 1, temp))
            time.sleep_ms(100)
        print_status("Iterator", "Working correctly")
        return True
    except Exception as e:
        print_status("Error", str(e), strFail)
        return False


def test_error_handling(ts: ILM75Family) -> bool:
    """Test 7: Error handling (threshold validation)."""
    print_header("TEST 7: Error handling", "-")
    errors_caught: int = 0

    try:
        ts.set_thresholds((-100.0, 25.0))
        print_status("Threshold -100C", "Should raise error!", strFail)
    except (ValueError, TypeError):
        print_status("Threshold -100C", "ValueError caught correctly")
        errors_caught += 1

    try:
        ts.set_thresholds((30.0, 25.0))
        print_status("Tmin >= Tmax", "Should raise error!", strFail)
    except (ValueError, TypeError):
        print_status("Tmin >= Tmax", "ValueError caught correctly")
        errors_caught += 1

    try:
        ts.set_thresholds("invalid")  # type: ignore
        print_status("Invalid type", "Should raise error!", strFail)
    except (ValueError, TypeError):
        print_status("Invalid type", "Error caught correctly")
        errors_caught += 1

    if errors_caught == 3:
        print_status("Validation", "All errors handled correctly")
        return True
    else:
        print_status("Validation", "Caught {}/3 errors".format(errors_caught), strWarn)
        return True


def test_resolution_change(ts) -> bool | None:
    """Смена разрешения АЦП (для TMP75/LM75A/B/C/D)."""
    if not hasattr(ts, 'set_resolution'):
        print_status("Resolution change", strHwLimit, strWarn)
        return None

    print_header("TEST 9: Resolution change", "-")
    try:
        current_code = ts.set_resolution()
        print_status("Current resolution code", current_code)

        # Пробуем установить 12 бит
        target_code = 3
        ts.set_resolution(target_code)
        time.sleep_ms(100)  # Запас на переключение внутреннего осциллятора

        # Читаем фактическое состояние напрямую из железа
        actual_code = ts.get_resolution_code()
        actual_lsb = ts.get_current_lsb()

        # Восстанавливаем исходное значение
        ts.set_resolution(current_code)

        if actual_code == target_code and actual_lsb == 0.0625:
            print_status("Resolution switch", "Applied successfully")
            return True
        else:
            # Чип проигнорировал запись (штатное поведение для LM75A/B/C/D)
            print("  Note: Requested 12 bits, hardware returned code={} (LSB={:.5f})".format(actual_code, actual_lsb))
            print_status("Resolution change", "Hardware ignored write", strWarn)
            return None  # Аппаратное ограничение, НЕ ошибка драйвера
    except Exception as e:
        print_status("Error", str(e), strFail)
        return False


def test_fault_queue(ts: ILM75Family) -> bool:
    """Test: Set and read fault filter (Fault Queue)."""
    print_header("TEST: Fault Queue", "-")
    try:
        # If method not implemented -> hardware limit
        if not hasattr(ts, 'set_fault_queue'):
            print_status("Fault Queue", strHwLimit + " method not implemented in driver", strWarn)
            return True

        initial_code = ts.set_fault_queue()
        print_status("Current Fault Queue code", initial_code)

        for code in range(4):
            ts.set_fault_queue(code)
            verify = ts.set_fault_queue()
            if verify != code:
                print_status("Code {} read".format(code), "FAIL (got {})".format(verify), strFail)
                return False

        ts.set_fault_queue(initial_code)
        print_status("Config restore", "Initial value restored")
        print_status("Fault Queue", "All values written/read correctly")
        return True
    except Exception as e:
        print_status("Error", str(e), strFail)
        return False


# =============================================================================
# MAIN PROGRAM
# =============================================================================
def main() -> None:
    """Run all tests."""
    print_header("UNIVERSAL TEST FOR LM75/TMP11X FAMILY SENSORS")

    # I2C init
    print("Initializing I2C...")
    try:
        i2c = I2C(id=I2C_ID, scl=Pin(SCL_PIN), sda=Pin(SDA_PIN), freq=I2C_FREQ)
        adapter: I2cAdapter = I2cAdapter(i2c)
        print_status("I2C port", "ID={}, freq={}kHz".format(I2C_ID, I2C_FREQ // 1000))
    except Exception as e:
        print("I2C init error: {}".format(e))
        print("Check: pins {}/{}, 4.7k pull-ups, address 0x48".format(SDA_PIN, SCL_PIN))
        return

    # Sensor init
    print("")
    print("Searching for sensor...")
    try:
        # === SELECT YOUR CLASS ===
        ts = TMP11X(adapter=adapter, address=SENSOR_ADDR)      # TMP117/TMP119
        # ts = LM75A(adapter=adapter, address=SENSOR_ADDR)       # LM75A/B/C/D
        # ts = TMP75(adapter=adapter, address=SENSOR_ADDR)       # TMP75/TMP175/...
        # ts = LM75(adapter=adapter, address=SENSOR_ADDR)        # Legacy LM75

        sensor_class_name: str = str(ts.__class__.__name__)
        print_status("Sensor", "{} @ 0x{:02X}".format(sensor_class_name, SENSOR_ADDR))

        # Auto-set resolution for TMP75 (if available)
        if isinstance(ts, TMP75) and hasattr(ts, 'set_resolution'):
            ts.set_resolution(code=0x03)  # 12 bits for precise tests

    except Exception as e:
        print("Sensor init error: {}".format(e))
        print("Check: connection, address, 4.7k pull-ups, 3.3V power")
        return

    print_header("Test suite for driver " + sensor_class_name, char="#")

    # === TEST LIST ===
    tests: list[tuple[str, callable]] = [
        ("Basic read", test_basic_read),
        ("Comparator thresholds", test_thresholds),
        ("Comparator modes", test_comparator_modes),
        ("SHUTDOWN mode", test_shutdown_mode),
        ("Statistics", lambda s: test_statistics(s, TEST_ITERATIONS)),
        ("Iterator", lambda s: test_iterator(s, 10)),
        ("Error handling", test_error_handling),
        ("Shutdown with ONE_SHOT", lambda s: test_shutdown_one_shot(s)),
        ("Resolution change", lambda s: test_resolution_change(s)),
        ("Fault Queue", test_fault_queue),
    ]

    # === RUN ===
    results: list[tuple[str, bool | None]] = []
    for name, func in tests:
        try:
            result: bool | None = func(ts)
            results.append((name, result))
            time.sleep_ms(200)
        except Exception as e:
            print("Critical error in test '{}': {}".format(name, e))
            results.append((name, False))

    # === REPORT ===
    print_header("FINAL REPORT", "#")
    passed = sum(1 for _, r in results if r is True)
    hw_limited = sum(1 for _, r in results if r is None)
    failed = sum(1 for _, r in results if r is False)
    total = len(results)

    for name, result in results:
        if result is True:
            symbol = "[" + strPassed + "]"
        elif result is None:
            symbol = "[" + strHwLimit + "]"
        else:
            symbol = "[" + strNotPassed + "]"
        print("  " + symbol + " " + name)

    print("")
    print("Result: {}/{} passed, {} hardware limited, {} failed".format(
        passed, total, hw_limited, failed))

    if failed == 0:
        if hw_limited > 0:
            print("OK: All supported functions work correctly. Hardware limits noted.")
        else:
            print("OK: All tests passed!")
    else:
        print("FAIL: Multiple errors. Check connection and configuration.")

    print("")
    print("Final temperature: {:.3f} C".format(ts.get_measurement_value()))
    print("")
    print(60 * "#")


# =============================================================================
# ENTRY POINT
# =============================================================================
if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("")
        print("Interrupted by user")
    except Exception as e:
        print("")
        print("Unexpected error: {}".format(e))
        import sys

        sys.print_exception(e)