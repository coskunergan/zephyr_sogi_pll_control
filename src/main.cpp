/*
    Main.cpp

    Created on: July 31, 2023

    Author: Coskun ERGAN
*/
#include <inttypes.h>
#include <stddef.h>
#include <stdint.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/eeprom.h>
#include <zephyr/drivers/dac.h>
#include <zephyr/drivers/watchdog.h>

#include <zpp.hpp>
#include <chrono>
#include "printf_io.h"
#include "button.h"
#include "buzzer.h"
#include "encoder.h"
#include "adc_io.h"
#include "mc_spll.h"
#include "arm_math.h"

#define EEPROM_SETVAL_OFFSET 0

#define STACK_SIZE (1024 + CONFIG_TEST_EXTRA_STACK_SIZE)

using namespace zpp;
using namespace std::chrono;
using namespace device_printf;
using namespace device_button;
using namespace device_buzzer;
using namespace device_encoder;
using namespace device_adc;
using namespace control;

#define NUM_THREADS 5
#define OFFSET_PHASE 90
#define PULSE_OFF_DEGREE 170
ADC adc;

SPLL Phase;

typedef enum
{
    ac_input = 0,
} adc_t;

mutex enc_mutex;
mutex btn_mutex;

bool set_mode_enable = false;
float set_value;
float measure_value;
uint8_t set_degree = PULSE_OFF_DEGREE;
int menu_timeout;
int wdt_channel_id;
const struct device *const wdt = DEVICE_DT_GET(DT_ALIAS(watchdog0));
int iwdt_count = 0;
#define IWDT_ADC_NUMBER     1
#define IWDT_SENSOR_NUMBER  2
#define IWDT_DISPLAY_NUMBER 4
#define IWDT_CHECK_NUMBER ( IWDT_ADC_NUMBER + IWDT_SENSOR_NUMBER + IWDT_DISPLAY_NUMBER )

const struct gpio_dt_spec supply_pin = GPIO_DT_SPEC_GET_OR(DT_NODELABEL(i2c_eeprom), supply_gpios,
                                       {
                                           0
                                       });

static const struct gpio_dt_spec pulse_pin =
    GPIO_DT_SPEC_GET_OR(DT_NODELABEL(pulse_pin), gpios,
                        {
                            0
                        });


#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)

#if (DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, dac) && \
	DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, dac_channel_id) && \
	DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, dac_resolution))
#define DAC_NODE DT_PHANDLE(ZEPHYR_USER_NODE, dac)
#define DAC_CHANNEL_ID DT_PROP(ZEPHYR_USER_NODE, dac_channel_id)
#define DAC_RESOLUTION DT_PROP(ZEPHYR_USER_NODE, dac_resolution)
#else

#endif

static const struct device *const dac_dev = DEVICE_DT_GET(DAC_NODE);

static const struct dac_channel_cfg dac_ch_cfg =
{
    .channel_id  = DAC_CHANNEL_ID,
    .resolution  = DAC_RESOLUTION,
    .buffered = true
};


namespace
{
    ZPP_THREAD_STACK_ARRAY_DEFINE(tstack, NUM_THREADS, STACK_SIZE);
    thread_data tcb[NUM_THREADS];
    thread t[NUM_THREADS];
}

static const struct device *get_eeprom_device(void)
{
    const struct device *dev = DEVICE_DT_GET(DT_ALIAS(eeprom_0));

    if(!device_is_ready(dev))
    {
        printk("\nError: Device \"%s\" is not ready; "
               "check the driver initialization logs for errors.\n",
               dev->name);
        return NULL;
    }

    printk("Found EEPROM device \"%s\"\n", dev->name);
    return dev;
}

static const struct device *get_ds18b20_device(void)
{
    const struct device *const dev = DEVICE_DT_GET_ANY(maxim_ds18b20);

    if(!dev)
    {
        /* No such node, or the node does not have status "okay". */
        printf("\nError: no device found.\n");
        return NULL;
    }

    if(!device_is_ready(dev))
    {
        printf("\nError: Device \"%s\" is not ready; "
               "check the driver initialization logs for errors.\n",
               dev->name);
        return NULL;
    }

    printf("Found device \"%s\", getting sensor data\n", dev->name);
    return dev;
}

float run_min_max_filter(float val)
{
    static float measure_value_arr[5] = {-99, 99, -99, 99, -99};
    static uint8_t index = 0;
    uint8_t min_index;
    uint8_t max_index;
    uint8_t catch_index = 0x1F;
    uint8_t i, j;
    float temp_min;
    float temp_max;
    measure_value_arr[index] = val;
    index++;
    index %= 5;
    for(j = 0; j < 2; j++)
    {
        //------- find min -------
        for(i = 0; i < 5; i++)
        {
            if(catch_index & (1U << i))
            {
                break;
            }
        }
        temp_min = measure_value_arr[i];
        for(i = 0; i < 5; i++)
        {
            if((catch_index & (1U << i)) && (measure_value_arr[i] < temp_min))
            {
                temp_min = measure_value_arr[i];
                min_index = i;
            }
        }
        catch_index &= ~(1U << min_index);
        //------- find max -------
        for(i = 0; i < 5; i++)
        {
            if(catch_index & (1U << i))
            {
                break;
            }
        }
        temp_max = measure_value_arr[i];
        for(i = 0; i < 5; i++)
        {
            if((catch_index & (1U << i)) && (measure_value_arr[i] > temp_max))
            {
                temp_max = measure_value_arr[i];
                max_index = i;
            }
        }
        catch_index &= ~(1U << max_index);
    }
    for(i = 0; i < 5; i++)
    {
        catch_index >>= 1U;
        if(catch_index == 0)
        {
            break;
        }
    }
    return measure_value_arr[i];
}

void sensor_task(int my_id) noexcept
{
    float measure;
    struct sensor_value temp;
    const struct device *dev = get_ds18b20_device();
    if(dev == NULL)
    {
        return;
    }
    for(;;)
    {
        int rc = sensor_sample_fetch(dev);
        if(rc == 0)
        {
            sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temp);
            //printf("\rTemp: %d.%06d   ", temp.val1, temp.val2);
            temp.val1 = (temp.val1 < 10) ? 10 : (temp.val1 > 45) ? 45 : temp.val1;
            measure = sensor_value_to_double(&temp);
            iwdt_count |= IWDT_SENSOR_NUMBER;
        }
        else
        {
            measure = 88.8;
        }
        measure_value = run_min_max_filter(measure);
        this_thread::sleep_for(2000ms);
        if(iwdt_count == IWDT_CHECK_NUMBER)
        {
            wdt_feed(wdt, wdt_channel_id);
        }
        iwdt_count = 0;
    }
}

void adc_task(int my_id) noexcept
{
    int degree;
    wdt_feed(wdt, wdt_channel_id);
    if(!device_is_ready(dac_dev))
    {
        printk("DAC device %s is not ready\n", dac_dev->name);
        return;
    }
    int ret = dac_channel_setup(dac_dev, &dac_ch_cfg);
    if(ret != 0)
    {
        printk("Setting up of DAC channel failed with code %d\n", ret);
        return;
    }
    gpio_pin_configure_dt(&pulse_pin, GPIO_OUTPUT_INACTIVE);
    Phase.reset();
    this_thread::sleep_for(500ms);
    adc.readAsyncISR(250us, [&](size_t idx, int16_t val)
    {
        switch((adc_t)idx)
        {
            case ac_input:
                Phase.transfer_1phase(val); // ~181uS CM3 32MHz
                degree = ((Phase.phase() / PI) * 180.0f);
                degree += OFFSET_PHASE;
                degree %= 180;
                dac_write_value(dac_dev, DAC_CHANNEL_ID, degree * 11);
                if(!Phase.is_lock())
                {
                    gpio_pin_set_dt(&pulse_pin, false);
                }
                else if(degree > set_degree)
                {
                    gpio_pin_set_dt(&pulse_pin, true);
                }
                else if(degree > PULSE_OFF_DEGREE || degree < set_degree)
                {
                    gpio_pin_set_dt(&pulse_pin, false);
                }
                iwdt_count |= IWDT_ADC_NUMBER;
                break;
            default:
                break;
        }
    });

    PID pid;
    pid.param.kp = 7.5;
    pid.param.ki = 0.15;
    pid.param.kd = 0;
    pid.param.kc = 0.10;
    pid.param.i_min = 0;
    pid.param.i_max = PULSE_OFF_DEGREE;
    pid.reset();
    pid.pi_transfer(-1e+3);

    for(;;)
    {
        this_thread::sleep_for(500ms);
        set_degree = pid.pi_transfer(measure_value - set_value);
    }
}

void btn_task(int my_id) noexcept // ok
{
    unique_lock lk(btn_mutex);

    button.set_debounce_time(100ms);

    for(;;)
    {
        button.event_cv().wait(lk);
        buzzer.beep(20ms);
        printf_io.turn_off_bl_enable();
        set_mode_enable = !set_mode_enable;
        menu_timeout = 50; // ~5sn
    }
}

void enc_task(int my_id) noexcept // ok
{
    unique_lock lk(enc_mutex);

    for(;;)
    {
        encoder.event_cv().wait(lk);
        buzzer.beep(5ms);
        printf_io.turn_off_bl_enable();
        menu_timeout = 50; // ~5sn
    }
}

void display_task(int my_id) noexcept
{
    const struct device *eeprom = get_eeprom_device();
    char buffer[16];
    float temp_set_value;

    if(eeprom == nullptr)
    {
        return;
    }

    if(supply_pin.port)
    {
        gpio_pin_configure_dt(&supply_pin, GPIO_OUTPUT_ACTIVE);
    }

    int rc = eeprom_read(eeprom, EEPROM_SETVAL_OFFSET, &set_value, sizeof(set_value));
    if(rc < 0)
    {
        printk("Error: Couldn't read eeprom: err: %d.\n", rc);
        return;
    }
    temp_set_value = set_value;
    float freq_sum = 0;
    int freq = 0;
    for(;;)
    {
        this_thread::sleep_for(100ms);
        freq_sum -= freq_sum / 10;
        freq_sum += Phase.freq() * Phase.freq();
        freq = (freq == 0) ? 1 : freq;
        freq = (freq + ((freq_sum / 10) / freq)) / 2;
        printf("\rISI: %.1f %%%02d", measure_value, 99 - ((set_degree * 100) / (PULSE_OFF_DEGREE + 1)));
        if(set_mode_enable)
        {
            set_value = (float)encoder.get_count()  / 10;
            set_value = (set_value < 18.0) ? 18.0 : set_value;
            set_value = (set_value > 45.0) ? 45.0 : set_value;
            sprintf(buffer, "\nSET:>%.1f<%c%02d ", set_value, (Phase.is_lock()) ? 'L' : 'x', (uint8_t)(freq - 10));
        }
        else
        {
            encoder.set_count(set_value * 10);
            sprintf(buffer, "\nSET: %.1f %c%02d ", set_value, (Phase.is_lock()) ? 'L' : 'x', (uint8_t)(freq - 10));
        }
        printf(buffer);
        if(temp_set_value != set_value)
        {
            eeprom_write(eeprom, EEPROM_SETVAL_OFFSET, &set_value, sizeof(set_value));
            temp_set_value = set_value;
        }
        //set_degree = PULSE_OFF_DEGREE - ((set_value * 10) - 180);// test
        if(menu_timeout)
        {
            if(--menu_timeout == 0)
            {
                set_mode_enable = false;
            }
        }
        iwdt_count |= IWDT_DISPLAY_NUMBER;
    }
}

int main(void)
{
    int err;
    if(!device_is_ready(wdt))
    {
        printk("%s: device not ready.\n", wdt->name);
        return 0;
    }

    struct wdt_timeout_cfg wdt_config;
    wdt_config.flags = WDT_FLAG_RESET_SOC;
    wdt_config.window.min = 0U;
    wdt_config.window.max = 8000U;

    wdt_channel_id = wdt_install_timeout(wdt, &wdt_config);
    if(wdt_channel_id == -ENOTSUP)
    {
        /* IWDG driver for STM32 doesn't support callback */
        printk("Callback support rejected, continuing anyway\n");
        wdt_config.callback = NULL;
        wdt_channel_id = wdt_install_timeout(wdt, &wdt_config);
    }
    if(wdt_channel_id < 0)
    {
        printk("Watchdog install error\n");
        return 0;
    }

    err = wdt_setup(wdt, WDT_OPT_PAUSE_HALTED_BY_DBG);
    if(err < 0)
    {
        printk("Watchdog setup error\n");
        return 0;
    }

    printf_io.turn_off_bl_enable();
    printf("\rCoskun ERGAN  ");
    printf("\nVersion : 1.1 ");
    this_thread::sleep_for(1000ms);
    buzzer.beep(10ms);

    const thread_attr attrs(
        thread_prio::preempt(10),
        thread_inherit_perms::no,
        thread_suspend::no
    );

    t[0] = thread(tcb[0], tstack(0), attrs, enc_task, 1);
    t[1] = thread(tcb[1], tstack(1), attrs, btn_task, 2);
    t[2] = thread(tcb[2], tstack(2), attrs, sensor_task, 3);
    t[3] = thread(tcb[3], tstack(3), attrs, adc_task, 4);
    t[4] = thread(tcb[4], tstack(4), attrs, display_task, 5);

    for(int i = 0; i < NUM_THREADS; i++)
    {
        t[i].join();
    }

    return 0;
}
