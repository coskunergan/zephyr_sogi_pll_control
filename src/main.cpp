/*
    Main.cpp

    Created on: July 31, 2023

    Author: Coskun ERGAN
*/
#include <inttypes.h>
#include <stddef.h>
#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/eeprom.h>

#include <zpp.hpp>
#include <chrono>
#include "printf_io.h"
#include "button.h"
#include "buzzer.h"
#include "encoder.h"
#include "adc_io.h"

#define EEPROM_SETVAL_OFFSET 0

#define STACK_SIZE (1024 + CONFIG_TEST_EXTRA_STACK_SIZE)

using namespace zpp;
using namespace std::chrono;
using namespace device_printf;
using namespace device_button;
using namespace device_buzzer;
using namespace device_encoder;
using namespace device_adc;

#define NUM_THREADS 5

ADC adc;

typedef enum
{
    ac_input = 0,
} adc_t;

mutex enc_mutex;
mutex btn_mutex;

bool set_mode_enable = false;
float set_value = 35.0;

namespace
{
    ZPP_THREAD_STACK_ARRAY_DEFINE(tstack, NUM_THREADS, STACK_SIZE);
    thread_data tcb[NUM_THREADS];
    thread t[NUM_THREADS];
}

static const struct device *get_eeprom_device(void)
{

    const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(eeprom));
    //const struct device *dev = DEVICE_DT_GET(DT_ALIAS(eeprom_0));

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

    if(dev == NULL)
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

void sensor_task(int my_id) noexcept
{
    struct sensor_value temp;
    const struct device *dev = get_ds18b20_device();
    if(dev == NULL)
    {
        return;
    }
    for(;;)
    {
        sensor_sample_fetch(dev);
        sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temp);
        //printf("\rTemp: %d.%06d   ", temp.val1, temp.val2);
        this_thread::sleep_for(3000ms);
    }
}

void adc_task(int my_id) noexcept
{
    adc.readAsync(1000ms, [&](size_t idx, int16_t val)
    {
        switch((adc_t)idx)
        {
            case ac_input:
                //printf("\rADC: %d   ", val);
                //buzzer.beep(10ms);
                break;
            default:
                break;
        }
    });

    for(;;)
    {
        this_thread::sleep_for(5000ms);
        buzzer.beep(3ms);
        //printf("\rV = %"PRId32" mV\n", adc.get_voltage(0));
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
    int rc = eeprom_read(eeprom, EEPROM_SETVAL_OFFSET, &set_value, sizeof(set_value));
    if(rc < 0)
    {
        printk("Error: Couldn't read eeprom: err: %d.\n", rc);
        return;
    }
    for(;;)
    {
        this_thread::sleep_for(100ms);
        printf("\rISI: 37.7 %%99 ");
        if(set_mode_enable)
        {
            set_value = (float)encoder.get_count()  / 10;
            sprintf(buffer, "\nSET:>%.1f<50L ", set_value);
            if(temp_set_value != set_value)
            {
                eeprom_write(eeprom, EEPROM_SETVAL_OFFSET, &set_value, sizeof(set_value));
                temp_set_value = set_value;
            }
        }
        else
        {
            encoder.set_count(set_value * 10);
            sprintf(buffer, "\nSET: %.1f 50L ", set_value);
        }
        printf(buffer);
    }
}

int main(void)
{
    printf_io.turn_off_bl_enable();
    printf("\rRestart..");
    buzzer.beep();

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
