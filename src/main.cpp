/*
    Main.cpp

    Created on: July 31, 2023

    Author: Coskun ERGAN
*/

#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/device.h>

#include <zpp.hpp>
#include <chrono>
#include "printf_io.h"
#include "button.h"
#include "buzzer.h"
#include "encoder.h"

#define STACK_SIZE (1024 + CONFIG_TEST_EXTRA_STACK_SIZE)

using namespace zpp;
using namespace std::chrono;
using namespace device_printf;
using namespace device_button;
using namespace device_buzzer;
using namespace device_encoder;

#define NUM_THREADS 3

mutex enc_mutex;
mutex btn_mutex;

namespace
{
    ZPP_THREAD_STACK_ARRAY_DEFINE(tstack, NUM_THREADS, STACK_SIZE);
    thread_data tcb[NUM_THREADS];
    thread t[NUM_THREADS];
}

void sensor_task(int my_id) noexcept
{
    struct sensor_value temperature;
    struct sensor_value humidity;
    const struct device *const dht22 = DEVICE_DT_GET_ONE(aosong_dht);
    if(!device_is_ready(dht22))
    {
        printf("\rDevice %s is not ready", dht22->name);
        return;
    }
    for(;;)
    {
        int rc = sensor_sample_fetch(dht22);
        if(rc == 0)
        {
            rc = sensor_channel_get(dht22, SENSOR_CHAN_AMBIENT_TEMP,
                                    &temperature);
            if(rc == 0)
            {
                rc = sensor_channel_get(dht22, SENSOR_CHAN_HUMIDITY,
                                        &humidity);
            }
            printf_io.turn_off_bl_enable();
            printf("\rTemp: %.1f RH: %.1f %%RH  ", sensor_value_to_double(&temperature), sensor_value_to_double(&humidity));
        }
        this_thread::sleep_for(1000ms);
    }
}

void btn_task(int my_id) noexcept
{
    unique_lock lk(btn_mutex);

    button.set_debounce_time(100ms);

    for(;;)
    {
        button.event_cv().wait(lk);
        buzzer.beep(20ms);
        printf_io.turn_off_bl_enable();
        printf("\rButton : %d Pressed.  ", button.get_id());
    }
}

void enc_task(int my_id) noexcept
{
    unique_lock lk(enc_mutex);

    for(;;)
    {
        encoder.event_cv().wait(lk);
        buzzer.beep(5ms);
        printf_io.turn_off_bl_enable();
        printf("\rEnc : %d  ", encoder.get_count());
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

    for(int i = 0; i < NUM_THREADS; i++)
    {
        t[i].join();
    }

    return 0;
}
