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

#include <zpp.hpp>
#include <chrono>
#include "printf_io.h"
#include "button.h"
#include "buzzer.h"
#include "encoder.h"
#include "adc_io.h"

#define STACK_SIZE (1024 + CONFIG_TEST_EXTRA_STACK_SIZE)

using namespace zpp;
using namespace std::chrono;
using namespace device_printf;
using namespace device_button;
using namespace device_buzzer;
using namespace device_encoder;
using namespace device_adc;

#if !DT_NODE_EXISTS(DT_PATH(zephyr_user)) || \
	!DT_NODE_HAS_PROP(DT_PATH(zephyr_user), io_channels)
#error "No suitable devicetree overlay specified"
#endif

#define DT_SPEC_AND_COMMA(node_id, prop, idx) \
 	ADC_DT_SPEC_GET_BY_IDX(node_id, idx),

/* Data of ADC io-channels specified in devicetree. */
static const struct adc_dt_spec adc_channels[] =
{
    DT_FOREACH_PROP_ELEM(DT_PATH(zephyr_user), io_channels, DT_SPEC_AND_COMMA)
};

#define NUM_THREADS 4

ADC my_adc(adc_channels[0].dev, adc_channels[0].channel_cfg, 12);
mutex enc_mutex;
mutex btn_mutex;

namespace
{
    ZPP_THREAD_STACK_ARRAY_DEFINE(tstack, NUM_THREADS, STACK_SIZE);
    thread_data tcb[NUM_THREADS];
    thread t[NUM_THREADS];
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
        printf("\rTemp: %d.%06d   ", temp.val1, temp.val2);
        this_thread::sleep_for(3000ms);
    }
}


void adc_task(int my_id) noexcept
{
    int err;
    uint32_t count = 0;
    uint16_t buf;
    struct adc_sequence sequence =
    {
        .buffer = &buf,
        /* buffer size in bytes, not number of samples */
        .buffer_size = sizeof(buf),
    };

    /* Configure channels individually prior to sampling. */
    // for(size_t i = 0U; i < ARRAY_SIZE(adc_channels); i++)
    // {
    //     if(!adc_is_ready_dt(&adc_channels[i]))
    //     {
    //         printk("ADC controller device %s not ready\n", adc_channels[i].dev->name);
    //         return;
    //     }

    //     err = adc_channel_setup_dt(&adc_channels[i]);
    //     //err = adc_channel_setup(adc_channels[i].dev, &adc_channels[i].channel_cfg);
    //     if(err < 0)
    //     {
    //         printk("Could not setup channel #%d (%d)\n", i, err);
    //         return;
    //     }
    // }
    my_adc.readAsync(1000000, [](int16_t val)
    {
        printf("\rADC: %d   ", val);
    });
    for(;;)
    {
        int32_t val_mv;
        printf("\r");

        // (void)adc_sequence_init_dt(&adc_channels[0], &sequence);

        // err = adc_read(adc_channels[0].dev, &sequence);
        // if(err < 0)
        // {
        //     printk("Could not read (%d)\n", err);
        //     continue;
        // }



        this_thread::sleep_for(5000ms);
        buzzer.beep(3ms);

        /*
         * If using differential mode, the 16 bit value
         * in the ADC sample buffer should be a signed 2's
         * complement value.
         */
        // if(adc_channels[0].channel_cfg.differential)
        // {
        //     val_mv = (int32_t)((int16_t)buf);
        // }
        // else
        // {
        //     val_mv = (int32_t)buf;
        // }
        //printk("%"PRId32, val_mv);
        //adc_raw_to_millivolts_dt(&adc_channels[0], &val_mv);
        //printf(" = %"PRId32" mV\n", val_mv);
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
    //buzzer.beep();

    const thread_attr attrs(
        thread_prio::preempt(10),
        thread_inherit_perms::no,
        thread_suspend::no
    );

    t[0] = thread(tcb[0], tstack(0), attrs, enc_task, 1);
    t[1] = thread(tcb[1], tstack(1), attrs, btn_task, 2);
    t[2] = thread(tcb[2], tstack(2), attrs, sensor_task, 3);
    t[3] = thread(tcb[3], tstack(3), attrs, adc_task, 4);

    for(int i = 0; i < NUM_THREADS; i++)
    {
        t[i].join();
    }

    return 0;
}
