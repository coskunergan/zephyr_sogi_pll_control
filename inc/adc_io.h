/*
    ADC Lib

    Created on: July 23, 2023

    Author: Coskun ERGAN
*/
#pragma once
#include <stdio.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <cassert>
#include <zephyr/irq.h>

#include <zpp.hpp>
#include <zpp/timer.hpp>
#include <chrono>
#include <zpp/thread.hpp>
#include <zpp/fmt.hpp>

namespace device_adc
{

#if defined(ADC)
#undef ADC
#endif


    class ADC
    {
    private:
        const struct device *_dev;
        uint8_t _channel;

        struct adc_channel_cfg _config;
        uint8_t _resolution;

        struct IsrContext
        {
            struct k_work work;
            ADC *self;
            struct adc_sequence_options options;
            struct adc_sequence sequence;
            int16_t buffer;

            int16_t sample;
        } _isrContext;

    public:
        ADC(const struct device *dev, const struct adc_channel_cfg &config, uint8_t resolution)
            : _dev(dev)
            , _config(config)
            , _resolution(resolution)
        {
            int result = 0;
            assert(dev != nullptr);            

            // Configure ADC channel
            result = adc_channel_setup(_dev, &_config);
            assert(result == 0);

            // Configure soft IRQ
            k_work_init(&_isrContext.work, _soft_isr);
            _isrContext.self = this;
        }

        /**
         * Starts the capture of a continuous sequence of samples.
         * @param interval_us Interval in microseconds between each consecutive sample.
         */
        virtual void readAsync(uint32_t interval_us = 0) /*override*/
        {
            _isrContext.options =
            {
                .interval_us = interval_us,
                .callback = _hard_isr,
            };

            _isrContext.sequence =
            {
                .options = &_isrContext.options,
                .channels = BIT(_config.channel_id),
                .buffer = &_isrContext.buffer,
                .buffer_size = sizeof(_isrContext.buffer),
                .resolution = _resolution,
            };

            int res = adc_read(_dev, (adc_sequence*)&_isrContext);
            //assert(res == 0);
        }

        /**
         * Cancel async read and stops the capture, does nothing in case there is
         * no capture in progress.
         */
        virtual void cancelRead() /*override*/
        {
            // TODO: To be implemented
        }

    private:
        static void _soft_isr(struct k_work *work)
        {
            (void)work;
            
        }

        static enum adc_action _hard_isr(const struct device *,
                                         const struct adc_sequence *sequence,
                                         uint16_t sampling_index __unused)
        {
            IsrContext *context = CONTAINER_OF(sequence, IsrContext, sequence);

            // // Store a copy of the conversion buffer
            context->sample = context->buffer;

            // // Forward IRQ to kernel worker thread.
            k_work_submit(&context->work);

            // Continue sampling
            return ADC_ACTION_REPEAT;
        }
    };

}
