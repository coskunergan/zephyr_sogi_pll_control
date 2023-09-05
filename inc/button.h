/*
    Button Lib

    Created on: July 26, 2023

    Author: Coskun ERGAN
*/

#pragma once
#include <stdio.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>

#include <zpp.hpp>
#include <zpp/timer.hpp>
#include <chrono>
#include <zpp/thread.hpp>
#include <zpp/fmt.hpp>

namespace device_button
{
    using namespace zpp;
    using namespace std::chrono;

#if !DT_NODE_EXISTS(DT_NODELABEL(buttons))
#error "Overlay for button node not properly defined."
#endif
    static const struct gpio_dt_spec buttons[] =
    {
        DT_FOREACH_PROP_ELEM_SEP(DT_NODELABEL(buttons), gpios,
                                 GPIO_DT_SPEC_GET_BY_IDX, (,))
    };

    static struct gpio_callback button_cb_data[ARRAY_SIZE(buttons)];
    static condition_variable button_cv;
    static milliseconds debounce_time;
    static size_t button_id;

    void timer_callback(timer_base *t);

    class timer_wrapper
    {
    public:
        timer_wrapper() noexcept : tmr(basic_timer(timer_callback)) {};
        basic_timer<void(*)(timer_base *)> tmr;
    };
    timer_wrapper timers[ARRAY_SIZE(buttons)];

    void timer_callback(timer_base *t) noexcept
    {
        for(size_t i = 0; i < ARRAY_SIZE(buttons); i++)
        {
            if(&timers[i].tmr == t)
            {
                gpio_add_callback(buttons[i].port, &button_cb_data[i]);
            }
        }
    }

    void button_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pin) noexcept
    {
        for(size_t i = 0; i < ARRAY_SIZE(buttons); i++)
        {
            if(&button_cb_data[i] == cb)
            {
                button_id = i;
                gpio_remove_callback(buttons[i].port, &button_cb_data[i]);
                timers[i].tmr.start(debounce_time);
            }
        }
        button_cv.notify_all();
    }

    class button
    {
    public:
        button()
        {
            set_debounce_time();
            for(size_t i = 0; i < ARRAY_SIZE(buttons); i++)
            {
                gpio_pin_configure_dt(&buttons[i], GPIO_INPUT);
                gpio_pin_interrupt_configure_dt(&buttons[i], GPIO_INT_EDGE_TO_ACTIVE);
                gpio_init_callback(&button_cb_data[i], button_isr, BIT(buttons[i].pin));
                gpio_add_callback(buttons[i].port, &button_cb_data[i]);
            }
        }
        ~button() = default;
        button(button &&) = delete;
        button &operator=(button &&) = delete;
        button(const button &) = delete;
        button &operator=(const button &) = delete;

        void set_debounce_time(milliseconds ms = 50ms)
        {
            debounce_time = ms;
        }

        milliseconds get_debounce_time()
        {
            return debounce_time;
        }

        size_t get_id()
        {
            return button_id;
        }

        condition_variable &event_cv()
        {
            return button_cv;
        }
    };

    button button;
}
