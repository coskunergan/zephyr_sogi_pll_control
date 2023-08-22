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
#define NUM_BUTTON_PINS DT_PROP_LEN(DT_PATH(buttons), gpios)
#if !DT_NODE_EXISTS(DT_NODELABEL(buttons))
#error "Overlay for button node not properly defined."
#endif
    static const struct gpio_dt_spec buttons[] =
    {
        GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(buttons), gpios, 0),
#if NUM_BUTTON_PINS >= 2
        GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(buttons), gpios, 1),
#endif
#if NUM_BUTTON_PINS >= 3
        GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(buttons), gpios, 2),
#endif
#if NUM_BUTTON_PINS >= 4
        GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(buttons), gpios, 3),
#endif
#if NUM_BUTTON_PINS >= 5
        GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(buttons), gpios, 4),
#endif
#if NUM_BUTTON_PINS >= 6
        GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(buttons), gpios, 5),
#endif
    };

    static struct gpio_callback button_cb_data[NUM_BUTTON_PINS];
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
    timer_wrapper timers[NUM_BUTTON_PINS];

    void timer_callback(timer_base *t) noexcept
    {
        for(size_t i = 0; i < NUM_BUTTON_PINS; i++)
        {
            if(&timers[i].tmr == t)
            {
                gpio_add_callback(buttons[i].port, &button_cb_data[i]);
            }
        }
    }

    void button_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pin) noexcept
    {
        for(size_t i = 0; i < NUM_BUTTON_PINS; i++)
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
            for(size_t i = 0; i < NUM_BUTTON_PINS; i++)
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
