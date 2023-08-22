/*
    Buzzer Lib

    Created on: March 13, 2023

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

namespace device_buzzer
{
    using namespace zpp;
    using namespace std::chrono;
#if !DT_NODE_EXISTS(DT_NODELABEL(buzzer))
#error "Overlay for gipo node not properly defined."
#endif

    static const struct gpio_dt_spec buzzer_pin =
        GPIO_DT_SPEC_GET_OR(DT_NODELABEL(buzzer), gpios,
                            {
                                0
                            });

    void timer_callback(timer_base *t) noexcept
    {
        gpio_pin_set_dt(&buzzer_pin, false);
    }

    auto m_t = make_timer(timer_callback);

    class buzzer
    {
    public:
        buzzer()
        {
            gpio_pin_configure_dt(&buzzer_pin, GPIO_OUTPUT_INACTIVE);
        }
        ~buzzer() = default;
        buzzer(buzzer &&) = delete;
        buzzer &operator=(buzzer &&) = delete;
        buzzer(const buzzer &) = delete;
        buzzer &operator=(const buzzer &) = delete;

        void beep(milliseconds ms = 50ms)
        {
            gpio_pin_set_dt(&buzzer_pin, true);
            m_t.start(ms);
        }
    private:
    };

    buzzer buzzer;
}
