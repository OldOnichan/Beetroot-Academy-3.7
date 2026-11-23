#include <algorithm>
#include <cstdio>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_err.h"

static void timer(ledc_timer_t id, unsigned hz, ledc_timer_bit_t bits) {
    ledc_timer_config_t cfg{};
    cfg.speed_mode = LEDC_LOW_SPEED_MODE;
    cfg.timer_num = id;
    cfg.duty_resolution = bits;
    cfg.freq_hz = hz;
    cfg.clk_cfg = LEDC_AUTO_CLK;
    ESP_ERROR_CHECK(ledc_timer_config(&cfg));
}

static void output(int pin, ledc_channel_t channel, ledc_timer_t id) {
    ledc_channel_config_t cfg{};
    cfg.gpio_num = pin;
    cfg.speed_mode = LEDC_LOW_SPEED_MODE;
    cfg.channel = channel;
    cfg.timer_sel = id;
    ESP_ERROR_CHECK(ledc_channel_config(&cfg));
}

static void duty(ledc_channel_t channel, unsigned value) {
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, value));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, channel));
}
#include "driver/gpio.h"
static void input(gpio_num_t pin) {
    gpio_config_t cfg{};
    cfg.pin_bit_mask = 1ULL << pin;
    cfg.mode = GPIO_MODE_INPUT;
    cfg.pull_up_en = GPIO_PULLUP_ENABLE;
    ESP_ERROR_CHECK(gpio_config(&cfg));
}
static void position(int degrees) {
    constexpr int left_us = 500, right_us = 2500;
    int us = left_us + degrees * (right_us - left_us) / 180;
    duty(LEDC_CHANNEL_0, (us * 4096 + 10000) / 20000);
}
#include "esp_adc/adc_oneshot.h"
static adc_oneshot_unit_handle_t adc;
static void init_adc() {
    adc_oneshot_unit_init_cfg_t unit{};
    unit.unit_id = ADC_UNIT_1;
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit, &adc));
    adc_oneshot_chan_cfg_t cfg{};
    cfg.atten = ADC_ATTEN_DB_12;
    cfg.bitwidth = ADC_BITWIDTH_12;
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc, ADC_CHANNEL_8, &cfg));
}

static int sample() {
    int raw;
    ESP_ERROR_CHECK(adc_oneshot_read(adc, ADC_CHANNEL_8, &raw));
    return raw;
}
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

static void light_control() {
    init_adc();
    adc_cali_handle_t calibration;
    adc_cali_curve_fitting_config_t cfg{};
    cfg.unit_id = ADC_UNIT_1;
    cfg.chan = ADC_CHANNEL_8;
    cfg.atten = ADC_ATTEN_DB_12;
    cfg.bitwidth = ADC_BITWIDTH_12;
    ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&cfg, &calibration));

    while (true) {
        int sum = 0;
        for (int i = 0; i < 8; ++i) sum += sample();
        int mv;
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(calibration, sum / 8, &mv));
        int angle = 10 + (std::clamp(mv, 50, 3000) - 50) * 160 / 2950;
        // Межі імпульсу відповідають крайнім положенням у прикладі уроку.
        int pulse_us = 500 + (angle - 10) * 2000 / 160;
        duty(LEDC_CHANNEL_0, (pulse_us * 4096 + 10000) / 20000);
        printf("%d mV -> %d deg (%d us)\n", mv, angle, pulse_us);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

enum class Mode { Servo, Safe, Calculator, Light };
constexpr Mode mode = Mode::Servo; // Вибери вправу і перепроший плату.

static int encoder() {
    // Квадратурний декодер: чотири переходи на один фіксований крок.
    static constexpr int direction[] = {0,-1,1,0, 1,0,0,-1, -1,0,0,1, 0,1,-1,0};
    int now = gpio_get_level(GPIO_NUM_4)*2 + gpio_get_level(GPIO_NUM_5);
    static int before = now, count = 0;
    if ((before ^ now) == 3) count = 0;
    count += direction[before*4 + now];
    before = now;
    if (count == 4 || count == -4) {
        int step = count / 4;
        count = 0;
        return step;
    }
    return 0;
}

static bool pressed() {
    static int stable = 1, previous = 1;
    static TickType_t changed = 0;
    int now = gpio_get_level(GPIO_NUM_6);
    if (now != previous) changed = xTaskGetTickCount();
    previous = now;
    if (now != stable && xTaskGetTickCount()-changed >= pdMS_TO_TICKS(25)) {
        stable = now;
        return now == 0;
    }
    return false;
}

static void servo_step(int turn, bool click) {
    static int angle = 90, step = 10;
    static bool sounding = false;
    static TickType_t started;
    if (click) step = (step == 10) ? 5 : 10;
    if (turn) {
        int target = angle + turn * step;
        angle = std::clamp(target, 0, 180);
        position(angle);
        if (target != angle) {
            duty(LEDC_CHANNEL_1, 512);
            sounding = true;
            started = xTaskGetTickCount();
        }
    }
    if (sounding && xTaskGetTickCount()-started >= pdMS_TO_TICKS(100)) {
        duty(LEDC_CHANNEL_1, 0);
        sounding = false;
    }
}

static void safe_step(int turn, bool reset) {
    static constexpr int pin[] = {2, 4, 1};
    static int digits[3]{}, index = 0, direction = 0, attempts = 3;
    static bool open = false;
    if (open || attempts == 0) return;
    if (turn && !reset) {
        if (direction && turn != direction) ++index;
        else digits[index] = (digits[index]+1)%10;
        direction = turn;
        if (index < 3) {
            if (digits[index] == 0 && index > 0) printf(" ");
            printf("\b%d", digits[index]);
            fflush(stdout);
            return;
        }
        open = std::equal(digits, digits+3, pin);
        if (open) {
            position(90);
            puts("\nOPEN");
            return;
        }
    } else if (!reset) return;
    --attempts;
    printf("\nAttempts left: %d\n0", attempts);
    if (!attempts) puts("\nLOCKED until reboot");
    fflush(stdout);
    std::fill(digits, digits+3, 0);
    index = direction = 0;
}

static void calculator_step(int turn, bool click) {
    static int a = 0, b = 0, op = 0, stage = 0, width = 0;
    constexpr char ops[] = "+-*/";
    if (!turn && !click) return;
    for (int i = 0; i < width; ++i) printf("\b \b");
    if (stage == 0) a = std::clamp(a+turn,-9999,9999);
    if (stage == 1) op = (op+turn+4)%4;
    if (stage == 2) b = std::clamp(b+turn,-9999,9999);
    if (click) ++stage;
    if (stage == 0) width = printf("%d",a);
    if (stage == 1) width = printf("%d %c ",a,ops[op]);
    if (stage == 2) width = printf("%d %c %d",a,ops[op],b);
    if (stage == 3) {
        printf("%d %c %d = ",a,ops[op],b);
        switch (op) {
            case 0: printf("%d\n", a+b); break;
            case 1: printf("%d\n", a-b); break;
            case 2: printf("%d\n", a*b); break;
            case 3: if (b) printf("%g\n", double(a)/b); else puts("division by zero"); break;
        }
        stage = a = b = op = 0;
        width = printf("0");
    }
    fflush(stdout);
}

extern "C" void app_main() {
    timer(LEDC_TIMER_0, 50, LEDC_TIMER_12_BIT);
    output(14, LEDC_CHANNEL_0, LEDC_TIMER_0);
    if (mode == Mode::Light) light_control();
    input(GPIO_NUM_4);
    input(GPIO_NUM_5);
    input(GPIO_NUM_6);
    timer(LEDC_TIMER_1, 2000, LEDC_TIMER_10_BIT);
    output(7, LEDC_CHANNEL_1, LEDC_TIMER_1);
    position(mode == Mode::Safe ? 0 : 90);
    if (mode == Mode::Safe || mode == Mode::Calculator) { printf("0"); fflush(stdout); }
    while (true) {
        int turn = encoder();
        bool click = pressed();
        switch (mode) {
            case Mode::Servo: servo_step(turn, click); break;
            case Mode::Safe: safe_step(turn, click); break;
            case Mode::Calculator: calculator_step(turn, click); break;
            case Mode::Light: break;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
