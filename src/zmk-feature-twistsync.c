#define DT_DRV_COMPAT zmk_feature_twistsync

#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/input/input.h>
#include <drivers/input_processor.h>
#include <stdlib.h> // abs() 用

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(zmk_input_processor_twist_sync, CONFIG_INPUT_LOG_LEVEL);

/* 判定用定数 */
#define SCROLL_THRESHOLD_MIN 5   // これ以下の動きを「微小操作」とみなす
#define CURSOR_BLOCK_LIMIT  2    // これ以上のカーソル移動があればスクロール判定をリセット
#define RATIO_MARGIN        3    // スクロール軸がカーソル軸の何倍必要か

struct twist_sync_config {
    const struct device *sensor_right; // センサーA
    const struct device *sensor_bottom; // センサーB
};

struct twist_sync_data {
    int16_t dy_a; // スクロール判定用軸 (XA)
    int16_t dy_b; // スクロール判定用軸 (XB)
    int16_t last_y_a;
    int16_t last_y_b;
};

static bool is_synchronized(int16_t a, int16_t b, int16_t cur_y_a, int16_t cur_y_b, uint32_t threshold) {
    if (a == 0 || b == 0) return false;
    if ((a > 0 && b < 0) || (a < 0 && b > 0)) return false;
    if (abs(a - b) > (int)threshold) return false;

    int16_t abs_a = abs(a);
    int16_t abs_y_a = abs(cur_y_a);
    int16_t abs_y_b = abs(cur_y_b);

    // 【2段構えガード】
    if (abs_a <= SCROLL_THRESHOLD_MIN) {
        if (abs_y_a > CURSOR_BLOCK_LIMIT || abs_y_b > CURSOR_BLOCK_LIMIT) return false;
    } else {
        if (abs_a < (abs_y_a * RATIO_MARGIN)) return false;
    }

    return true;
}

static int twist_sync_handle_event(const struct device *dev, struct input_event *event,
                                   uint32_t param1, uint32_t param2,
                                   struct zmk_input_processor_state *state) {
    const struct twist_sync_config *config = dev->config;
    struct twist_sync_data *data = dev->data;

    if (event->type != INPUT_EV_REL) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    // --- センサーA (右側面) ---
    if (event->dev == config->sensor_right) {
        if (event->code == INPUT_REL_Y) {
            event->code = INPUT_REL_X;
            event->value = -event->value; 
            data->last_y_a = event->value;

            // 【強制クリア】カーソル移動が発生したら、スクロール用バッファをリセット
            if (abs(event->value) > CURSOR_BLOCK_LIMIT) {
                data->dy_a = 0;
                data->dy_b = 0;
            }
            return ZMK_INPUT_PROC_CONTINUE;
        } else if (event->code == INPUT_REL_X) {
            data->dy_a = event->value;
            goto check_sync;
        }
    }

    // --- センサーB (手前側面) ---
    if (event->dev == config->sensor_bottom) {
        if (event->code == INPUT_REL_Y) {
            event->value = -event->value; 
            data->last_y_b = event->value;

            // 【強制クリア】カーソル移動が発生したら、スクロール用バッファをリセット
            if (abs(event->value) > CURSOR_BLOCK_LIMIT) {
                data->dy_a = 0;
                data->dy_b = 0;
            }
            return ZMK_INPUT_PROC_CONTINUE;
        } else if (event->code == INPUT_REL_X) {
            data->dy_b = event->value;
            goto check_sync;
        }
    }

    return ZMK_INPUT_PROC_CONTINUE;

check_sync:
    if (is_synchronized(data->dy_a, data->dy_b, data->last_y_a, data->last_y_b, param1)) {
        event->code = INPUT_REL_WHEEL;
        int16_t avg = (data->dy_a + data->dy_b) / 2;
        event->value = -(avg / (int16_t)(param2 > 0 ? param2 : 1));

        data->dy_a = 0;
        data->dy_b = 0;
        data->last_y_a = 0;
        data->last_y_b = 0;
        return ZMK_INPUT_PROC_CONTINUE;
    }

    // 同調しなかった場合も、片方のデータが大きすぎる場合はノイズとみなしてクリアを検討
    // 今回は安全のため、単純なSTOPに留めます
    return ZMK_INPUT_PROC_STOP;
}

// (以下、API定義などは変更なし)