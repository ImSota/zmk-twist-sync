#define DT_DRV_COMPAT zmk_feature_twistsync

#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/input/input.h>
#include <drivers/input_processor.h>
#include <stdlib.h> // abs() 用

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(zmk_input_processor_twist_sync, CONFIG_INPUT_LOG_LEVEL);

/* 判定用定数（必要に応じて調整） */
#define SCROLL_THRESHOLD_MIN 100   // これ以下の動きを「微小操作」とみなす
#define CURSOR_BLOCK_LIMIT  10    // 微小操作中、これ以上のカーソル移動軸の動きがあればブロック
#define RATIO_MARGIN        2    // 高速域での比率（XがYの何倍以上必要か）

struct twist_sync_config {
    const struct device *sensor_right; // センサーA
    const struct device *sensor_bottom; // センサーB
};

struct twist_sync_data {
    int16_t dy_a; // スクロール判定用軸 (XA)
    int16_t dy_b; // スクロール判定用軸 (XB)
    int16_t last_y_a; // 直前のカーソル移動軸の値 (YA)
    int16_t last_y_b; // 直前のカーソル移動軸の値 (YB)
};

static bool is_synchronized(int16_t a, int16_t b, int16_t cur_y_a, int16_t cur_y_b, uint32_t threshold) {
    if (a == 0 || b == 0) return false;
    
    // 1. 符号一致（方向）
    if ((a > 0 && b < 0) || (a < 0 && b > 0)) return false;
    
    // 2. 同調精度（param1）
    if (abs(a - b) > (int)threshold) return false;

    // --- 2段構えのガードロジック ---
    int16_t abs_a = abs(a);
    int16_t abs_y_a = abs(cur_y_a);

    if (abs_a <= SCROLL_THRESHOLD_MIN) {
        // 【低速域】カーソル軸がほぼ静止（CURSOR_BLOCK_LIMIT以下）している時だけ許可
        if (abs_y_a > CURSOR_BLOCK_LIMIT || abs(cur_y_b) > CURSOR_BLOCK_LIMIT) return false;
    } else {
        // 【高速域】スクロール軸がカーソル軸より圧倒的に大きい（比率判定）
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
            event->value = -event->value; // 反転設定
            data->last_y_a = event->value; // カーソル軸の値を記録
            return ZMK_INPUT_PROC_CONTINUE;
        } else if (event->code == INPUT_REL_X) {
            data->dy_a = event->value;
            goto check_sync;
        }
    }

    // --- センサーB (手前側面) ---
    if (event->dev == config->sensor_bottom) {
        if (event->code == INPUT_REL_Y) {
            event->value = -event->value; // 反転設定
            data->last_y_b = event->value; // カーソル軸の値を記録
            return ZMK_INPUT_PROC_CONTINUE;
        } else if (event->code == INPUT_REL_X) {
            data->dy_b = event->value;
            goto check_sync;
        }
    }

    return ZMK_INPUT_PROC_CONTINUE;

check_sync:
    // 同調判定（YA, YBの現在の状態を渡す）
    if (is_synchronized(data->dy_a, data->dy_b, data->last_y_a, data->last_y_b, param1)) {
        event->code = INPUT_REL_WHEEL;
        int16_t avg = (data->dy_a + data->dy_b) / 2;
        event->value = -(avg / (int16_t)(param2 > 0 ? param2 : 1));

        // 使用後は全バッファをクリア
        data->dy_a = 0;
        data->dy_b = 0;
        data->last_y_a = 0;
        data->last_y_b = 0;
        return ZMK_INPUT_PROC_CONTINUE;
    }

    return ZMK_INPUT_PROC_STOP;
}

static const struct zmk_input_processor_driver_api twist_sync_driver_api = {
    .handle_event = twist_sync_handle_event,
};

#define TWIST_SYNC_INST(n)                                                                         \
    static struct twist_sync_data twist_sync_data_##n = {0};                                       \
    static const struct twist_sync_config twist_sync_config_##n = {                                \
        .sensor_right = DEVICE_DT_GET(DT_INST_PHANDLE(n, sensor_right)),                           \
        .sensor_bottom = DEVICE_DT_GET(DT_INST_PHANDLE(n, sensor_bottom)),                         \
    };                                                                                             \
    DEVICE_DT_INST_DEFINE(n, NULL, NULL, &twist_sync_data_##n, &twist_sync_config_##n,             \
                        POST_KERNEL, 90, &twist_sync_driver_api);

DT_INST_FOREACH_STATUS_OKAY(TWIST_SYNC_INST)