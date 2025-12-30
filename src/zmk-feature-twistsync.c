#define DT_DRV_COMPAT zmk_feature_twistsync

#include <zephyr/device.h>
#include <zephyr/input/input.h>
#include <drivers/input_processor.h>
#include <stdlib.h> // abs() 用

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(zmk_input_processor_twist_sync, CONFIG_INPUT_LOG_LEVEL);

struct twist_sync_config {
    const struct device *sensor_right; // センサーA
    const struct device *sensor_bottom; // センサーB
};

struct twist_sync_data {
    int16_t dy_a;
    int16_t dy_b;
};

static bool is_synchronized(int16_t a, int16_t b, uint32_t threshold) {
    if (a == 0 || b == 0) return false;
    // 符号一致確認
    if ((a > 0 && b < 0) || (a < 0 && b > 0)) return false;
    // 差分がしきい値以内か
    return (abs(a - b) <= (int)threshold);
}

static int twist_sync_handle_event(const struct device *dev, struct input_event *event,
                                   uint32_t param1, uint32_t param2,
                                   struct zmk_input_processor_state *state) {
    const struct twist_sync_config *config = dev->config;
    struct twist_sync_data *data = dev->data;

    if (event->type != INPUT_EV_REL) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    // --- センサーA (右側面) の処理 ---
    if (event->dev == config->sensor_right) {
        if (event->code == INPUT_REL_Y) {
            // YA方向の動作を全体X（横移動）に変換
            event->code = INPUT_REL_X;
            // ★横移動が逆なら - をつける（または消す）
            event->value = -event->value;
            return ZMK_INPUT_PROC_CONTINUE;
        } else if (event->code == INPUT_REL_X) {
            // XA方向の動作をスクロール判定用に保持
            data->dy_a = event->value;
            goto check_sync;
        }
    }

    // --- センサーB (手前側面) の処理 ---
    if (event->dev == config->sensor_bottom) {
        if (event->code == INPUT_REL_Y) {
            // YB方向の動作を全体Y（縦移動）として扱う
            // (既に REL_Y なので code の書き換えは不要)
            // ★縦移動が逆なら - をつける（または消す）
            event->value = -event->value;
            return ZMK_INPUT_PROC_CONTINUE;
        } else if (event->code == INPUT_REL_X) {
            // XB方向の動作をスクロール判定用に保持
            data->dy_b = event->value;
            goto check_sync;
        }
    }

    return ZMK_INPUT_PROC_CONTINUE;

check_sync:
    // 同調判定
    if (is_synchronized(data->dy_a, data->dy_b, param1)) {
        event->code = INPUT_REL_WHEEL;
        int16_t avg = (data->dy_a + data->dy_b) / 2;
        // ★スクロール方向が逆なら - をつける（または消す）
        event->value = -(avg / (int16_t)(param2 > 0 ? param2 : 1));
        // event->value = avg / (int16_t)(param2 > 0 ? param2 : 1);

        data->dy_a = 0;
        data->dy_b = 0;
        return ZMK_INPUT_PROC_CONTINUE;
    }

    // 同調していない XA/XB イベントはドロップ
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