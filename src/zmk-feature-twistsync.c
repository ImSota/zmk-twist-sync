#define DT_DRV_COMPAT zmk_feature_twistsync

#include <zephyr/device.h>
#include <zephyr/input/input.h>
#include <drivers/input_processor.h>
#include <stdlib.h> // abs() 用

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(zmk_input_processor_twist_sync, CONFIG_INPUT_LOG_LEVEL);

/* センサーごとのデータを保持する構造体 */
struct twist_sync_config {
    const struct device *sensor_right;
    const struct device *sensor_bottom;
};

/* 状態を保持する構造体（dyのバッファ） */
struct twist_sync_data {
    int16_t dy_right;
    int16_t dy_bottom;
};

/**
 * 同調判定ロジック
 * @param r 右センサーのdy
 * @param b 下センサーのdy
 * @param threshold 許容誤差 (param1)
 */
static bool is_synchronized(int16_t r, int16_t b, uint32_t threshold) {
    if (r == 0 || b == 0) return false;
    // 符号が一致しているか (同じ方向に回転しているか)
    if ((r > 0 && b < 0) || (r < 0 && b > 0)) return false;
    // 差分がしきい値以内か
    return (abs(r - b) <= (int)threshold);
}

/**
 * イベントハンドラ
 */
static int twist_sync_handle_event(const struct device *dev, struct input_event *event,
                                   uint32_t param1, uint32_t param2,
                                   struct zmk_input_processor_state *state) {
    const struct twist_sync_config *config = dev->config;
    struct twist_sync_data *data = dev->data;

    // 相対移動イベント以外は処理を続行 (マウスボタンなど)
    if (event->type != INPUT_EV_REL) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    // --- センサーA (右側面) ---
    if (event->dev == config->sensor_right) {
        if (event->code == INPUT_REL_X) {
            // dx_A はそのまま Cursor X
            return ZMK_INPUT_PROC_CONTINUE;
        } else if (event->code == INPUT_REL_Y) {
            data->dy_right = event->value;
            goto check_sync;
        }
    }

    // --- センサーB (下側面) ---
    if (event->dev == config->sensor_bottom) {
        if (event->code == INPUT_REL_X) {
            // dx_B は座標系定義に基づき Cursor Y に変換
            event->code = INPUT_REL_Y;
            return ZMK_INPUT_PROC_CONTINUE;
        } else if (event->code == INPUT_REL_Y) {
            data->dy_bottom = event->value;
            goto check_sync;
        }
    }

    return ZMK_INPUT_PROC_CONTINUE;

check_sync:
    // 同調判定の実施
    if (is_synchronized(data->dy_right, data->dy_bottom, param1)) {
        // 条件一致：スクロールイベント（WHEEL）に書き換え
        event->code = INPUT_REL_WHEEL;
        // 両方の平均値を算出し、param2（分割数）で感度調整
        int16_t avg = (data->dy_right + data->dy_bottom) / 2;
        event->value = avg / (int16_t)(param2 > 0 ? param2 : 1);

        // 使用したバッファをクリア
        data->dy_right = 0;
        data->dy_bottom = 0;
        
        LOG_DBG("Twist detected: %d", event->value);
        return ZMK_INPUT_PROC_CONTINUE;
    }

    // 同調していない dy イベントは、もう一方のセンサーの値を待つか、
    // あるいはノイズとしてドロップ（STOP）する
    return ZMK_INPUT_PROC_STOP;
}

/* ドライバーAPIの定義 */
static const struct zmk_input_processor_driver_api twist_sync_driver_api = {
    .handle_event = twist_sync_handle_event,
};

/**
 * インスタンス生成マクロ
 */
#define TWIST_SYNC_INST(n)                                                                         \
    static struct twist_sync_data twist_sync_data_##n = {0};                                       \
    static const struct twist_sync_config twist_sync_config_##n = {                                \
        .sensor_right = DEVICE_DT_GET(DT_INST_PHANDLE(n, sensor_right)),                           \
        .sensor_bottom = DEVICE_DT_GET(DT_INST_PHANDLE(n, sensor_bottom)),                         \
    };                                                                                             \
    DEVICE_DT_INST_DEFINE(n,                                                                       \
                        NULL,                                                                      \
                        NULL,                                                                      \
                        &twist_sync_data_##n,                                                      \
                        &twist_sync_config_##n,                                                    \
                        POST_KERNEL,                                                               \
                        CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,                                       \
                        &twist_sync_driver_api);

DT_INST_FOREACH_STATUS_OKAY(TWIST_SYNC_INST)