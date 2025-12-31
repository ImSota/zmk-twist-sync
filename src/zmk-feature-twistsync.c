#define DT_DRV_COMPAT zmk_feature_twistsync

#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/input/input.h>
#include <drivers/input_processor.h>
#include <stdlib.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(zmk_input_processor_twist_sync, CONFIG_INPUT_LOG_LEVEL);

/* 調整用定数 (固定小数点: 256倍) */
#define EMA_ALPHA_SHIFT 3
#define CURSOR_THRESHOLD 512  // 2.0 * 256 (カーソル移動を検知しやすく下方修正)
#define SCROLL_THRESHOLD 1024 // 4.0 * 256
#define EXIT_THRESHOLD   256  // 1.0 * 256
#define SYNC_WINDOW_MS   50

struct twist_sync_config {
    const struct device *sensor_right; // センサーA
    const struct device *sensor_bottom; // センサーB
};

struct twist_sync_data {
    int16_t dy_a; // 物理X (ひねり) バッファ
    int16_t dy_b; // 物理X (ひねり) バッファ
    uint32_t last_time_a;
    uint32_t last_time_b;
    
    bool scroll_mode;
    bool not_scroll_mode;

    int32_t avg_twist;
    int32_t avg_cursor;
};

static void update_ema(int32_t *avg, int16_t new_val) {
    int32_t val_scaled = (int32_t)abs(new_val) << 8; 
    *avg = *avg + ((val_scaled - *avg) >> EMA_ALPHA_SHIFT);
}

static void process_synchronized_logic(struct twist_sync_data *data) {
    // ひねり成分の計算
    if ((data->dy_a > 0 && data->dy_b > 0) || (data->dy_a < 0 && data->dy_b < 0)) {
        update_ema(&data->avg_twist, (abs(data->dy_a) + abs(data->dy_b)) / 2);
    } else if (data->dy_a == 0 || data->dy_b == 0) {
        update_ema(&data->avg_twist, (abs(data->dy_a) + abs(data->dy_b)) / 4);
    } else {
        data->avg_twist >>= 1;
    }

    // モード判定
    if (data->avg_cursor < EXIT_THRESHOLD && data->avg_twist < EXIT_THRESHOLD) {
        data->scroll_mode = false;
        data->not_scroll_mode = false;
    }

    if (!data->scroll_mode && data->avg_cursor > CURSOR_THRESHOLD) {
        data->not_scroll_mode = true;
    }
    if (!data->not_scroll_mode && data->avg_twist > SCROLL_THRESHOLD) {
        data->scroll_mode = true;
    }
}

static int twist_sync_handle_event(const struct device *dev, struct input_event *event,
                                   uint32_t param1, uint32_t param2,
                                   struct zmk_input_processor_state *state) {
    const struct twist_sync_config *config = dev->config;
    struct twist_sync_data *data = dev->data;

    if (event->type != INPUT_EV_REL) return ZMK_INPUT_PROC_CONTINUE;

    uint32_t now = k_uptime_get_32();
    int16_t val = event->value;
    bool is_right = (event->dev == config->sensor_right);

    // 物理的な軸に基づいたフラグ
    bool is_physical_x = (event->code == INPUT_REL_X);
    bool is_physical_y = (event->code == INPUT_REL_Y);

    if (is_physical_x) {
        // ひねり軸の処理
        if (is_right) { data->dy_a = val; data->last_time_a = now; }
        else { data->dy_b = val; data->last_time_b = now; }
        
        process_synchronized_logic(data);
    } else if (is_physical_y) {
        // 移動軸の処理
        update_ema(&data->avg_cursor, val);
        process_synchronized_logic(data); // カーソル移動中もモード判定を回す

        if (is_right) {
            event->code = INPUT_REL_X; // 横移動へ変換
            event->value = -val;
        } else {
            event->value = -val; // 縦移動
        }
    }

    // --- 出力フェーズ ---

    // 1. スクロールモード確定時
    if (data->scroll_mode) {
        if (is_physical_x) {
            event->code = INPUT_REL_WHEEL;
            event->value = -(val / (int16_t)(param2 > 0 ? param2 : 1));
            return ZMK_INPUT_PROC_CONTINUE;
        }
        return ZMK_INPUT_PROC_STOP; // スクロール中はカーソル移動軸を殺す
    }

    // 2. カーソルモード確定時
    if (data->not_scroll_mode) {
        if (is_physical_x) return ZMK_INPUT_PROC_STOP; // ひねり軸を殺す
        return ZMK_INPUT_PROC_CONTINUE; // 座標変換済みの移動軸を通す
    }

    // 3. モード未確定時
    if (is_physical_x) return ZMK_INPUT_PROC_STOP; // ひねり軸は相方を待つために止める
    return ZMK_INPUT_PROC_CONTINUE; // 移動軸（変換後のX含む）は常に通す
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