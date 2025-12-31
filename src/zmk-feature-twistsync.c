#define DT_DRV_COMPAT zmk_feature_twistsync

#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/input/input.h>
#include <drivers/input_processor.h>
#include <stdlib.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(zmk_input_processor_twist_sync, CONFIG_INPUT_LOG_LEVEL);

/* 調整用定数 (256倍固定小数点) */
#define EMA_ALPHA_SHIFT 2     // 反応を速めるために少し小さく設定
#define CURSOR_THRESHOLD 1536 // 6.0 * 256 (誤爆を防ぎつつ移動を許容)
#define SCROLL_THRESHOLD 768  // 3.0 * 256 (スクロールを最優先で発動しやすく)
#define EXIT_THRESHOLD   128  // 0.5 * 256 (より敏感にモードをリセット)
#define SYNC_WINDOW_MS   100  // 余裕を持った同期窓

struct twist_sync_config {
    const struct device *sensor_right;
    const struct device *sensor_bottom;
};

struct twist_sync_data {
    int16_t dy_a; 
    int16_t dy_b;
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
    // 1. 勢いの計算
    // ひねり成分の強化更新
    if ((data->dy_a > 0 && data->dy_b > 0) || (data->dy_a < 0 && data->dy_b < 0)) {
        update_ema(&data->avg_twist, (abs(data->dy_a) + abs(data->dy_b)) / 2);
        // ひねりがある程度強ければ、カーソルモードを即座に阻害する
        if (abs(data->dy_a) > 2 && abs(data->dy_b) > 2) {
            data->not_scroll_mode = false;
        }
    } else {
        // 逆方向または片方のみならひねりEMAを減衰
        data->avg_twist = (data->avg_twist * 3) / 4; 
    }

    // 2. 静止によるリセット
    if (data->avg_cursor < EXIT_THRESHOLD && data->avg_twist < EXIT_THRESHOLD) {
        data->scroll_mode = false;
        data->not_scroll_mode = false;
    }

    // 3. モード確定 (スクロール判定を先にチェック)
    if (!data->not_scroll_mode && data->avg_twist > SCROLL_THRESHOLD) {
        data->scroll_mode = true;
    } 
    if (!data->scroll_mode && data->avg_cursor > CURSOR_THRESHOLD) {
        data->not_scroll_mode = true;
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

    // 物理軸判定
    bool is_phys_x = (event->code == INPUT_REL_X);
    bool is_phys_y = (event->code == INPUT_REL_Y);

    if (is_phys_x) {
        if (is_right) { data->dy_a = val; data->last_time_a = now; }
        else { data->dy_b = val; data->last_time_b = now; }
        process_synchronized_logic(data);
    } else if (is_phys_y) {
        update_ema(&data->avg_cursor, val);
        process_synchronized_logic(data);
        
        // 座標変換
        if (is_right) { event->code = INPUT_REL_X; event->value = -val; }
        else { event->value = -val; }
    }

    // --- 出力制御 ---
    if (data->scroll_mode) {
        if (is_phys_x) {
            event->code = INPUT_REL_WHEEL;
            event->value = -(val / (int16_t)(param2 > 0 ? param2 : 1));
            return ZMK_INPUT_PROC_CONTINUE;
        }
        return ZMK_INPUT_PROC_STOP;
    }

    if (data->not_scroll_mode) {
        if (is_phys_x) return ZMK_INPUT_PROC_STOP;
        return ZMK_INPUT_PROC_CONTINUE;
    }

    // 未確定時：ひねり軸(物理X)のみ止める
    if (is_phys_x) return ZMK_INPUT_PROC_STOP;
    return ZMK_INPUT_PROC_CONTINUE;
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