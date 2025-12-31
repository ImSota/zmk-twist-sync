#define DT_DRV_COMPAT zmk_feature_twistsync

#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/input/input.h>
#include <drivers/input_processor.h>
#include <stdlib.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(zmk_input_processor_twist_sync, CONFIG_INPUT_LOG_LEVEL);

/* 調整用定数 (固定小数点: 値を256倍して保持) */
#define EMA_ALPHA_SHIFT 3     // 追従速度 (小さいほど敏感)
#define CURSOR_THRESHOLD 1024 // 8.0 * 256
#define SCROLL_THRESHOLD 1024 // 4.0 * 256 (発動しやすくするため下方修正)
#define EXIT_THRESHOLD   256  // 1.0 * 256
#define SYNC_WINDOW_MS   50   // 現実的な同期窓

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

    int32_t avg_twist;  // 256倍スケール
    int32_t avg_cursor; // 256倍スケール
};

/* EMA更新: 精度向上のため256倍スケールで計算 */
static void update_ema(int32_t *avg, int16_t new_val) {
    int32_t val_scaled = (int32_t)abs(new_val) << 8; 
    *avg = *avg + ((val_scaled - *avg) >> EMA_ALPHA_SHIFT);
}

static void process_synchronized_logic(struct twist_sync_data *data) {
    // ひねり成分の計算
    int16_t combined_twist = 0;
    
    // 符号一致の確認
    if ((data->dy_a > 0 && data->dy_b > 0) || (data->dy_a < 0 && data->dy_b < 0)) {
        combined_twist = (abs(data->dy_a) + abs(data->dy_b)) / 2;
        update_ema(&data->avg_twist, combined_twist);
    } else if (data->dy_a == 0 || data->dy_b == 0) {
        // 片方しか動いていない時は、その値を弱めに反映（起動を助ける）
        combined_twist = (abs(data->dy_a) + abs(data->dy_b)) / 4;
        update_ema(&data->avg_twist, combined_twist);
    } else {
        // 明確な逆方向なら減衰
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
    // スクロールモードへの移行（カーソルモードでないことが条件）
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

    if (event->code == INPUT_REL_X) {
        if (is_right) {
            data->dy_a = val;
            data->last_time_a = now;
        } else {
            data->dy_b = val;
            data->last_time_b = now;
        }
        
        // 同期判定
        process_synchronized_logic(data);
        
        // スクロール判定軸は一旦止めて同期を待つ
        if (!data->scroll_mode && !data->not_scroll_mode) {
             return ZMK_INPUT_PROC_STOP;
        }
    } else if (event->code == INPUT_REL_Y) {
        update_ema(&data->avg_cursor, val);
        
        if (is_right) {
            event->code = INPUT_REL_X;
            event->value = -val;
        } else {
            event->value = -val;
        }
    }

    // --- 出力フェーズ ---
    // 1. スクロールモード確定時
    if (data->scroll_mode) {
        if (event->code == INPUT_REL_X && (is_right ? (data->dy_a != 0) : (data->dy_b != 0))) {
            event->code = INPUT_REL_WHEEL;
            event->value = -(val / (int16_t)(param2 > 0 ? param2 : 1));
            data->dy_a = 0; data->dy_b = 0;
            return ZMK_INPUT_PROC_CONTINUE;
        }
        return ZMK_INPUT_PROC_STOP; 
    }

    // 2. カーソル移動モード確定時
    if (data->not_scroll_mode) {
        // 物理的なひねり軸(REL_X)由来のイベントだけを捨てる
        // 変換後の横移動（YA由来のREL_X）は通す
        if (event->code == INPUT_REL_X && (is_right ? (data->dy_a != 0) : (data->dy_b != 0))) {
            data->dy_a = 0; data->dy_b = 0;
            return ZMK_INPUT_PROC_STOP;
        }
        return ZMK_INPUT_PROC_CONTINUE;
    }

    // 3. モード未確定時（重要：ここを修正）
    // 物理的な「ひねり軸」のイベントのみ、相方を待つためにSTOPする
    if (event->code == INPUT_REL_X && (is_right ? (data->dy_a != 0) : (data->dy_b != 0))) {
        // 勢いが溜まるまでは出さないが、YA由来の横移動は止めない
        return ZMK_INPUT_PROC_STOP;
    }

    // それ以外（変換後の横移動、縦移動）は常に通す
    return ZMK_INPUT_PROC_CONTINUE;
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