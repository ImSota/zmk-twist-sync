#define DT_DRV_COMPAT zmk_feature_twistsync

#include <stdint.h>
#include <zephyr/kernel.h> // k_uptime_get_32 用
#include <zephyr/device.h>
#include <zephyr/input/input.h>
#include <drivers/input_processor.h>
#include <stdlib.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(zmk_input_processor_twist_sync, CONFIG_INPUT_LOG_LEVEL);

/* 調整用定数 */
#define EMA_ALPHA_SHIFT 2
#define CURSOR_THRESHOLD 10  // 固定小数点(x8)
#define SCROLL_THRESHOLD 24  // スクロールしやすさを考慮し少し下方修正
#define EXIT_THRESHOLD 4
#define SYNC_WINDOW_MS 10    // AとBのパケットを待つ時間（ミリ秒）

struct twist_sync_config {
    const struct device *sensor_right; // センサーA
    const struct device *sensor_bottom; // センサーB
};

struct twist_sync_data {
    int16_t dy_a; // 蓄積用バッファ
    int16_t dy_b;
    uint32_t last_time_a;
    uint32_t last_time_b;
    
    bool scroll_mode;
    bool not_scroll_mode;

    int32_t avg_twist;
    int32_t avg_cursor;
};

static void update_ema(int32_t *avg, int16_t new_val) {
    int32_t val_scaled = (int32_t)abs(new_val) << 3;
    *avg = *avg + ((val_scaled - *avg) >> EMA_ALPHA_SHIFT);
}

/* QMKスタイルの同期判定ロジック */
static void process_synchronized_logic(struct twist_sync_data *data) {
    // 1. 勢い（EMA）の更新
    // カーソル成分（絶対値の和）
    update_ema(&data->avg_cursor, (abs(data->dy_a) + abs(data->dy_b)) / 2);

    // ひねり成分（符号が一致している場合のみ）
    if ((data->dy_a > 0 && data->dy_b > 0) || (data->dy_a < 0 && data->dy_b < 0)) {
        update_ema(&data->avg_twist, (abs(data->dy_a) + abs(data->dy_b)) / 2);
    } else {
        data->avg_twist >>= 1; // 符号不一致なら急減衰
    }

    // 2. モードロック判定
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

    // --- 同期ウィンドウ処理 ---
    if (event->code == INPUT_REL_X) {
        if (is_right) {
            data->dy_a = val;
            data->last_time_a = now;
        } else {
            data->dy_b = val;
            data->last_time_b = now;
        }

        // AとBの両方のデータが SYNC_WINDOW_MS 以内に届いているか確認
        if (abs((int32_t)data->last_time_a - (int32_t)data->last_time_b) <= SYNC_WINDOW_MS) {
            process_synchronized_logic(data);
        } else {
            // 片方しか届いていない間は判定を保留
            return ZMK_INPUT_PROC_STOP;
        }
    } else if (event->code == INPUT_REL_Y) {
        // Y軸（カーソル移動軸）は即座に座標変換して出力準備
        if (is_right) {
            event->code = INPUT_REL_X;
            event->value = -val;
        } else {
            event->value = -val;
        }
        // カーソル移動があったら一応EMAを更新しておく
        update_ema(&data->avg_cursor, val);
    }

    // --- 出力制御（モード別） ---
    if (data->scroll_mode) {
        if (event->code == INPUT_REL_X && data->dy_a != 0 && data->dy_b != 0) {
            event->code = INPUT_REL_WHEEL;
            int16_t avg = (data->dy_a + data->dy_b) / 2;
            event->value = -(avg / (int16_t)(param2 > 0 ? param2 : 1));
            data->dy_a = 0; data->dy_b = 0; // 送信後にクリア
            return ZMK_INPUT_PROC_CONTINUE;
        }
        return ZMK_INPUT_PROC_STOP;
    }

    if (data->not_scroll_mode) {
        // 物理的なREL_X（ひねり軸）由来のイベントをブロック
        if (event->code == INPUT_REL_X && (is_right ? (data->dy_a != 0) : (data->dy_b != 0))) {
            data->dy_a = 0; data->dy_b = 0;
            return ZMK_INPUT_PROC_STOP;
        }
        return ZMK_INPUT_PROC_CONTINUE;
    }

    // モード未確定時は安全のためREL_Xを止める
    if (event->code == INPUT_REL_X) return ZMK_INPUT_PROC_STOP;

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