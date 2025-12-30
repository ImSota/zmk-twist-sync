#define DT_DRV_COMPAT zmk_feature_twistsync

#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/input/input.h>
#include <drivers/input_processor.h>
#include <stdlib.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(zmk_input_processor_twist_sync, CONFIG_INPUT_LOG_LEVEL);

/* 調整用定数 (固定小数点: 値を8倍して保持) */
#define EMA_ALPHA_SHIFT 2    // 移動平均の重み (小さいほどゆっくり変化)
#define MODE_THRESHOLD  32   // モード確定しきい値 (4.0 * 8)
#define EXIT_THRESHOLD  4    // モード解除しきい値 (0.5 * 8)

struct twist_sync_config {
    const struct device *sensor_right; // センサーA
    const struct device *sensor_bottom; // センサーB
};

struct twist_sync_data {
    int16_t dy_a; // 判定用一時バッファ
    int16_t dy_b;
    
    // 状態ロック用フラグ
    bool scroll_mode;
    bool not_scroll_mode;

    // 移動平均 (EMA) 蓄積変数
    int32_t avg_twist;  // ひねり方向の勢い
    int32_t avg_cursor; // カーソル方向の勢い
};

/* 指数移動平均の更新関数 */
static void update_ema(int32_t *avg, int16_t new_val) {
    int32_t val_scaled = (int32_t)abs(new_val) << 3; // 8倍スケール
    *avg = *avg + ((val_scaled - *avg) >> EMA_ALPHA_SHIFT);
}

static int twist_sync_handle_event(const struct device *dev, struct input_event *event,
                                   uint32_t param1, uint32_t param2,
                                   struct zmk_input_processor_state *state) {
    const struct twist_sync_config *config = dev->config;
    struct twist_sync_data *data = dev->data;

    if (event->type != INPUT_EV_REL) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    int16_t val = event->value;
    bool is_right = (event->dev == config->sensor_right);

    // 1. 各軸のイベントを仕分け、EMAを更新
    if (event->code == INPUT_REL_Y) {
        // カーソル移動軸 (YA または YB)
        update_ema(&data->avg_cursor, val);
        
        // 座標変換 (センサーAならX、センサーBならY)
        if (is_right) {
            event->code = INPUT_REL_X;
            event->value = -val;
        } else {
            event->value = -val;
        }
    } else if (event->code == INPUT_REL_X) {
        // スクロール判定軸 (XA または XB)
        if (is_right) data->dy_a = val;
        else data->dy_b = val;

        // 同調成分を計算してEMA更新
        if (data->dy_a != 0 && data->dy_b != 0) {
            if ((data->dy_a > 0 && data->dy_b > 0) || (data->dy_a < 0 && data->dy_b < 0)) {
                int16_t sync_val = (abs(data->dy_a) + abs(data->dy_b)) / 2;
                update_ema(&data->avg_twist, sync_val);
            }
        }
    }

    // 2. 状態ロックの判定
    // 動きが止まればモードリセット
    if (data->avg_cursor < EXIT_THRESHOLD && data->avg_twist < EXIT_THRESHOLD) {
        data->scroll_mode = false;
        data->not_scroll_mode = false;
    }

    // モード確定
    if (!data->scroll_mode && data->avg_cursor > MODE_THRESHOLD) {
        data->not_scroll_mode = true;
    }
    if (!data->not_scroll_mode && data->avg_twist > MODE_THRESHOLD) {
        data->scroll_mode = true;
    }

    // 3. モードに基づいたイベントの実行
    if (data->scroll_mode) {
        // スクロールモード中：同調イベント(REL_X同士)のみ通し、それ以外（カーソル移動）は封印
        if (event->code == INPUT_REL_X && data->dy_a != 0 && data->dy_b != 0) {
            event->code = INPUT_REL_WHEEL;
            int16_t avg = (data->dy_a + data->dy_b) / 2;
            event->value = -(avg / (int16_t)(param2 > 0 ? param2 : 1));
            data->dy_a = 0;
            data->dy_b = 0;
            return ZMK_INPUT_PROC_CONTINUE;
        }
        return ZMK_INPUT_PROC_STOP; 
    }

    if (data->not_scroll_mode) {
        // カーソル移動モード中：
        // 「本来のスクロール軸(REL_X)」から来たイベントのみをブロックする
        if (event->code == INPUT_REL_WHEEL) return ZMK_INPUT_PROC_STOP;
        
        // センサーが本来持っていた物理的な REL_X イベント（ひねり成分）を捨てる
        // ※ 既に座標変換で REL_Y -> REL_X になっているものは通す必要があるため、
        // ここでは「イベントの発生源となった物理コード」をチェックするのが理想ですが、
        // 簡易的には「変換後のコード」ではなく「デバイスごとの役割」で判定します。

        if (is_right && data->dy_a != 0) { // 右センサーのスクロール軸に値がある時
            data->dy_a = 0;
            return ZMK_INPUT_PROC_STOP;
        }
        if (!is_right && data->dy_b != 0) { // 下センサーのスクロール軸に値がある時
            data->dy_b = 0;
            return ZMK_INPUT_PROC_STOP;
        }

        return ZMK_INPUT_PROC_CONTINUE;
    }

    // モード未確定時は全て通す（または慎重に制限する）
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