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

// ... (ヘッダー・構造体部分は変更なし)

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

    // --- 1. 各軸のイベントを仕分け、EMA（勢い）を更新 ---
    if (event->code == INPUT_REL_Y) {
        // カーソル移動軸の勢いを更新
        update_ema(&data->avg_cursor, val);
        
        // 座標変換
        if (is_right) {
            event->code = INPUT_REL_X; // YA -> 全体X
            event->value = -val;
        } else {
            event->value = -val; // YB -> 全体Y
        }
    } else if (event->code == INPUT_REL_X) {
        // スクロール判定軸 (XA または XB) の値をバッファ
        if (is_right) data->dy_a = val;
        else data->dy_b = val;

        // 両方の軸に値が揃ったら、ひねり方向の勢いを更新
        if (data->dy_a != 0 && data->dy_b != 0) {
            if ((data->dy_a > 0 && data->dy_b > 0) || (data->dy_a < 0 && data->dy_b < 0)) {
                int16_t sync_val = (abs(data->dy_a) + abs(data->dy_b)) / 2;
                update_ema(&data->avg_twist, sync_val);
            }
        }
        // ここで飛ばさず、一旦下のモード判定へ流す
    }

    // --- 2. 状態ロックの判定（QMKロジック） ---
    // 完全に静止したらモードリセット
    if (data->avg_cursor < EXIT_THRESHOLD && data->avg_twist < EXIT_THRESHOLD) {
        data->scroll_mode = false;
        data->not_scroll_mode = false;
    }

    // 勢いに基づいてモードを「ロック」
    if (!data->scroll_mode && data->avg_cursor > MODE_THRESHOLD) {
        data->not_scroll_mode = true;
    }
    if (!data->not_scroll_mode && data->avg_twist > MODE_THRESHOLD) {
        data->scroll_mode = true;
    }

    // --- 3. モードに基づいたイベントの実行制御 ---
    
    // A: スクロールモード確定時
    if (data->scroll_mode) {
        if (event->code == INPUT_REL_X && data->dy_a != 0 && data->dy_b != 0) {
            event->code = INPUT_REL_WHEEL;
            int16_t avg = (data->dy_a + data->dy_b) / 2;
            event->value = -(avg / (int16_t)(param2 > 0 ? param2 : 1));
            data->dy_a = 0;
            data->dy_b = 0;
            return ZMK_INPUT_PROC_CONTINUE;
        }
        return ZMK_INPUT_PROC_STOP; // スクロール中はカーソル移動を遮断
    }

    // B: カーソル移動モード確定時
    if (data->not_scroll_mode) {
        // 元の物理コードが REL_X（ひねり軸）由来なら捨てる
        // 座標変換された後の REL_X (YA由来) は通す必要がある
        if (event->code == INPUT_REL_X && (is_right ? (data->dy_a != 0) : (data->dy_b != 0))) {
            data->dy_a = 0; data->dy_b = 0;
            return ZMK_INPUT_PROC_STOP;
        }
        return ZMK_INPUT_PROC_CONTINUE;
    }

    // C: モード未確定時（中立）
    // ひねり軸（REL_X）単体でのイベントは、相方が来るまで一旦止める
    if (event->code == INPUT_REL_X) {
        return ZMK_INPUT_PROC_STOP;
    }
    
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