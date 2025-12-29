#include <zephyr/device.h>
#include <zephyr/input/input.h>
#include <dt-bindings/zmk/input_transform.h>
#include "input_processor.h"

#define LOG_LEVEL CONFIG_INPUT_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(zmk_input_processor_twist_sync);

struct twist_sync_data {
    int16_t dy_a; // センサーAのバッファ
    int16_t dy_b; // センサーBのバッファ
    const struct device *dev_a;
    const struct device *dev_b;
};

// ヘルパー: 同調性の判定ロジック
static bool is_synchronized_twist(int16_t a, int16_t b, uint32_t sensitivity) {
    if (a == 0 || b == 0) return false;
    // 1. 符号が一致しているか (両方CW、または両方CCW)
    if ((a > 0 && b < 0) || (a < 0 && b > 0)) return false;

    // 2. 値が極端に乖離していないか (param1を許容誤差として使用)
    // |a - b| <= sensitivity (この値が小さいほど厳密なひねりが必要)
    return (abs(a - b) <= (int)sensitivity);
}

static int twist_sync_handle_event(const struct device *dev, struct input_event *event,
                                   uint32_t param1, uint32_t param2,
                                   struct zmk_input_processor_state *state) {
    struct twist_sync_data *data = dev->data;

    // 相対移動イベント以外はスルー
    if (event->type != INPUT_EV_REL) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    // --- 軸のマッピングロジック ---
    
    // センサーA (右側面) の処理
    if (event->dev == data->dev_a) {
        if (event->code == INPUT_REL_X) {
            // dx_A はそのまま Cursor X として流す
            return ZMK_INPUT_PROC_CONTINUE;
        } else if (event->code == INPUT_REL_Y) {
            data->dy_a = event->value;
            goto check_sync;
        }
    }

    // センサーB (下側面) の処理
    if (event->dev == data->dev_b) {
        if (event->code == INPUT_REL_X) {
            // dx_B は Cursor Y に変換して流す
            event->code = INPUT_REL_Y;
            return ZMK_INPUT_PROC_CONTINUE;
        } else if (event->code == INPUT_REL_Y) {
            data->dy_b = event->value;
            goto check_sync;
        }
    }

    return ZMK_INPUT_PROC_CONTINUE;

check_sync:
    // 両方のセンサーの dy が揃ったか判定
    if (is_synchronized_twist(data->dy_a, data->dy_b, param1)) {
        // ひねりと判定: イベントを Wheel に書き換える
        event->code = INPUT_REL_WHEEL;
        // 平均値をスクロール量とする。param2で倍率調整
        event->value = (data->dy_a + data->dy_b) / 2;
        if (param2 > 0) event->value /= param2; 

        // バッファクリア
        data->dy_a = 0;
        data->dy_b = 0;
        return ZMK_INPUT_PROC_CONTINUE;
    } 

    // 片方しか動いていない、あるいは逆方向に動いている場合は、
    // 誤爆防止のため現在の dy イベントを消費(STOP)して隠す
    // ※ 完全に揃うまで待機する設計
    return ZMK_INPUT_PROC_STOP;
}

static const struct zmk_input_processor_driver_api twist_sync_api = {
    .handle_event = twist_sync_handle_event,
};

// 初期化時にデバイスツリーからセンサーのphandleを取得する設定が必要
// ここでは簡略化のため、最初のイベント時にデバイスを記憶する実装例としています