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
            // XA方向の動作（スクロール判定用）
            data->dy_a = event->value;
            goto check_sync;
        }
    }

    // --- センサーB (手前側面) の処理 ---
    if (event->dev == config->sensor_bottom) {
        if (event->code == INPUT_REL_Y) {
            // YB方向の動作を全体Y（縦移動）
            // ★縦移動が逆なら - をつける（または消す）
            event->value = -event->value; 
            return ZMK_INPUT_PROC_CONTINUE;
        } else if (event->code == INPUT_REL_X) {
            // XB方向の動作（スクロール判定用）
            data->dy_b = event->value;
            goto check_sync;
        }
    }

    return ZMK_INPUT_PROC_CONTINUE;

check_sync:
    if (is_synchronized(data->dy_a, data->dy_b, param1)) {
        event->code = INPUT_REL_WHEEL;
        int16_t avg = (data->dy_a + data->dy_b) / 2;
        // ★スクロール方向が逆なら - をつける（または消す）
        event->value = -(avg / (int16_t)(param2 > 0 ? param2 : 1));

        data->dy_a = 0;
        data->dy_b = 0;
        return ZMK_INPUT_PROC_CONTINUE;
    }

    return ZMK_INPUT_PROC_STOP;
}