target_sources_ifdef(CONFIG_ZMK_FEATURE_TWISTSYNC app PRIVATE src/zmk-feature-twistsync.c)

# モジュールをライブラリとして宣言
##zephyr_library()

# ZMK 本体の include パスをこのモジュールに追加
# これにより <zmk/input_processor.h> が見つかるようになります
#zephyr_library_include_directories(${PYTHON_EXECUTABLE} ${WEST_TOPDIR}/zmk/app/include)

# ソースファイルの追加
#zephyr_library_sources(src/zmk-feature-twistsync.c)