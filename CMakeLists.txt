# target_sources_ifdef(CONFIG_ZMK_FEATURE_TWISTSYNC, app PRIVATE src/zmk-feature-twistsync.c)

# モジュール用のライブラリとして宣言
zephyr_library()

# ソイスファイルの追加（相対パスを自動解決）
zephyr_library_sources(src/zmk-feature-twistsync.c)

# ZMKのアプリ層（app）にこのライブラリをリンクさせる
zephyr_library_link_libraries(app)