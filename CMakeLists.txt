if (CONFIG_ZMK_FEATURE_TWISTSYNC)
    target_sources(app PRIVATE src/twist_sync.c)
endif()