idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES bt esp_hid driver esp_timer nvs_flash)

# Prevent Bluedroid from requesting sniff for HID device.
# bta_dm_cfg.c defaults to BTA_DM_PM_SNIFF5 (intv 10-18) which collides with
# the Switch's sniff (intv 24), causing disconnect. Set to 0 (NO_ACTION) so
# Bluedroid never initiates a PM change and just follows what the Switch sets.
idf_component_get_property(bt_lib bt COMPONENT_LIB)
target_compile_definitions(${bt_lib} PRIVATE
    BTA_DM_PM_SNIFF_HD_ACTIVE_IDX=0
    BTA_DM_PM_SNIFF_HD_IDLE_IDX=0
)
