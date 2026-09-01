/* Tracks the connection handle of our own peripheral (advertising) GAP
 * session — updated by nimble_hid_gap_event() below on connect/disconnect.
 * Exposed so callers (e.g. the ble_keyboard component) can force-disconnect
 * the current central device using a clean ble_gap_terminate() call,
 * WITHOUT tearing down the whole NimBLE host stack (which proved unsafe:
 * calling nimble_port_stop()/nimble_port_deinit() from outside the host
 * task, combined with esp_hidd_dev_deinit(), reliably crashed with
 * LoadProhibited — see field logs, PC 0x400828ca / EXCVADDR 0x1a,
 * happening on every stop() call, not just racing ones).
 *
 * RU: Хранит handle соединения нашей собственной (рекламирующей) GAP-сессии
 * — обновляется в nimble_hid_gap_event() ниже при коннекте/дисконнекте.
 * Позволяет извне (компоненту ble_keyboard) корректно разорвать текущее
 * соединение через ble_gap_terminate(), НЕ разрушая весь NimBLE-хост
 * целиком (это оказалось небезопасно: вызов nimble_port_stop()/
 * nimble_port_deinit() из чужой задачи в связке с esp_hidd_dev_deinit()
 * стабильно приводил к крэшу LoadProhibited — см. полевые логи,
 * PC 0x400828ca / EXCVADDR 0x1a, при КАЖДОМ вызове stop(), а не только
 * при гонке). */
static volatile uint16_t s_hid_conn_handle = 0xffff; /* BLE_HS_CONN_HANDLE_NONE */

uint16_t esp_hid_ble_gap_conn_handle(void)
{
    return s_hid_conn_handle;
}

esp_err_t esp_hid_ble_gap_adv_stop(void)
{
    int rc = ble_gap_adv_stop();
    if (rc != 0 && rc != BLE_HS_EALREADY) {
        ESP_LOGE(TAG, "error stopping advertisement; rc=%d", rc);
        return ESP_FAIL;
    }
    return ESP_OK;
}
