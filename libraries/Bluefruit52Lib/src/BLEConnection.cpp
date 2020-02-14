/**************************************************************************/
/*!
    @file     BLEConnection.cpp
    @author   hathach (tinyusb.org)

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2019, Adafruit Industries (adafruit.com)
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
    1. Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
    3. Neither the name of the copyright holders nor the
    names of its contributors may be used to endorse or promote products
    derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/**************************************************************************/

#include "bluefruit.h"

//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM DECLARATION
//--------------------------------------------------------------------+

BLEConnection::BLEConnection(uint16_t conn_hdl, ble_gap_evt_connected_t const* evt_connected, uint8_t hvn_qsize, uint8_t wrcmd_qsize)
{
  _conn_hdl = conn_hdl;
  _connected = true;

  _mtu = BLE_GATT_ATT_MTU_DEFAULT;
  _data_length = BLE_GATT_ATT_MTU_DEFAULT + 4; // 27
  _phy = BLE_GAP_PHY_1MBPS;
  _conn_interval = 0;
  _peer_addr = evt_connected->peer_addr;
  _role = evt_connected->role;

  _hvn_sem   = xSemaphoreCreateCounting(hvn_qsize, hvn_qsize);
  _wrcmd_sem = xSemaphoreCreateCounting(wrcmd_qsize, wrcmd_qsize);

  _paired = false;
  _hvc_sem = NULL;
  _hvc_received = false;
  _pair_sem = NULL;

  _ediv = 0xFFFF;
  _bond_keys = NULL;
  _peer_pubkey = NULL;
}

BLEConnection::~BLEConnection()
{
  vSemaphoreDelete( _hvn_sem );
  vSemaphoreDelete( _wrcmd_sem );

  //------------- on-the-fly data must be freed -------------//
  if (_hvc_sem  ) vSemaphoreDelete(_hvc_sem );
  if (_pair_sem ) vSemaphoreDelete(_pair_sem);

  if (_bond_keys   ) rtos_free(_bond_keys);
  if (_peer_pubkey ) rtos_free(_peer_pubkey);
}

uint16_t BLEConnection::handle (void)
{
  return _conn_hdl;
}

bool BLEConnection::connected(void)
{
  return _connected;
}

bool BLEConnection::paired (void)
{
  return _paired;
}

uint8_t BLEConnection::getRole (void)
{
  return _role;
}

uint16_t BLEConnection::getMtu (void)
{
  return _mtu;
}

uint16_t BLEConnection::getConnectionInterval(void)
{
  return _conn_interval;
}

uint16_t BLEConnection::getDataLength(void)
{
  return _data_length;
}

uint8_t BLEConnection::getPHY(void)
{
  return _phy;
}

ble_gap_addr_t BLEConnection::getPeerAddr (void)
{
  return _peer_addr;
}

uint16_t BLEConnection::getPeerName(char* buf, uint16_t bufsize)
{
  return Bluefruit.Gatt.readCharByUuid(_conn_hdl, BLEUuid(BLE_UUID_GAP_CHARACTERISTIC_DEVICE_NAME), buf, bufsize);
}

static inline bool is_tx_power_valid(int8_t power)
{
#if defined(NRF52832_XXAA)
  int8_t const accepted[] = { -40, -20, -16, -12, -8, -4, 0, 3, 4 };
#elif defined( NRF52840_XXAA)
  int8_t const accepted[] = { -40, -20, -16, -12, -8, -4, 0, 2, 3, 4, 5, 6, 7, 8 };
#endif

  for (uint32_t i=0; i<sizeof(accepted); i++)
  {
    if (accepted[i] == power) return true;
  }

  return false;
}

bool BLEConnection::setTxPower(int8_t power)
{
  VERIFY(is_tx_power_valid(power));
  VERIFY_STATUS(sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_CONN, _conn_hdl, power), false);
  return true;
}

bool BLEConnection::requestMtuExchange(uint16_t mtu)
{
  VERIFY_STATUS(sd_ble_gattc_exchange_mtu_request(_conn_hdl, mtu), false);
  return true;
}

bool BLEConnection::requestDataLengthUpdate(ble_gap_data_length_params_t const *p_dl_params, ble_gap_data_length_limitation_t *p_dl_limitation)
{
  VERIFY_STATUS(sd_ble_gap_data_length_update(_conn_hdl, p_dl_params, p_dl_limitation), false);
  return true;
}

bool BLEConnection::requestConnectionParameter(uint16_t conn_interval, uint16_t slave_latency, uint16_t sup_timeout)
{
  ble_gap_conn_params_t const conn_params =
  {
    .min_conn_interval = conn_interval,
    .max_conn_interval = conn_interval,
    .slave_latency = slave_latency,
    .conn_sup_timeout = sup_timeout
  };
  VERIFY_STATUS(sd_ble_gap_conn_param_update(_conn_hdl, &conn_params), false);

  return true;
}

bool BLEConnection::requestPHY(uint8_t phy)
{
  ble_gap_phys_t gap_phy = { .tx_phys = phy, .rx_phys = phy };
  VERIFY_STATUS( sd_ble_gap_phy_update(_conn_hdl, &gap_phy), false);
  return true;
}

bool BLEConnection::disconnect(void)
{
  return ERROR_NONE == sd_ble_gap_disconnect(_conn_hdl, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
}

bool BLEConnection::monitorRssi(uint8_t threshold)
{
  // number of samples is 3
  VERIFY_STATUS(sd_ble_gap_rssi_start(_conn_hdl, threshold, 3), false);
  return true;
}

int8_t BLEConnection::getRssi(void)
{
  int8_t rssi;
  uint8_t channel_idx;

  VERIFY_STATUS(sd_ble_gap_rssi_get(_conn_hdl, &rssi, &channel_idx), 0);
  (void) channel_idx;

  return rssi;
}

void BLEConnection::stopRssi(void)
{
  sd_ble_gap_rssi_stop(_conn_hdl);
}

bool BLEConnection::getHvnPacket (void)
{
  return xSemaphoreTake(_hvn_sem, ms2tick(BLE_GENERIC_TIMEOUT));
}

bool BLEConnection::getWriteCmdPacket (void)
{
  return xSemaphoreTake(_wrcmd_sem, ms2tick(BLE_GENERIC_TIMEOUT));
}

bool BLEConnection::storeCccd(void)
{
  return bond_save_cccd( _role, _conn_hdl, _ediv);
}

bool BLEConnection::loadKeys(bond_keys_t* bkeys)
{
  return bond_load_keys(_role, _ediv, bkeys);
}

bool BLEConnection::requestPairing(void)
{
  // skip if already paired
  if ( _paired ) return true;

  ble_gap_sec_params_t sec_param = Bluefruit.Pairing.getSecureParam();

  // on-the-fly semaphore
  _pair_sem = xSemaphoreCreateBinary();

  if ( _role == BLE_GAP_ROLE_PERIPH )
  {
    VERIFY_STATUS( sd_ble_gap_authenticate(_conn_hdl, &sec_param ), false);
    xSemaphoreTake(_pair_sem, portMAX_DELAY);
  }
  else
  {
    uint16_t cntr_ediv = 0xFFFF;
    bond_keys_t bkeys;

    // Check to see if we did bonded with current prph previously
    // TODO currently only matches key using fixed address
    if ( bond_find_cntr(&_peer_addr, &bkeys) )
    {
      cntr_ediv = bkeys.peer_enc.master_id.ediv;
      LOG_LV2("BOND", "Load Keys from file " BOND_FNAME_CNTR, cntr_ediv);
      VERIFY_STATUS( sd_ble_gap_encrypt(_conn_hdl, &bkeys.peer_enc.master_id, &bkeys.peer_enc.enc_info), false);

    }else
    {
      VERIFY_STATUS( sd_ble_gap_authenticate(_conn_hdl, &sec_param ), false);
    }

    xSemaphoreTake(_pair_sem, portMAX_DELAY);

    // Failed to pair using central stored keys, this happens when
    // Prph delete bonds while we did not --> let's remove the obsolete keyfile and retry
    if ( !_paired && (cntr_ediv != 0xffff) )
    {
      bond_remove_key(BLE_GAP_ROLE_CENTRAL, cntr_ediv);

      // Re-try with a fresh session
      VERIFY_STATUS( sd_ble_gap_authenticate(_conn_hdl, &sec_param ), false);

      xSemaphoreTake(_pair_sem, portMAX_DELAY);
    }
  }

  vSemaphoreDelete(_pair_sem);
  _pair_sem = NULL;

  return _paired;
}

bool BLEConnection::waitForIndicateConfirm(void)
{
  // on the fly semaphore
  _hvc_sem = xSemaphoreCreateBinary();

  _hvc_received = false;
  xSemaphoreTake(_hvc_sem, portMAX_DELAY);

  vSemaphoreDelete(_hvc_sem);
  _hvc_sem = NULL;

  return _hvc_received;
}

void BLEConnection::_eventHandler(ble_evt_t* evt)
{
  switch(evt->header.evt_id)
  {
    case BLE_GAP_EVT_DISCONNECTED:
      // mark as disconnected
      _connected = false;
    break;

    //--------------------------------------------------------------------+
    /* First-time Pairing
     *
     * Connect -> SEC_PARAMS_REQUEST -> PASSKEY_DISPLAY -> BLE_GAP_EVT_LESC_DHKEY_REQUEST ->
     *            CONN_SEC_UPDATE -> AUTH_STATUS
     * 1. Either we or peer initiate the process
     * 2. Peer ask for Secure Parameter ( I/O Caps ) BLE_GAP_EVT_SEC_PARAMS_REQUEST
     * 3. Pair PassKey exchange
     * 4. Connection is secured BLE_GAP_EVT_CONN_SEC_UPDATE
     * 5. Long term Keys exchanged BLE_GAP_EVT_AUTH_STATUS
     *
     * Reconnect using bonded key
     * Connect -> SEC_INFO_REQUEST -> CONN_SEC_UPDATE
     * 1. Either we or peer initiate the process
     * 2. Peer ask for Secure Info ( bond keys ) BLE_GAP_EVT_SEC_INFO_REQUEST
     * 3. Connection is secured BLE_GAP_EVT_CONN_SEC_UPDATE
     */
    //--------------------------------------------------------------------+

    case BLE_GAP_EVT_AUTH_STATUS:
    {
      ble_gap_evt_auth_status_t* status = &evt->evt.gap_evt.params.auth_status;

      // Pairing succeeded
      if (BLE_GAP_SEC_STATUS_SUCCESS == status->auth_status) _paired = true;
    }
    break;

    case BLE_GAP_EVT_CONN_SEC_UPDATE:
    {
      const ble_gap_conn_sec_t* conn_sec = &evt->evt.gap_evt.params.conn_sec_update.conn_sec;

      // Connection is secured (paired) if encryption level > 1
      if ( !( conn_sec->sec_mode.sm == 1 && conn_sec->sec_mode.lv == 1) )
      {
        _paired = true;
      }

      if (_pair_sem) xSemaphoreGive(_pair_sem);
    }
    break;

    //--------------------------------------------------------------------+
    // GAP parameter negotiation
    //--------------------------------------------------------------------+
    case BLE_GAP_EVT_CONN_PARAM_UPDATE:
    {
      ble_gap_conn_params_t* param = &evt->evt.gap_evt.params.conn_param_update.conn_params;
      _conn_interval = param->max_conn_interval;

      LOG_LV1("GAP", "Conn Interval= %.2f ms, Latency = %d, Supervisor Timeout = %d ms", _conn_interval*1.25f, param->slave_latency, 10*param->conn_sup_timeout);
    }
    break;

    //------------- MTU -------------//
    case BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST:
    {
      uint16_t const max_mtu = Bluefruit.getMaxMtu(_role);
      _mtu = minof(evt->evt.gatts_evt.params.exchange_mtu_request.client_rx_mtu, max_mtu);

      VERIFY_STATUS( sd_ble_gatts_exchange_mtu_reply(_conn_hdl, _mtu), );

      LOG_LV1("GAP", "ATT MTU is changed to %d", _mtu);
    }
    break;

    case BLE_GATTC_EVT_EXCHANGE_MTU_RSP:
      _mtu = evt->evt.gattc_evt.params.exchange_mtu_rsp.server_rx_mtu;
      LOG_LV1("GAP", "ATT MTU is changed to %d", _mtu);
    break;

    //------------- Data Length -------------//
    case BLE_GAP_EVT_DATA_LENGTH_UPDATE_REQUEST:
    {
      ble_gap_data_length_params_t* param = &evt->evt.gap_evt.params.data_length_update_request.peer_params;
      (void) param;

      LOG_LV1("GAP", "Data Length Request is (tx, rx) octets = (%d, %d), (tx, rx) time = (%d, %d) us",
              param->max_tx_octets, param->max_rx_octets, param->max_tx_time_us, param->max_rx_time_us);

      // Let Softdevice decide the data length
      VERIFY_STATUS( sd_ble_gap_data_length_update(_conn_hdl, NULL, NULL), );
    }
    break;

    case BLE_GAP_EVT_DATA_LENGTH_UPDATE:
    {
      ble_gap_data_length_params_t* datalen =  &evt->evt.gap_evt.params.data_length_update.effective_params;
      (void) datalen;

      LOG_LV1("GAP", "Data Length is (tx, rx) octets = (%d, %d), (tx, rx) time = (%d, %d) us",
                   datalen->max_tx_octets, datalen->max_rx_octets, datalen->max_tx_time_us, datalen->max_rx_time_us);

      _data_length = datalen->max_tx_octets;
    }
    break;

    //------------- PHY -------------//
    case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
    {
      ble_gap_phys_t* req_phy = &evt->evt.gap_evt.params.phy_update_request.peer_preferred_phys;
      (void) req_phy;

      LOG_LV1("GAP", "PHY request tx = 0x%02X, rx = 0x%02X", req_phy->tx_phys, req_phy->rx_phys);

      // Tell SoftDevice to choose PHY automatically
      ble_gap_phys_t phy = { BLE_GAP_PHY_AUTO, BLE_GAP_PHY_AUTO };
      (void) sd_ble_gap_phy_update(_conn_hdl, &phy);
    }
    break;

    case BLE_GAP_EVT_PHY_UPDATE:
    {
      ble_gap_evt_phy_update_t* active_phy = &evt->evt.gap_evt.params.phy_update;

      if ( active_phy->status != BLE_HCI_STATUS_CODE_SUCCESS )
      {
        LOG_LV1("GAP", "Failed HCI status = 0x%02X", active_phy->status);
      }else
      {
        char const *phy_str[] = { "Auto", "1 Mbps", "2 Mbps", "Coded" };
        (void) phy_str;
        LOG_LV1("GAP", "PHY active tx: %s, rx: %s", phy_str[active_phy->tx_phy], phy_str[active_phy->rx_phy]);

        _phy = active_phy->tx_phy;
      }
    }
    break;

    //--------------------------------------------------------------------+
    //
    //--------------------------------------------------------------------+
    case BLE_GATTS_EVT_HVN_TX_COMPLETE:
      for(uint8_t i=0; i<evt->evt.gatts_evt.params.hvn_tx_complete.count; i++) xSemaphoreGive(_hvn_sem);
    break;

    case BLE_GATTC_EVT_WRITE_CMD_TX_COMPLETE:
      for(uint8_t i=0; i<evt->evt.gattc_evt.params.write_cmd_tx_complete.count; i++) xSemaphoreGive(_wrcmd_sem);
    break;

    case BLE_GATTS_EVT_HVC:
    {
      LOG_LV2("GATTS", "Confirm received handle = 0x%04X", evt->evt.gatts_evt.params.hvc.handle);

      _hvc_received = true;
      if ( _hvc_sem ) xSemaphoreGive(_hvc_sem);
    }
    break;

    case BLE_GATTS_EVT_TIMEOUT:
    {
      uint8_t timeout_src = evt->evt.gatts_evt.params.timeout.src;
      LOG_LV2("GATTS", "Timeout Source = %d", timeout_src);

      if (BLE_GATT_TIMEOUT_SRC_PROTOCOL == timeout_src)
      {
        _hvc_received = false;
        if ( _hvc_sem ) xSemaphoreGive(_hvc_sem);
      }
    }
    break;

    case BLE_GATTS_EVT_SYS_ATTR_MISSING:
      sd_ble_gatts_sys_attr_set(_conn_hdl, NULL, 0, 0);
    break;

    default: break;
  }
}
