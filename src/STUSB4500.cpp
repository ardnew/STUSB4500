/*******************************************************************************
 *
 *  name: STUSB4500.cpp
 *  date: Dec 31, 2019
 *  auth: ardnew <andrew@ardnew.com>
 *  desc: Main library implementation source code
 *
 ******************************************************************************/

// ----------------------------------------------------------------- includes --

#include "STUSB4500.h"

// ---------------------------------------------------------- private defines --

//#define I2C_READ_TIMEOUT_MS    2000
//#define I2C_WRITE_TIMEOUT_MS   2000
#define I2C_CLOCK_FREQ_HZ    400000

#define EVAL_DEVICE_ID         0x21 // in device ID reg (0x2F)
#define PROD_DEVICE_ID         0x25 //

#define TLOAD_REG_INIT_MS       250 // section 6.1.3 in datasheet
#define SW_RESET_DEBOUNCE_MS     27
#define ATTACH_DEBOUNCE_MS       25

#define USB_DEFAULT_VOLTAGE_MV 5000
#define USB_DEFAULT_CURRENT_MA 1500

#define USBPD_MESSAGE_TYPE_DATA 'D'
#define USBPD_MESSAGE_TYPE_CTRL 'C'

// ----------------------------------------------------------- private macros --

// convert value at addr to little-endian (16-bit)
#define LE_u16(addr)                                      \
    ( ( (((uint16_t)(*(((uint8_t *)(addr)) + 1)))     ) + \
        (((uint16_t)(*(((uint8_t *)(addr)) + 0))) << 8) ) )

// convert value at addr to little-endian (32-bit)
#define LE_u32(addr)                                       \
    ( ( (((uint32_t)(*(((uint8_t *)(addr)) + 3)))      ) + \
        (((uint32_t)(*(((uint8_t *)(addr)) + 2))) <<  8) + \
        (((uint32_t)(*(((uint8_t *)(addr)) + 1))) << 16) + \
        (((uint32_t)(*(((uint8_t *)(addr)) + 0))) << 24) ) )

#define NO_INTERRUPT(e) { noInterrupts(); (e); interrupts(); }
#define DELAY(ms) delay(ms)

#define CABLE_CONNECTED(c) \
    ((CableStatus::CC1Connected == (c)) || (CableStatus::CC2Connected == (c)))

// ------------------------------------------------------------ private types --

typedef union
{
  uint16_t d16;
  struct {
#if defined(USBPD_REV_3_0_SUPPORT)
    uint16_t messageType     : 5; // USBPD rev >= 3.0 message type
#else
    uint16_t messageType     : 4; // USBPD rev  < 3.0 message type
    uint16_t reserved_4      : 1; // reserved
#endif
    uint16_t portDataRole    : 1; // port data role
    uint16_t specRevision    : 2; // spec revision
    uint16_t portPowerRole   : 1; // port power role/cable plug
    uint16_t messageID       : 3; // message ID
    uint16_t dataObjectCount : 3; // number of data objects
    uint16_t extended        : 1; // reserved
  } b;
}
USBPDMessageHeader;

// ------------------------------------------------------- exported variables --

static STUSB4500 *_usbpd; // used for reference in ISRs

// -------------------------------------------------------- private variables --

/* nothing */

// -------------------------------------------------------- private functions --

static void _alertISR(void) { _usbpd->alertISR(); }
static void _attachISR(void) { _usbpd->attachISR(); }

// ------------------------------------------------------- exported functions --

STUSB4500::STUSB4500(
    uint16_t const resetPin,
    uint8_t const slaveAddress,
    TwoWire const *wire
):
  _resetPin(resetPin),
  _slaveAddress(slaveAddress),
  _wire(wire),
  _srcCapRequestMax(DEFAULT_SRC_CAP_REQ_MAX),
  _status(),
  _state()
{
  _snkRDO = PDO();
  for (size_t i = 0U; i < NVM_SNK_PDO_MAX; ++i)
    { _snkPDO[i] = PDO(); }
  for (size_t i = 0U; i < NVM_SRC_PDO_MAX; ++i)
    { _srcPDO[i] = PDO(); }

  _cableAttached = nullptr;
  _cableDetached = nullptr;
  _sourceCapabilitiesReceived = nullptr;

  _usbpd = nullptr; // NULL until begin() is called with ALRT/ATCH pins
}

STUSB4500::STUSB4500(
    uint16_t const resetPin
):
  STUSB4500(
      resetPin, STUSB4500_I2C_SLAVE_BASE_ADDR, &Wire)
{
  /* empty */
}

bool STUSB4500::begin(uint16_t const alertPin, uint16_t const attachPin)
{
  if (nullptr == _usbpd) {
    _usbpd = this;
    attachInterrupt(digitalPinToInterrupt(alertPin), _alertISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(attachPin), _attachISR, CHANGE);

    // initialize I2C interface
    TwoWire *wire = (TwoWire *)_wire;
    wire->begin();
    wire->setClock(I2C_CLOCK_FREQ_HZ);
  }

  _started = false;
  if (ready())
    { _started = initialize(); }

  return _started;
}

/****
 * initializes the STUSB4500 device by waiting until it is responding to I2C
 * commands, clearing any current alerts, and unmasking useful interrupts.
 ****/
bool STUSB4500::initialize(void)
{
  if (!setPDOSnkCount(1U)) { return false; }
  if (!updatePDOSnk()) { return false; }
  if (!updateRDOSnk()) { return false; }
  if (!clearAlerts(true)) { return false; }
  if (!updatePrtStatus()) { return false; }
  if (!clearPDOSrc()) { return false; }

  CableStatus cable = cableStatus();
  if (CABLE_CONNECTED(cable)) {
    if (!updatePDOSrc())
      { return false; }
  }

  return true;
}

/****
 * query the internal device ID register of the STUSB4500 and verify it matches
 * the expected manufacturer-specified ID. this is used to determine if the
 * device has powered on and can respond to I2C read requests.
 ****/
bool STUSB4500::ready(void) const
{
  uint8_t deviceID;

  if (!wireRead(REG_DEVICE_ID, &deviceID, 1U)) { return false; }

  return (EVAL_DEVICE_ID == deviceID) || (PROD_DEVICE_ID == deviceID);
}

/****
 * repeatedly query the device in blocking mode until it is ready.
 ****/
void STUSB4500::waitUntilReady(void) const
{
  while (!ready()) { continue ; }
}

/****
 * enable or disable power output by toggling the hardware reset pin. this
 * routine can optionally block return until the device has verifiably powered
 * back on and is responding to I2C commands.
 ****/
void STUSB4500::enablePower(bool const enable, bool const wait)
{
  // uses hardware reset pin to toggle power output
  if (enable) {
    digitalWrite(_resetPin, LOW);

    // wait for I2C registers to be initialized from NVM
    DELAY(TLOAD_REG_INIT_MS);

    if (wait)
      { waitUntilReady(); }
  }
  else {
    digitalWrite(_resetPin, HIGH);
    DELAY(25);
  }
}

/****
 * assert and de-assert the hardware reset pin on the STUSB4500. after reset,
 * the device powers on and behaves according to NVM settings by default.
 * this routine can optionally block return until the device has verifiably
 * powered back on and is responding to I2C commands.
 ****/
void STUSB4500::reset(bool const wait)
{
  // the reset pin on STUSB4500 is active high, so driving high temporarily
  // will reset the device
  digitalWrite(_resetPin, HIGH);
  DELAY(25);
  digitalWrite(_resetPin, LOW);

  // wait for I2C registers to be initialized from NVM
  DELAY(TLOAD_REG_INIT_MS);

  if (wait)
    { waitUntilReady(); }
}

/****
 * resets the STUSB4500 by temporarily setting and then clearing the reset
 * register. this resets the internal Type-C and USB PD state machines and
 * causes electrical disconnect on both source and sink sides. while reset, all
 * pending interrupt alarms are cleared. this routine can optionally block
 * return until the device has verifiably powered back on and is responding to
 * I2C commands.
 ****/
void STUSB4500::softReset(bool const wait)
{
  uint8_t buff = SW_RST;
  if (!wireWrite(STUSB_GEN1S_RESET_CTRL_REG, &buff, 1U))
    { return; }

  if (!clearAlerts(false))
    { return; }

  DELAY(SW_RESET_DEBOUNCE_MS);

  // restore reset control register
  buff = No_SW_RST;
  if (!wireWrite(STUSB_GEN1S_RESET_CTRL_REG, &buff, 1U))
    { return; }

  // wait for I2C registers to be initialized from NVM
  DELAY(TLOAD_REG_INIT_MS);

  if (wait)
    { waitUntilReady(); }
}

void STUSB4500::update(void)
{
  static CableStatus cablePrev = CableStatus::NONE;

  uint8_t irq;
  uint8_t msg;
  uint8_t value;

  while (0U != _state.alertReceived) {
    NO_INTERRUPT(--(_state.alertReceived));
    processAlerts();
  }

  while (0U != _state.irqReceived) {
    --(_state.irqReceived);
    popPDInterrupt(irq);
  }

  // process attach events first so that we don't do any unnecessary
  // processing until a cable is actually attached
  while (0U != _state.attachReceived) {
    NO_INTERRUPT(--(_state.attachReceived));
    processAttach();

    CableStatus cable = cableStatus();
    // ignore duplicate interrupts, only handle if the connection state
    // actually changed.
    if (cablePrev != cable) {
      if (CABLE_CONNECTED(cable)) {
        DELAY(ATTACH_DEBOUNCE_MS);
        if (NULL != _cableAttached)
          { _cableAttached(); }
      }
      else {
        // verify the new state isn't an error state -- that we have positively
        // determined the cable is -not- connected
        if (CableStatus::NotConnected == cable) {
          if (NULL != _cableDetached)
            { _cableDetached(); }
        }
        (void)clearPDOSrc();
      }
    }
    cablePrev = cable;
  }

  if (0U != _state.msgReceived) {
    --(_state.msgReceived);

    while (popPDMessage(msg)) {
      switch (msg) {
        case USBPD_MESSAGE_TYPE_CTRL:
          if (popPDMessage(value)) { /* TBD */ }
          break;
        case USBPD_MESSAGE_TYPE_DATA:
          if (popPDMessage(value)) { /* TBD */ }
          break;
        default:
          break;
      }
    }
  }

  if (0U != _state.srcPDOReceived) {
    _state.srcPDOReceived = 0U;
    if (NULL != _sourceCapabilitiesReceived)
      { _sourceCapabilitiesReceived(); }
  }

  if (0U != _state.psrdyReceived) {
    --(_state.psrdyReceived);
  }
}

void STUSB4500::alertISR(void)
{
  ++(_state.alertReceived);
}

void STUSB4500::attachISR(void)
{
  ++(_state.attachReceived);
}

/****
 * read the port and Type-C status registers to determine if a USB cable is
 * connected to the STUSB4500 device. if connected, the orientation is
 * determined and indicated by return value of the selected CC line.
 ****/
CableStatus STUSB4500::cableStatus(void) const
{
  uint8_t portStatus;
  uint8_t typeCStatus;

  if (!wireRead(REG_PORT_STATUS, &portStatus, 1U))
    { return CableStatus::NONE; }

  if (VALUE_ATTACHED == (portStatus & STUSBMASK_ATTACHED_STATUS))
  {
    if (!wireRead(REG_TYPE_C_STATUS, &typeCStatus, 1U))
      { return CableStatus::NONE; }

    if (0U == (typeCStatus & MASK_REVERSE))
      { return CableStatus::CC1Connected; }
    else
      { return CableStatus::CC2Connected; }
  }
  else
  {
    return CableStatus::NotConnected;
  }
}

bool STUSB4500::requestSourceCapabilities(void)
{
  CableStatus cable = cableStatus();
  if (!CABLE_CONNECTED(cable)) { return false; }

  waitUntilReady();

  if (!clearAlerts(true)) { return false; }
  if (!updatePrtStatus()) { return false; }
  if (!clearPDOSrc()) { return false; }
  if (!updatePDOSrc()) { return false; }

  return true;
}

bool STUSB4500::updateSinkCapabilities(void)
{
  CableStatus cable = cableStatus();
  if (!CABLE_CONNECTED(cable)) { return false; }

  waitUntilReady();

  if (!updatePDOSnk()) { return false; }

  return true;
}

PDO STUSB4500::sourcePDO(size_t const n) const
{
  if (n < _status.pdoSrcCount)
    { return _srcPDO[n]; }

  return PDO(); // empty PDO
}

PDO STUSB4500::sinkPDO(size_t const n) const
{
  if (n < _status.pdoSnkCount)
    { return _snkPDO[n]; }

  return PDO(); // empty PDO
}

PDO STUSB4500::requestedPDO(void)
{
  PDO pdo = PDO();

  if (updateRDOSnk())
    { pdo = _snkRDO; }

  return pdo;
}

bool STUSB4500::setPower(uint32_t const voltage_mV, uint32_t const current_mA)
{
  CableStatus cable = cableStatus();
  if (!CABLE_CONNECTED(cable)) { return false; }

  if (!setPDOSnkCount(2U)) { return false; }
  if (!setPDOSnk(PDO(2U, voltage_mV, current_mA))) { return false; }
  if (!sendPDCableReset()) { return false; }
  if (!updatePDOSnk()) { return false; }

  return true;
}

bool STUSB4500::setPowerDefaultUSB(void)
{
  CableStatus cable = cableStatus();
  if (!CABLE_CONNECTED(cable)) { return false; }

  if (!setPDOSnkCount(1U)) { return false; }
  if (!sendPDCableReset()) { return false; }
  if (!updatePDOSnk()) { return false; }

  return true;
}

// -------------------------------------------------------- private functions --

/****
 * perform an I2C read on a STUSB4500 slave device.
 ****/
bool STUSB4500::wireRead(
    uint16_t const addr, uint8_t *buff, uint16_t const size) const
{
  uint8_t reg_addr = (uint8_t)(addr & 0xFF);
  uint8_t buff_sz = (uint8_t)(size & 0xFF);
  size_t count;

  TwoWire *wire = (TwoWire *)_wire;

  wire->beginTransmission(_slaveAddress);
  wire->write(reg_addr);  // set register for read
  wire->endTransmission(false); // false to not release the line
  wire->beginTransmission(_slaveAddress);
  wire->requestFrom(_slaveAddress, buff_sz);
  count = wire->readBytes(buff, buff_sz);
  if (count != size) { return false; }

  return true;
}

/****
 * perform an I2C write on a STUSB4500 slave device.
 ****/
bool STUSB4500::wireWrite(
    uint16_t const addr, uint8_t *buff, uint16_t const size) const
{
  uint8_t reg_addr = (uint8_t)(addr & 0xFF);
  uint8_t buff_sz  = (uint8_t)(size & 0xFF);
  uint8_t result;

  TwoWire *wire = (TwoWire *)_wire;

  wire->beginTransmission(_slaveAddress);
  wire->write(reg_addr); // command byte, sets register pointer address
  wire->write(buff, buff_sz);
  result = wire->endTransmission();
  if (result > 0U) { return false; }

  return true;
}

/****
 * clear all pending alerts by reading the status registers.
 ****/
bool STUSB4500::clearAlerts(bool const unmask)
{
  // clear alert status
  uint8_t alertStatus[12];
  if (!wireRead(ALERT_STATUS_1, alertStatus, 12U))
    { return false; }

  STUSB_GEN1S_ALERT_STATUS_MASK_RegTypeDef alertMask;
  if (unmask) {

    // set interrupts to unmask
    alertMask.d8 = 0xFF;

    //alertMask.b.PHY_STATUS_AL_MASK          = 0U;
    alertMask.b.PRT_STATUS_AL_MASK          = 0U;
    alertMask.b.PD_TYPEC_STATUS_AL_MASK     = 0U;
    //alertMask.b.HW_FAULT_STATUS_AL_MASK     = 0U;
    alertMask.b.MONITORING_STATUS_AL_MASK   = 0U;
    alertMask.b.CC_DETECTION_STATUS_AL_MASK = 0U;
    alertMask.b.HARD_RESET_AL_MASK          = 0U;

    // unmask the above alarms
    if (!wireWrite(ALERT_STATUS_MASK, &(alertMask.d8), 1U))
      { return false; }
  }

  return true;
}

bool STUSB4500::updatePrtStatus(void)
{
  uint8_t prtStatus[10];
  if (!wireRead(REG_PORT_STATUS, prtStatus, 10U))
    { return false; }

  _status.ccDetectionStatus.d8 = prtStatus[1];
  _status.ccStatus.d8          = prtStatus[3];
  _status.monitoringStatus.d8  = prtStatus[3];
  _status.hwFaultStatus.d8     = prtStatus[6];

  return true;
}

bool STUSB4500::updatePDOSnk(void)
{
#define BUFF_SZ NVM_SNK_PDO_MAX * sizeof(USB_PD_SNK_PDO_TypeDef)
  uint8_t pdo[BUFF_SZ];
  uint8_t pdoCount;

  waitUntilReady();

  if (!wireRead(DPM_PDO_NUMB, &pdoCount, 1U))
    { return false; }

  if (!wireRead(DPM_SNK_PDO1, pdo, BUFF_SZ))
    { return false; }

  _status.pdoSnkCount = pdoCount;
  for (uint8_t i = 0, j = 0; i < NVM_SNK_PDO_MAX; ++i, j += 4) {
    if (i < pdoCount) {
      _status.pdoSnk[i].d32 = LE_u32(&pdo[j]);
      _snkPDO[i] = PDO(i + 1,
          _status.pdoSnk[i].fix.Voltage * 50U,
          _status.pdoSnk[i].fix.Operational_Current * 10U);
    }
    else {
      _status.pdoSnk[i].d32 = 0U;
      _snkPDO[i] = PDO();
    }
  }
  return true;
#undef BUFF_SZ
}

bool STUSB4500::setPDOSnkCount(uint8_t const count)
{
  if ((count < 1) || (count > NVM_SNK_PDO_MAX))
    { return false; }

  uint8_t pdoCount = (uint8_t)count;

  return wireWrite(DPM_PDO_NUMB, &pdoCount, 1);
}

bool STUSB4500::setPDOSnk(PDO const pdo)
{
  if ((pdo.number < 1) || (pdo.number > NVM_SNK_PDO_MAX))
    { return false; }

  uint16_t voltage_mV = pdo.voltage_mV;
//  if (1U == pdo.number) // PDO 1 must always be USB +5V
//    { voltage_mV = 5000U; }

  // use the received PDO definition #1 (USB default) as template for new PDO,
  // just update the voltage and current to the input PDO.
  USB_PD_SNK_PDO_TypeDef def = _status.pdoSnk[0];
  uint8_t address = DPM_SNK_PDO1 + (4U * (pdo.number - 1U));
  def.fix.Voltage = voltage_mV / 50U;
  def.fix.Operational_Current = pdo.current_mA / 10U;

  return wireWrite(address, (uint8_t *)&(def.d32), 4U);
}

// static bool stusb4500_set_snk_var_pdo(void)
// {
//   return true;
// }

// static bool stusb4500_set_snk_bat_pdo(void)
// {
//   return true;
// }

bool STUSB4500::clearPDOSrc(void)
{
  _status.pdoSrcCount = 0U;
  for (uint8_t i = 0; i < NVM_SRC_PDO_MAX; ++i) {
    _status.pdoSrc[i].d32 = 0U;
    _srcPDO[i] = PDO();
  }
  return true;
}

bool STUSB4500::updatePDOSrc(void)
{
  waitUntilReady();

  //DELAY(TLOAD_REG_INIT_MS);

  static uint8_t const maxRequests = _srcCapRequestMax;
  uint8_t request = 0U;
  bool status = true;

  ++(_state.srcPDORequesting);

  while ((0U != _state.srcPDORequesting) && (request < maxRequests)) {
    if (!sendPDCableReset())
      { break; }
    processAlerts();
    ++request;
  }

  return status;
}

bool STUSB4500::updateRDOSnk(void)
{
  STUSB_GEN1S_RDO_REG_STATUS_RegTypeDef rdo;
  if (!wireRead(RDO_REG_STATUS, (uint8_t *)&rdo, sizeof(rdo)))
    { return false; }

  if (rdo.b.Object_Pos > 0) {

    _snkRDO.number        = rdo.b.Object_Pos;
    _snkRDO.current_mA    = rdo.b.OperatingCurrent * 10U;
    _snkRDO.maxCurrent_mA = rdo.b.MaxCurrent * 10U;

    if (_snkRDO.number <= _status.pdoSnkCount) {
      _snkRDO.voltage_mV =
          _status.pdoSnk[_snkRDO.number - 1U].fix.Voltage * 50U;
    }
    return true;
  }
  else {
    _snkRDO = PDO();
    return false;
  }
}

/****
 * send a USB power delivery reset message, forcing the source to send all of
 * its available PDOs.
 ****/
bool STUSB4500::sendPDCableReset(void)
{
#define USBPD_HEADER_SOFT_RESET 0x0D
#define USBPD_PD_COMMAND        0x26

  CableStatus cable = cableStatus();
  if (!CABLE_CONNECTED(cable)) { return false; }

  // send PD message "soft reset" to source by setting TX header (0x51) to 0x0D,
  // and set PD command (0x1A) to 0x26.
  uint8_t data = USBPD_HEADER_SOFT_RESET;
  if (!wireWrite(TX_HEADER, &data, 1U))
    { return false; }

  uint8_t command = USBPD_PD_COMMAND;
  if (!wireWrite(STUSB_GEN1S_CMD_CTRL, &command, 1U))
    { return false; }

  return true;

#undef USBPD_HEADER_SOFT_RESET
#undef USBPD_PD_COMMAND
}

void STUSB4500::processAlerts(void)
{
#define READ_BUFF_MAX_SZ 40
  STUSB_GEN1S_ALERT_STATUS_RegTypeDef      alertStatus;
  STUSB_GEN1S_ALERT_STATUS_MASK_RegTypeDef alertMask;
  uint8_t buff[READ_BUFF_MAX_SZ];

  if (!wireRead(ALERT_STATUS_1, buff, 2U))
    { return; }

  alertMask.d8   = buff[1];
  alertStatus.d8 = buff[0] & ~(alertMask.d8);

  pushPDInterrupt(alertStatus.d8);

  if (0U != alertStatus.d8) {

    // bit 2
    if (0U != alertStatus.b.PRT_STATUS_AL) {

      USBPDMessageHeader header;

      if (!wireRead(PRT_STATUS, buff, 1U))
        { return; }
      _status.prtStatus.d8 = buff[0];

      if (1U == _status.prtStatus.b.MSG_RECEIVED) {

        if (!wireRead(RX_HEADER, buff, 2U))
          { return; }

        header.d16 = LE_u16(buff);

        if (header.b.dataObjectCount > 0) {

          pushPDMessage(USBPD_MESSAGE_TYPE_DATA, header.b.messageType);

          if (!wireRead(RX_BYTE_CNT, buff, 1U))
            { return; }
          uint8_t rxByteCount = buff[0];
          if (rxByteCount != header.b.dataObjectCount * 4U)
            { return; }

          switch (header.b.messageType) {
            case USBPD_DATAMSG_Source_Capabilities:

              if (!wireRead(RX_DATA_OBJ, buff, rxByteCount))
                { return; }

              _status.pdoSrcCount = header.b.dataObjectCount;
              for (uint8_t i = 0U, j = 0U;
                    i < header.b.dataObjectCount;
                    ++i, j += 4) {
                _status.pdoSrc[i].d32 = LE_u32(&buff[j]);
                if (0U == i)
                  { _status.pdoSrc[i].fix.Voltage = 100U; }
                _srcPDO[i] = PDO(i + 1,
                    _status.pdoSrc[i].fix.Voltage * 50U,
                    _status.pdoSrc[i].fix.Max_Operating_Current * 10U);
              }

              _state.srcPDORequesting = 0U;
              ++(_state.srcPDOReceived);
              break;

            case USBPD_DATAMSG_Request:
            case USBPD_DATAMSG_Sink_Capabilities:
            case USBPD_DATAMSG_Vendor_Defined:
            default :
              break;
          }
        }
        else {

          pushPDMessage(USBPD_MESSAGE_TYPE_CTRL, header.b.messageType);

          switch (header.b.messageType) {
            case USBPD_CTRLMSG_GoodCRC:
                 ++(_state.msgGoodCRC);
                break;

            case USBPD_CTRLMSG_Accept:
                 ++(_state.msgAccept);
                break;

            case USBPD_CTRLMSG_Reject:
                 ++(_state.msgReject);
                break;

            case USBPD_CTRLMSG_PS_RDY:
                 ++(_state.psrdyReceived);
                break;

            case USBPD_CTRLMSG_Reserved1:
            case USBPD_CTRLMSG_Get_Source_Cap:
            case USBPD_CTRLMSG_Get_Sink_Cap:
            case USBPD_CTRLMSG_Wait:
            case USBPD_CTRLMSG_Soft_Reset:
            case USBPD_CTRLMSG_Not_Supported:
            case USBPD_CTRLMSG_Get_Source_Cap_Extended:
            case USBPD_CTRLMSG_Get_Status:
            case USBPD_CTRLMSG_FR_Swap:
            case USBPD_CTRLMSG_Get_PPS_Status:
            case USBPD_CTRLMSG_Get_Country_Codes:
            default:
              break;
          }
        }
      }
    }

    // bit 8
    _status.hwReset = buff[0] >> 7U;
    if (0U != _status.hwReset)
      { ++(_state.irqHardReset); }

    // bit 7
    if (0U != alertStatus.b.CC_DETECTION_STATUS_AL) {
      if (!wireRead(PORT_STATUS_TRANS, buff, 2U))
        { return; }
      _status.ccDetectionStatus.d8 = buff[1];
      if (0U != (buff[0] & STUSBMASK_ATTACH_STATUS_TRANS))
        { ++(_state.attachTransition); }
    }

    // bit 6
    if (0U != alertStatus.b.MONITORING_STATUS_AL) {
      if (!wireRead(TYPEC_MONITORING_STATUS_0, buff, 2U))
        { return; }
      _status.monitoringStatus.d8 = buff[1];
    }

    // always read & update CC attachement status
    if (!wireRead(CC_STATUS, buff, 1U))
      { return; }
    _status.ccStatus.d8 = buff[0];

    // bit 5
    if (0U != alertStatus.b.HW_FAULT_STATUS_AL) {
      if (!wireRead(CC_HW_FAULT_STATUS_0, buff, 2U))
        { return; }
      _status.hwFaultStatus.d8 = buff[1];
    }

  }

  return;
#undef READ_BUFF_MAX_SZ
}

void STUSB4500::processAttach(void)
{
  return;
}

bool STUSB4500::pushPDMessage(uint8_t const type, uint8_t const value)
{
  _state.msg[_state.msgHead] = type;
  ++(_state.msgHead);

  _state.msg[_state.msgHead] = value;
  ++(_state.msgHead);

  if (_state.msgHead == _state.msgTail)
    { return false; } // buffer overflow

  if (_state.msgHead >= USBPD_MESSAGE_QUEUE_SZ)
    { _state.msgHead = 0U; }

  ++(_state.msgReceived);

  return true;
}

bool STUSB4500::popPDMessage(uint8_t &type)
{
  if (_state.msgHead == _state.msgTail)
    { return false; } // empty buffer

  type = _state.msg[_state.msgTail];
  _state.msg[_state.msgTail] = 0U;

  ++(_state.msgTail);
  if (_state.msgTail >= USBPD_MESSAGE_QUEUE_SZ)
    { _state.msgTail = 0U; }

  return true;
}

bool STUSB4500::pushPDInterrupt(uint8_t const type)
{
  _state.irq[_state.irqHead] = type;
  ++(_state.irqHead);

  if (_state.irqHead == _state.irqTail)
    { return false; } // buffer overflow

  if (_state.irqHead >= USBPD_INTERRUPT_QUEUE_SZ)
    { _state.irqHead = 0U; }

  ++(_state.irqReceived);

  return true;
}

bool STUSB4500::popPDInterrupt(uint8_t &type)
{
  if (_state.irqHead == _state.irqTail)
    { return false; } // empty buffer

  type = _state.irq[_state.irqTail];
  _state.irq[_state.irqTail] = 0U;

  ++(_state.irqTail);
  if (_state.irqTail >= USBPD_INTERRUPT_QUEUE_SZ)
    { _state.irqTail = 0U; }

  return true;
}
