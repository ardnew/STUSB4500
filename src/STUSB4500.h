/*******************************************************************************
 *
 *  name: STUSB4500.h
 *  date: Dec 31, 2019
 *  auth: ardnew <andrew@ardnew.com>
 *  desc: Main library interface source code
 *
 ******************************************************************************/

#ifndef __STUSB4500_H__
#define __STUSB4500_H__

// ----------------------------------------------------------------- includes --

#include <Arduino.h>
#include <Wire.h>

#include "dat/STUSB4500_register.h"

// ------------------------------------------------------------------ defines --

#define VERSION_MAJ 1
#define VERSION_MIN 0
#define VERSION_REV 5

#define STUSB4500_I2C_SLAVE_BASE_ADDR 0x28

#define USBPD_REV_3_0_SUPPORT      1
#define USBPD_MESSAGE_QUEUE_SZ    32
#define USBPD_INTERRUPT_QUEUE_SZ  32
#define NVM_SNK_PDO_MAX            3
#define NVM_SRC_PDO_MAX           10
#define DEFAULT_SRC_CAP_REQ_MAX  200

// ------------------------------------------------------------------- macros --

/* nothing */

// ----------------------------------------------------------- exported types --

class STUSB4500Version {
protected:
  constexpr static uint8_t const _strSize = 12U;
  char _str[_strSize] = { '\0' };
  uint8_t const _major;
  uint8_t const _minor;
  uint8_t const _revision;
  uint32_t const _base256;

public:
  STUSB4500Version(
      uint8_t const major,
      uint8_t const minor,
      uint8_t const revision
  ):
    _major(major),
    _minor(minor),
    _revision(revision),
    _base256(((major << 16U) | (minor << 8U) | revision) & 0x00FFFFFF)
  {
    sprintf(_str, "%u.%u.%u", _major, _minor, _revision);
  }
  char const *str() const
    { return _str; }
  inline bool operator==(STUSB4500Version const &version)
    { return _base256 == version._base256; }
  inline bool operator!=(STUSB4500Version const &version)
    { return _base256 != version._base256; }
  inline bool operator<(STUSB4500Version const &version)
    { return _base256 < version._base256; }
};

enum class CableStatus {
  NONE = -1,
  NotConnected, // = 0
  CC1Connected, // = 1
  CC2Connected, // = 2
  COUNT         // = 3
};

enum class PDOSupplyType {
  NONE = -1,
  Fixed,    // = 0
  Variable, // = 1
  Battery,  // = 2
  COUNT     // = 3
};

class PDO {
public:
  size_t   number;
  uint16_t voltage_mV;
  uint16_t current_mA;
  uint16_t maxCurrent_mA;
  PDO(void):
    number(0U),
    voltage_mV(0U),
    current_mA(0U),
    maxCurrent_mA(0U)
  {}
  PDO(
      size_t const number,
      uint16_t const voltage_mV,
      uint16_t const current_mA,
      uint16_t const maxCurrent_mA = 0U
  ):
    number(number),
    voltage_mV(voltage_mV),
    current_mA(current_mA),
    maxCurrent_mA(maxCurrent_mA)
  {}

  inline bool operator==(PDO const &pdo)
  {
    return (    voltage_mV == pdo.voltage_mV    ) &&
           (    current_mA == pdo.current_mA    ) &&
           ( maxCurrent_mA == pdo.maxCurrent_mA ) ;
  }
  inline bool operator!=(PDO const &pdo)
  {
    return (    voltage_mV != pdo.voltage_mV    ) ||
           (    current_mA != pdo.current_mA    ) ||
           ( maxCurrent_mA != pdo.maxCurrent_mA ) ;
  }
};

typedef void (* USBEventCallback)(void);

class USBPDStatus {
public:
  uint8_t                                    hwReset;
  STUSB_GEN1S_HW_FAULT_STATUS_RegTypeDef     hwFaultStatus;     // 8-bit
  STUSB_GEN1S_MONITORING_STATUS_RegTypeDef   monitoringStatus;  // 8-bit
  STUSB_GEN1S_CC_DETECTION_STATUS_RegTypeDef ccDetectionStatus; // 8-bit
  STUSB_GEN1S_CC_STATUS_RegTypeDef           ccStatus;          // 8-bit
  STUSB_GEN1S_PRT_STATUS_RegTypeDef          prtStatus;         // 8-bit
  STUSB_GEN1S_PHY_STATUS_RegTypeDef          phyStatus;         // 8-bit
  STUSB_GEN1S_RDO_REG_STATUS_RegTypeDef      rdoSnk;
  size_t                                     pdoSnkCount;
  USB_PD_SNK_PDO_TypeDef                     pdoSnk[NVM_SNK_PDO_MAX];
  size_t                                     pdoSrcCount;
  USB_PD_SRC_PDO_TypeDef                     pdoSrc[NVM_SRC_PDO_MAX];

  USBPDStatus(void)
  {
    hwReset              = 0U;
    hwFaultStatus.d8     = 0U;
    monitoringStatus.d8  = 0U;
    ccDetectionStatus.d8 = 0U;
    ccStatus.d8          = 0U;
    prtStatus.d8         = 0U;
    phyStatus.d8         = 0U;
    rdoSnk.d32           = 0U;

    pdoSnkCount = 0U;
    for (size_t i = 0U; i < NVM_SNK_PDO_MAX; ++i)
      { pdoSnk[i].d32 = 0U; }

    pdoSrcCount = 0U;
    for (size_t i = 0U; i < NVM_SRC_PDO_MAX; ++i)
      { pdoSrc[i].d32 = 0U; }
  }
};

class USBPDStateMachine {
public:
  volatile uint8_t alertReceived;
  volatile uint8_t attachReceived;
  uint16_t irqReceived;
  uint16_t irqHardReset;
  uint16_t attachTransition;
  uint16_t srcPDOReceived;
  uint16_t srcPDORequesting;
  uint16_t psrdyReceived;
  uint16_t msgReceived;
  uint16_t msgAccept;
  uint16_t msgReject;
  uint16_t msgGoodCRC;
  uint8_t  msgHead;
  uint8_t  msgTail;
  uint8_t  msg[USBPD_MESSAGE_QUEUE_SZ];
  uint8_t  irqHead;
  uint8_t  irqTail;
  uint8_t  irq[USBPD_INTERRUPT_QUEUE_SZ];

  USBPDStateMachine(void)
  {
    alertReceived    = 0U;
    attachReceived   = 0U;
    irqReceived      = 0U;
    irqHardReset     = 0U;
    attachTransition = 0U;
    srcPDOReceived   = 0U;
    srcPDORequesting = 0U;
    psrdyReceived    = 0U;
    msgReceived      = 0U;
    msgAccept        = 0U;
    msgReject        = 0U;
    msgGoodCRC       = 0U;

    msgHead = 0U;
    msgTail = 0U;
    for (size_t i = 0U; i < USBPD_MESSAGE_QUEUE_SZ; ++i)
      { msg[i] = 0U; }

    irqHead = 0U;
    irqTail = 0U;
    for (size_t i = 0U; i < USBPD_INTERRUPT_QUEUE_SZ; ++i)
      { irq[i] = 0U; }
  }
};

class STUSB4500 {
protected:
  STUSB4500Version _VERSION =
      STUSB4500Version(VERSION_MAJ, VERSION_MIN, VERSION_REV);

  uint16_t       _resetPin;
  uint8_t        _slaveAddress; // real address, NOT shifted
  TwoWire const *_wire;

  uint16_t _srcCapRequestMax;

  USBPDStatus       _status;
  USBPDStateMachine _state;

  PDO _snkRDO;
  PDO _snkPDO[NVM_SNK_PDO_MAX];
  PDO _srcPDO[NVM_SRC_PDO_MAX];

  USBEventCallback _cableAttached;
  USBEventCallback _cableDetached;
  USBEventCallback _sourceCapabilitiesReceived;

private:
  // whether or not the device has been initialized
  bool _started;

  // I2C read/write operations
  bool wireRead(
      uint16_t const addr, uint8_t *buff, uint16_t const size) const;
  bool wireWrite(
      uint16_t const addr, uint8_t *buff, uint16_t const size) const;

  // STUSB4500 device operations
  bool clearAlerts(bool const unmask);
  bool updatePrtStatus(void);

  bool updateRDOSnk(void);
  bool updatePDOSnk(void);
  bool setPDOSnkCount(uint8_t const count);
  bool setPDOSnk(PDO const pdo);
  bool clearPDOSrc(void);
  bool updatePDOSrc(void);

  bool sendPDCableReset(void);

  void processAlerts(void);
  void processAttach(void);

  bool pushPDMessage(uint8_t const type, uint8_t const value);
  bool popPDMessage(uint8_t &type);
  bool pushPDInterrupt(uint8_t const type);
  bool popPDInterrupt(uint8_t &type);

public:
  STUSB4500(
      uint16_t const resetPin,
      uint8_t const slaveAddress,
      TwoWire const *wire
  );
  STUSB4500(uint16_t const resetPin);

  char const *version() { return _VERSION.str(); }

  // ready the object for first use, verify I2C comms
  bool started(void) const { return _started; }
  bool begin(uint16_t const alertPin, uint16_t const attachPin);
  bool initialize(void); // (re-)initialize the device, registers, and flags

  bool ready(void) const; // check if the device is responding on I2C interface
  void waitUntilReady(void) const; // block until ready

  // (de-)assert the hardware reset pin to enable or disable power output
  void enablePower(bool const enable, bool const wait);

  void reset(bool const wait);     // hard reset using device pin
  void softReset(bool const wait); // soft reset using device register

  void update(void); // main event loop, call as frequently as possible
  void alertISR(void);
  void attachISR(void);

  CableStatus cableStatus(void) const; // check if cable attached/orientation

  void setMaxSourceCapabilityRequests(uint16_t const max)
    { _srcCapRequestMax = max; }

  bool requestSourceCapabilities(void); // request source caps from source
  bool updateSinkCapabilities(void); // retrieve sink caps from device

  size_t sourcePDOCount(void) const { return _status.pdoSrcCount; }
  PDO sourcePDO(size_t const n) const; // currently stored src PDO at index n

  size_t sinkPDOCount(void) const { return _status.pdoSnkCount; }
  PDO sinkPDO(size_t const n) const; // currently stored snk PDO at index n

  PDO requestedPDO(void); // currently requested PDO (RDO)

  // immediately set power to fixed PDO with given voltage and current
  bool setPower(uint32_t const voltage_mV, uint32_t const current_mA);
  bool setPowerDefaultUSB(void); // set fixed PDO to 5V 1.5A

  // callback registration
  void setCableAttached(USBEventCallback const callback)
    { _cableAttached = callback; }
  void setCableDetached(USBEventCallback const callback)
    { _cableDetached = callback; }
  void setSourceCapabilitiesReceived(USBEventCallback const callback)
    { _sourceCapabilitiesReceived = callback; }
};

// ------------------------------------------------------- exported variables --

/* nothing */

// ------------------------------------------------------- exported functions --

#undef VERSION_MAJ
#undef VERSION_MIN
#undef VERSION_REV

#endif /* __STUSB4500_H__ */
