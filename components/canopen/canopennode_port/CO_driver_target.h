/*
 * CANopenNode target definitions for ESPHome (esphome-canopen component).
 *
 * Generic part only: data types, locking and the default stack configuration. The driver functions
 * (CO_CANmodule_init(), CO_CANsend(), ...) are implemented by the ESPHome component on top of its
 * canbus abstraction. Everything runs on the ESPHome main loop, so the locks are empty.
 *
 * Lives in the component, not in the CANopenNode library. This directory is added by codegen as a
 * header-only package (library.json on Zephyr/PlatformIO; on ESP32 codegen copies it into a generated IDF component),
 * so CANopenNode's 301/CO_driver.h finds it.
 *
 * Every CO_CONFIG_* below is a default and can be overridden with a -D build flag.
 */

#pragma once
#ifndef CO_DRIVER_TARGET_H
#define CO_DRIVER_TARGET_H

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef CO_DRIVER_CUSTOM
#include "CO_driver_custom.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Object dictionary is built at runtime by the application */
#ifndef CO_MULTIPLE_OD
#define CO_MULTIPLE_OD
#endif

/* Stack configuration: only what the component uses. Services whose default is on but whose sources aren't built
 * (LEDs, LSS) or that aren't used yet (TIME, SYNC) are turned off, the other services default to off; PDO and storage
 * (0x1010 / 0x1011, for the PDO parameters) are enabled by codegen (enable_pdo()).
 * The SDO server is segmented only: block transfer needs a buffer of at least 900 bytes. */
#ifndef CO_CONFIG_SDO_SRV
#define CO_CONFIG_SDO_SRV (CO_CONFIG_SDO_SRV_SEGMENTED)
#endif
#ifndef CO_CONFIG_SDO_SRV_BUFFER_SIZE
#define CO_CONFIG_SDO_SRV_BUFFER_SIZE 32
#endif
#ifndef CO_CONFIG_NMT
#define CO_CONFIG_NMT (CO_CONFIG_NMT_CALLBACK_CHANGE)
#endif
#ifndef CO_CONFIG_HB_CONS
#define CO_CONFIG_HB_CONS (CO_CONFIG_HB_CONS_ENABLE | CO_CONFIG_HB_CONS_CALLBACK_MULTI | CO_CONFIG_HB_CONS_QUERY_FUNCT)
#endif
#ifndef CO_CONFIG_EM
#define CO_CONFIG_EM (CO_CONFIG_EM_PRODUCER)
#endif
#ifndef CO_CONFIG_TIME
#define CO_CONFIG_TIME (0)
#endif
#ifndef CO_CONFIG_SYNC
#define CO_CONFIG_SYNC (0)
#endif
#ifndef CO_CONFIG_PDO
#define CO_CONFIG_PDO (0)
#endif
#ifndef CO_CONFIG_STORAGE
#define CO_CONFIG_STORAGE (0)
#endif
#ifndef CO_CONFIG_LEDS
#define CO_CONFIG_LEDS (0)
#endif
#ifndef CO_CONFIG_LSS
#define CO_CONFIG_LSS (0)
#endif

/* Basic definitions */
#define CO_LITTLE_ENDIAN
#define CO_SWAP_16(x) x
#define CO_SWAP_32(x) x
#define CO_SWAP_64(x) x
typedef uint_fast8_t bool_t;
typedef float float32_t;
typedef double float64_t;

/* Received CAN message, passed to CANrx_callback() */
typedef struct {
  uint32_t ident;
  uint8_t DLC;
  uint8_t data[8];
} CO_CANrxMsg_t;

#define CO_CANrxMsg_readIdent(msg) ((uint16_t) (((CO_CANrxMsg_t *) (msg))->ident))
#define CO_CANrxMsg_readDLC(msg) ((uint8_t) (((CO_CANrxMsg_t *) (msg))->DLC))
#define CO_CANrxMsg_readData(msg) ((const uint8_t *) (((CO_CANrxMsg_t *) (msg))->data))

/* Received message object */
typedef struct {
  uint16_t ident; /* 11-bit identifier, bit 11 = RTR */
  uint16_t mask;
  void *object;
  void (*CANrx_callback)(void *object, void *message);
} CO_CANrx_t;

/* Transmit message object */
typedef struct {
  uint32_t ident;
  uint8_t DLC;
  uint8_t data[8];
  bool_t bufferFull;
  bool_t syncFlag;
} CO_CANtx_t;

/* CAN module object */
/* no volatile members: the driver runs on the main loop only, never in interrupt context */
typedef struct {
  void *CANptr; /* the ESPHome component owning this module */
  CO_CANrx_t *rxArray;
  uint16_t rxSize;
  CO_CANtx_t *txArray;
  uint16_t txSize;
  uint16_t CANerrorStatus;
  bool_t CANnormal;
  bool_t useCANrxFilters;
  bool_t bufferInhibitFlag;
  bool_t firstCANtxMessage;
  uint16_t CANtxCount;
  uint32_t errOld;
} CO_CANmodule_t;

/* Data storage object for one entry (CO_storage, 0x1010 / 0x1011): the hub's callbacks store the PDO parameters in
 * ESPHome preferences, so only the CO_driver.h fields CO_storage.c uses are needed */
typedef struct {
  void *addr;
  size_t len;
  uint8_t subIndexOD;
  uint8_t attr;
  void *addrNV;
} CO_storage_entry_t;

/* Single-threaded (ESPHome main loop): no locking */
#define CO_LOCK_CAN_SEND(CAN_MODULE)
#define CO_UNLOCK_CAN_SEND(CAN_MODULE)
#define CO_LOCK_EMCY(CAN_MODULE)
#define CO_UNLOCK_EMCY(CAN_MODULE)
#define CO_LOCK_OD(CAN_MODULE)
#define CO_UNLOCK_OD(CAN_MODULE)

#define CO_MemoryBarrier()
#define CO_FLAG_READ(rxNew) ((rxNew) != NULL)
#define CO_FLAG_SET(rxNew) \
  { \
    CO_MemoryBarrier(); \
    rxNew = (void *) 1L; \
  }
#define CO_FLAG_CLEAR(rxNew) \
  { \
    CO_MemoryBarrier(); \
    rxNew = NULL; \
  }

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* CO_DRIVER_TARGET_H */
