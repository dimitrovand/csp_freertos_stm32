#include "stm32h7xx_hal.h"

/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void) {
    while (1);
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void) {
    while (1);
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void) {
    while (1);
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void) {
    while (1);
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void) {
    while (1);
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void) {
}
