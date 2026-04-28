/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Port power control API
 *
 * Shared interface between the port-power driver (74HC595 GPIO expander)
 * and consumers such as the fast-fuse overcurrent protection driver.
 *
 * There is exactly one port-power controller per system, so the API
 * uses a global instance internally — no handle needed by consumers.
 *
 * TODO(Trevor): Update the copyright date everywhere (and if we even use copyright)
 * Copyright (C) 2025 TechnipFMC Schilling Robotics
 */

#ifndef _LINUX_PORT_POWER_H
#define _LINUX_PORT_POWER_H

#include <linux/types.h>

#define PORT_POWER_MAX_PORTS	16

#if IS_ENABLED(CONFIG_PORT_POWER)

/* Returns true if the port-power driver has probed successfully */
bool port_power_available(void);

/* Trip (cut) power to a single port (called from fast-fuse IRQ thread) */
void port_power_trip(int port);

#else

static inline bool port_power_available(void) { return false; }
static inline void port_power_trip(int port) {}

#endif /* CONFIG_PORT_POWER */
#endif /* _LINUX_PORT_POWER_H */
