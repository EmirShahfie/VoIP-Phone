#pragma once

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include <zephyr/net/socket.h>   // sockaddr, net_sa_family_t, inet_pton, etc.
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/net/net_event.h>
#include <zephyr/net/net_compat.h>

#include <string.h>
#define WIFI_SSID "YourSSID"
#define WIFI_PASSWORD "YourPassword"
#define MY_PORT 0
#define PC_PORT 5000
#define PC_IP "192.168.1.100"

int wifi_connect(void);

int wifi_disconnect(void);

void wifi_init(void);

int udp_init(void);

int udp_send(const void *data, size_t len);