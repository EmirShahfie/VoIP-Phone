#include "network.h"

#include <errno.h>
static int udp_socket = -1;
static struct sockaddr_in pc_addr;

static struct net_mgmt_event_callback wifi_cb;

int wifi_connect(void)
{
    struct net_if *iface = net_if_get_default();

    struct wifi_connect_req_params cnx_params = {
        .ssid = WIFI_SSID,
        .ssid_length = strlen(WIFI_SSID),
        .psk = WIFI_PASSWORD,
        .psk_length = strlen(WIFI_PASSWORD),
        .security = WIFI_SECURITY_TYPE_PSK,
        .channel = WIFI_CHANNEL_ANY,
    };

    int ret = net_mgmt(NET_REQUEST_WIFI_CONNECT, iface,
                       &cnx_params, sizeof(cnx_params));
    if (ret) {
        printk("WiFi connect request failed: %d\n", ret);
        return ret;
    }

    printk("WiFi connection initiated\n");
    return 0;
}

static void wifi_event_handler(struct net_mgmt_event_callback *cb,
                               uint64_t mgmt_event,
                               struct net_if *iface)
{
    ARG_UNUSED(cb);
    ARG_UNUSED(iface);

    if (mgmt_event == NET_EVENT_WIFI_CONNECT_RESULT) {
        const struct wifi_status *status =
            (const struct wifi_status *)cb->info;

        if (status && status->status) {
            printk("WiFi connect failed: %d\n", status->status);
        } else {
            printk("WiFi connected\n");
        }

    } else if (mgmt_event == NET_EVENT_WIFI_DISCONNECT_RESULT) {
        printk("WiFi disconnected\n");

    } else if (mgmt_event == NET_EVENT_IPV4_ADDR_ADD) {
        printk("Got IPv4 address\n");
    }
}

void wifi_init(void)
{
    net_mgmt_init_event_callback(&wifi_cb, wifi_event_handler,
        NET_EVENT_WIFI_CONNECT_RESULT |
        NET_EVENT_WIFI_DISCONNECT_RESULT |
        NET_EVENT_IPV4_ADDR_ADD);

    net_mgmt_add_event_callback(&wifi_cb);
}

int wifi_disconnect(void)
{
    struct net_if *iface = net_if_get_default();

    int ret = net_mgmt(NET_REQUEST_WIFI_DISCONNECT, iface, NULL, 0);
    if (ret) {
        printk("WiFi disconnect request failed: %d\n", ret);
        return ret;
    }

    printk("WiFi disconnection initiated\n");
    return 0;
}

int udp_init(void)
{
    if (udp_socket >= 0) {
        zsock_close(udp_socket);
        udp_socket = -1;
    }

    udp_socket = zsock_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (udp_socket < 0) {
        printk("zsock_socket failed: errno=%d\n", errno);
        return -errno;
    }

    memset(&pc_addr, 0, sizeof(pc_addr));
    pc_addr.sin_family = AF_INET;
    pc_addr.sin_port = htons(PC_PORT);

    int ret = zsock_inet_pton(AF_INET, PC_IP, &pc_addr.sin_addr);
    if (ret != 1) {
        printk("zsock_inet_pton failed: errno=%d\n", errno);
        zsock_close(udp_socket);
        udp_socket = -1;
        return -EINVAL;
    }

    ret = zsock_connect(udp_socket, (struct sockaddr *)&pc_addr, sizeof(pc_addr));
    if (ret < 0) {
        printk("zsock_connect failed: errno=%d\n", errno);
        zsock_close(udp_socket);
        udp_socket = -1;
        return -errno;
    }

    printk("UDP connected to %s:%d\n", PC_IP, PC_PORT);
    return 0;
}

int udp_send(const void *data, size_t len)
{
    if (udp_socket < 0) {
        return -ENOTCONN;
    }

    int ret = zsock_send(udp_socket, data, len, 0);
    if (ret < 0) {
        return -errno;
    }
    return ret;
}

int udp_receive(void *buffer, size_t len)
{
    if (udp_socket < 0) {
        return -ENOTCONN;
    }

    int ret = zsock_recv(udp_socket, buffer, len, 0);
    if (ret < 0) {
        return -errno;
    }
    return ret;
}