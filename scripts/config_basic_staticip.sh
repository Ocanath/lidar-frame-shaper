#!/usr/bin/env bash
set -euo pipefail

usage() {
    echo "Usage: $0 -i <ip> -g <gateway> -n <interface> [-m <netmask>]"
    echo ""
    echo "  -i  Static IP address (e.g. 192.168.1.100)"
    echo "  -g  Gateway IP (e.g. 192.168.1.1), use '' to leave unset"
    echo "  -n  Network interface name (e.g. eth0, enp3s0)"
    echo "  -m  Subnet mask in dotted decimal (default: 255.255.255.0)"
    exit 1
}

mask_to_cidr() {
    local mask="$1"
    local cidr=0
    IFS='.' read -r -a octets <<< "$mask"
    for octet in "${octets[@]}"; do
        case "$octet" in
            255) cidr=$((cidr + 8)) ;;
            254) cidr=$((cidr + 7)) ;;
            252) cidr=$((cidr + 6)) ;;
            248) cidr=$((cidr + 5)) ;;
            240) cidr=$((cidr + 4)) ;;
            224) cidr=$((cidr + 3)) ;;
            192) cidr=$((cidr + 2)) ;;
            128) cidr=$((cidr + 1)) ;;
            0)   ;;
            *)
                echo "Error: invalid netmask octet '$octet'" >&2
                exit 1
                ;;
        esac
    done
    echo "$cidr"
}

IP=""
GATEWAY=""
INTERFACE=""
NETMASK="255.255.255.0"

while getopts ":i:g:n:m:" opt; do
    case "$opt" in
        i) IP="$OPTARG" ;;
        g) GATEWAY="$OPTARG" ;;
        n) INTERFACE="$OPTARG" ;;
        m) NETMASK="$OPTARG" ;;
        :) echo "Error: option -$OPTARG requires an argument." >&2; usage ;;
        \?) echo "Error: unknown option -$OPTARG" >&2; usage ;;
    esac
done

if [[ -z "$IP" || -z "$INTERFACE" ]]; then
    echo "Error: -i (ip) and -n (interface) are required." >&2
    usage
fi

# Gateway is mandatory but can be explicitly passed as empty string to skip
if [[ "${GATEWAY+x}" != "x" ]] && [[ -z "$GATEWAY" ]]; then
    echo "Error: -g (gateway) is required. Pass '' to leave it unset." >&2
    usage
fi

CIDR=$(mask_to_cidr "$NETMASK")
ADDRESS="${IP}/${CIDR}"

# Find the nmcli connection name tied to this interface
CONNECTION=$(nmcli -t -f NAME,DEVICE connection show | awk -F: -v iface="$INTERFACE" '$2 == iface {print $1; exit}')

if [[ -z "$CONNECTION" ]]; then
    CONNECTION="static-${INTERFACE}"
    echo "No existing connection found for '$INTERFACE'. Creating new profile '$CONNECTION'."
    nmcli connection add \
        type ethernet \
        con-name "$CONNECTION" \
        ifname "$INTERFACE" \
        ipv4.method manual \
        ipv4.addresses "$ADDRESS" \
        ipv4.gateway "$GATEWAY" \
        ipv6.method disabled
else
    echo "Modifying existing connection '$CONNECTION' on interface '$INTERFACE'."
    nmcli connection modify "$CONNECTION" \
        ipv4.method manual \
        ipv4.addresses "$ADDRESS" \
        ipv4.gateway "$GATEWAY" \
        ipv6.method disabled
fi

echo "  Address : $ADDRESS"
echo "  Gateway : ${GATEWAY:-"(none)"}"
echo ""

# Bring the interface up (equivalent to ifconfig up after configuration)
nmcli connection up "$CONNECTION"

echo "Done. Current address:"
ip addr show "$INTERFACE" | grep "inet "
