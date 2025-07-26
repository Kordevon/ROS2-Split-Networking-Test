#!/bin/bash
    
    # --- Configuration (ADJUST THESE TO MATCH YOUR SYSTEM & PLANNING) ---
    ETH0_IFACE="eth0"
    ETH0_IP="192.168.10.101"
    ETH0_SUBNET="192.168.10.0/24"
    ETH0_DOCKER_IP_RANGE="192.168.10.50/28"
    ETH0_SHIM_IP="192.168.10.254"
    
    ETH1_IFACE="ros_eth_usb" # <--- Use your persistent USB Ethernet name here
    ETH1_IP="192.168.20.10"
    ETH1_SUBNET="192.168.20.0/24"
    ETH1_DOCKER_IP_RANGE="192.168.20.50/28"
    ETH1_SHIM_IP="192.168.20.254"
    
    ROS_MACVLAN_NET_1="ros_macvlan_net_1"
    ROS_MACVLAN_NET_2="ros_macvlan_net_2"
    SHIM_IFACE_1="macvlan_shim_1"
    SHIM_IFACE_2="macvlan_shim_2"
    
    # --- Policy Routing Table IDs and Names ---
    TABLE_ID_1=101
    TABLE_NAME_1="subnet1_route"
    TABLE_ID_2=102
    TABLE_NAME_2="subnet2_route"
    
  
   
   
    

    
    
    
    echo "--- Enabling IP Forwarding on Host (acting as router) ---"
    sudo sysctl -w net.ipv4.ip_forward=1
    if ! grep -q "net.ipv4.ip_forward = 1" /etc/sysctl.conf; then
      echo "net.ipv4.ip_forward = 1" | sudo tee -a /etc/sysctl.conf > /dev/null
    fi
    echo "IP forwarding enabled and set for persistence."
    
    echo "--- Creating Docker Macvlan Networks ---"
    docker network rm ${ROS_MACVLAN_NET_1} 2>/dev/null || true
    docker network rm ${ROS_MACVLAN_NET_2} 2>/dev/null || true
    
    # Gateway for Docker Macvlan networks is the host's own IP on that segment.
    sudo docker network create -d macvlan \
      --subnet=${ETH0_SUBNET} \
      --gateway=${ETH0_IP} \
      --ip-range=${ETH0_DOCKER_IP_RANGE} \
      -o parent=${ETH0_IFACE} \
      ${ROS_MACVLAN_NET_1}
    echo "Created Docker network ${ROS_MACVLAN_NET_1} on ${ETH0_IFACE}"
    
    sudo docker network create -d macvlan \
      --subnet=${ETH1_SUBNET} \
      --gateway=${ETH1_IP} \
      --ip-range=${ETH1_DOCKER_IP_RANGE} \
      -o parent=${ETH1_IFACE} \
      ${ROS_MACVLAN_NET_2}
    echo "Created Docker network ${ROS_MACVLAN_NET_2} on ${ETH1_IFACE}"
    
    echo "--- Setting up Host Macvlan Shim Interfaces for Host-Container Communication ---"
    sudo ip link del ${SHIM_IFACE_1} 2>/dev/null || true
    sudo ip link del ${SHIM_IFACE_2} 2>/dev/null || true
    
    sudo ip link add ${SHIM_IFACE_1} link ${ETH0_IFACE} type macvlan mode bridge
    sudo ip addr add ${ETH0_SHIM_IP}/24 dev ${SHIM_IFACE_1}
    sudo ip link set ${SHIM_IFACE_1} up
    echo "Created host shim ${SHIM_IFACE_1} with IP ${ETH0_SHIM_IP}"
    
    sudo ip link add ${SHIM_IFACE_2} link ${ETH1_IFACE} type macvlan mode bridge
    sudo ip addr add ${ETH1_SHIM_IP}/24 dev ${SHIM_IFACE_2}
    sudo ip link set ${SHIM_IFACE_2} up
    echo "Created host shim ${SHIM_IFACE_2} with IP ${ETH1_SHIM_IP}"
    
    
    echo "--- Configuring Policy-Based Routing for Host-Originated Traffic ---"
    # Add custom routing tables to /etc/iproute2/rt_tables if they don't exist
    if ! grep -q "${TABLE_ID_1} ${TABLE_NAME_1}" /etc/iproute2/rt_tables; then
      echo "${TABLE_ID_1} ${TABLE_NAME_1}" | sudo tee -a /etc/iproute2/rt_tables > /dev/null
    fi
    if ! grep -q "${TABLE_ID_2} ${TABLE_NAME_2}" /etc/iproute2/rt_tables; then
      echo "${TABLE_ID_2} ${TABLE_NAME_2}" | sudo tee -a /etc/iproute2/rt_tables > /dev/null
    fi
    
    # Route for Subnet 1 originating traffic
    sudo ip route add ${ETH0_SUBNET} dev ${ETH0_IFACE} src ${ETH0_IP} table ${TABLE_ID_1}
    sudo ip route add ${ETH1_SUBNET} via ${ETH0_IP} dev ${ETH0_IFACE} src ${ETH0_IP} table ${TABLE_ID_1} # Route to Subnet 2 via host IP on ETH0
    sudo ip route add default via ${ETH0_DEFAULT_GW} dev ${ETH0_IFACE} table ${TABLE_ID_1} # Internet via ETH0's default gateway
    sudo ip rule add from ${ETH0_IP} lookup ${TABLE_ID_1} pref 100
    sudo ip rule add from ${ETH0_SHIM_IP} lookup ${TABLE_ID_1} pref 100 # Traffic from shim also uses this table
    echo "Policy rules for ${ETH0_IP} and ${ETH0_SHIM_IP} added to table ${TABLE_NAME_1}."
    
    
    # Route for Subnet 2 originating traffic
    sudo ip route add ${ETH1_SUBNET} dev ${ETH1_IFACE} src ${ETH1_IP} table ${TABLE_ID_2}
    sudo ip route add ${ETH0_SUBNET} via ${ETH1_IP} dev ${ETH1_IFACE} src ${ETH1_IP} table ${TABLE_ID_2} # Route to Subnet 1 via host IP on ETH1
    # IMPORTANT: If Subnet 2 should NOT have internet access, uncomment the blackhole line below
    # sudo ip route add blackhole default table ${TABLE_ID_2}
    # Otherwise, it routes via ETH0 for internet if needed (common for robots)
    sudo ip route add default via ${ETH0_DEFAULT_GW} dev ${ETH0_IFACE} table ${TABLE_ID_2} # Routes internet via ETH0's default gateway
    sudo ip rule add from ${ETH1_IP} lookup ${TABLE_ID_2} pref 101
    sudo ip rule add from ${ETH1_SHIM_IP} lookup ${TABLE_ID_2} pref 101 # Traffic from shim also uses this table
    echo "Policy rules for ${ETH1_IP} and ${ETH1_SHIM_IP} added to table ${TABLE_NAME_2}."
    
    echo "Host network setup complete with Policy-Based Routing."
    echo "Remember to set static routes on your Base Station Server to reach robot subnets."
    echo "Verify routes: 'ip rule show', 'ip route show table subnet1_route', 'ip route show table subnet2_route'"