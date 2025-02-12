/*change function: 
    previous: 1. run dijstra only on gateway
                2. no duplicated receive    
                3.check the adress for unicast
    new: use unicast not boardcast*/
#include "contiki.h"
#include "net/rime/rime.h"
#include "net/netstack.h"
#include "dev/leds.h"
#include "dev/adc-zoul.h"
#include "dev/zoul-sensors.h"
#include "dev/sys-ctrl.h"
#include "dev/cc2538-temp-sensor.h"
#include <stdio.h>
#include <limits.h>
#include <string.h>
#include "dev/watchdog.h"  // 添加头文件
#include "net/mac/contikimac/contikimac.h"
#include "project-conf.h"

#ifndef NODE_ID
#define NODE_ID 20   // 默认节点ID
#endif
// Constants
#define SENSOR_READ_INTERVAL (CLOCK_SECOND * 5)  // 传感器读取间隔
#define ROUTING_INTERVAL (CLOCK_SECOND * 15)      // 路由更新间隔
#define MAX_NODES 8                              // 最大节点数（0-7）
#define GATEWAY_ADDR 1                           // 网关地址固定为1
#define INF 0x7FFF                              // 无穷大值
#define MAX_PATH_LENGTH MAX_NODES               // 最大路径长度
#define EMERGENCY_TEMP_THRESHOLD 30  // 紧急温度阈值（40°C）
#define DATA_SEND_INTERVAL (CLOCK_SECOND * 10)  // 数据发送间隔（1分钟）
#define MAX_ROUTE_ENTRIES 32
// #define WATCHDOG_DISABLE
// 节点类型定义
#define NODE_TYPE_SENSOR 1
#define NODE_TYPE_RELAY 2
#define NODE_TYPE_GATEWAY 3

// 包类型定义
#define PACKET_TYPE_ROUTING 1
#define PACKET_TYPE_DATA 2
#define PACKET_TYPE_EMERGENCY 3
#define PACKET_TYPE_ROUTE_UPDATE 4  // 新的包类型，用于网关向中继节点广播路由信息
// Dijkstra算法上下文
typedef struct {
    int16_t cost[MAX_NODES][MAX_NODES];  // 邻接矩阵
    uint8_t visited[MAX_NODES];          // 访问标记
    uint16_t distance[MAX_NODES];        // 到源点的距离
    uint16_t parent[MAX_NODES];          // 路径中的父节点
} dijkstra_ctx_t;

// 节点链路信息
typedef struct {
    uint16_t addr;         // 节点地址
    int16_t rssi;         // 信号强度
    clock_time_t last_seen; // 最后一次收到该节点消息的时间
    uint8_t is_active;    // 节点是否活跃
} node_link_t;

// 路由包结构
typedef struct {
    uint8_t type;         // 包类型：PACKET_TYPE_ROUTING
    uint16_t source_addr; // 源节点
    uint16_t from_addr;   // 发送节点
    int16_t rssi_sum;    // RSSI累加值
    uint8_t hop_count;   // 跳数
} routing_packet_t;

// 修改后的数据包结构，添加路径跟踪信息
typedef struct {
    uint8_t type;              // 包类型：PACKET_TYPE_DATA
    uint16_t source_addr;      // 源节点
    uint16_t dest_addr;        // 目标节点
    uint8_t distance_status;   // 距离状态
    int16_t temperature;       // 温度值
    uint16_t next_hop;        // 下一跳节点
    uint8_t hop_count;        // 跳数
    uint16_t path[MAX_PATH_LENGTH];  // 路径记录
    uint8_t path_length;      // 当前路径长度
    uint8_t ttl;             // 添加TTL字段
} data_packet_t;

// 添加共享数据结构
typedef struct {
    int16_t temperature;       // 温度数据
    float distance;           // 距离数据
    uint8_t distance_status;  // 距离状态
    clock_time_t last_update; // 最后更新时间
} sensor_data_t;

// 定义紧急消息包结构
typedef struct {
    uint8_t type;              // 包类型：PACKET_TYPE_EMERGENCY
    uint16_t source_addr;      // 源节点
    int16_t temperature;       // 温度值
    uint8_t seq_no;           // 序列号，用于flooding去重
    uint8_t ttl;             // 新增TTL字段

} emergency_packet_t;
// 网关广播的路由更新包结构
typedef struct {
    uint8_t type;              // 包类型：新定义一个PACKET_TYPE_ROUTE_UPDATE
    uint16_t source_addr;      // 源节点(网关)
    uint16_t target_node;      // 目标节点
    uint16_t next_hop;         // 该目标节点的下一跳
} route_update_packet_t;
// 中继节点的路由表项
typedef struct {
    uint16_t dest_addr;    // 目标节点地址
    uint16_t next_hop;     // 下一跳节点地址
    clock_time_t last_update;  // 最后更新时间
} relay_route_entry_t;
// 全局变量
static node_link_t node_links[MAX_NODES];    // 节点链路信息表
static uint8_t link_count = 0;               // 已知节点数量
static const uint16_t node_addr = NODE_ID;   // 本节点地址
static uint8_t node_type;                    // 节点类型
static struct broadcast_conn broadcast;       // 广播连接
static struct unicast_conn unicast;             //单播连接
static dijkstra_ctx_t ctx;                   // Dijkstra上下文
static sensor_data_t current_sensor_data;
static uint8_t emergency_seq = 0;  // 紧急消息序列号
static uint8_t received_emergency_msgs[MAX_NODES];  // 记录已收到的紧急消息序列号
static relay_route_entry_t relay_route_table[MAX_ROUTE_ENTRIES];  // 中继节点的路由表
static uint8_t route_entry_count = 0;  // 当前路由表项数量
// 函数声明
static void init_node_type(void);
static void update_node_link(uint16_t addr, int16_t rssi);
static void build_adjacency_matrix(void);
static void run_dijkstra(uint16_t source);
static uint16_t find_next_hop(uint16_t dest);
static void broadcast_route_info(void);
static float calculate_distance(void);
static void print_routing_table(void);
static void broadcast_route_updates(void);
static void cleanup_route_table(void);
static void unicast_recv(struct unicast_conn *c, const linkaddr_t *from);
static const struct unicast_callbacks unicast_call = {unicast_recv};
static void broadcast_recv(struct broadcast_conn *c, const linkaddr_t *from);
static const struct broadcast_callbacks broadcast_call = {broadcast_recv};

// 初始化节点类型
static void init_node_type(void) {
    if (node_addr == GATEWAY_ADDR) {
        node_type = NODE_TYPE_GATEWAY;
        printf("Initialized as Gateway node\n");
    }
    else if (node_addr >= 2 && node_addr <= 10) {  // 2-10号为中继节点
        node_type = NODE_TYPE_RELAY;
        printf("Initialized as Relay node\n");
    }
    else {                                         // 其余的为传感器节点
        node_type = NODE_TYPE_SENSOR;
        printf("Initialized as Sensor node\n");
    }
}

// 更新节点链路信息
static void update_node_link(uint16_t addr, int16_t rssi) {
    if(addr == node_addr) {
        return;
    }
    uint8_t i;
    for(i = 0; i < link_count; i++) {
        if(node_links[i].addr == addr) {
            node_links[i].rssi = rssi;
            node_links[i].last_seen = clock_time();
            node_links[i].is_active = 1;
            // printf("\n╔═══════════════ LINK UPDATE ═══════════════╗\n");
            // printf("║ Node: %-3d updated link to Node: %-3d      ║\n", node_addr, addr);
            // printf("║ RSSI: %-4d, Status: Active                ║\n", rssi);
            // printf("╚═════════════════════════════════════════════╝\n");
            return;
        }
    }
    
    if(link_count < MAX_NODES) {
        node_links[link_count].addr = addr;
        node_links[link_count].rssi = rssi;
        node_links[link_count].last_seen = clock_time();
        node_links[link_count].is_active = 1;
        // printf("\n╔═══════════════ NEW LINK ═══════════════╗\n");
        // printf("║ Node: %-3d discovered Node: %-3d        ║\n", node_addr, addr);
        // printf("║ RSSI: %-4d, Status: Active            ║\n", rssi);
        // printf("╚═════════════════════════════════════════╝\n");
        link_count++;
    }
}
static void broadcast_route_updates(void) {
   if(node_type != NODE_TYPE_GATEWAY) return;
   
   printf("\n===== Gateway Route Updates =====\n");
   printf("Building routes for gateway (1)...\n");

   // 先发送到网关自己的路由
   route_update_packet_t rup;
   rup.type = PACKET_TYPE_ROUTE_UPDATE;
   rup.source_addr = node_addr;
   rup.target_node = GATEWAY_ADDR;
   rup.next_hop = GATEWAY_ADDR;
   packetbuf_copyfrom(&rup, sizeof(route_update_packet_t));
   broadcast_send(&broadcast);
   
   // 为活跃节点发送路由
   for(uint8_t i = 0; i < link_count; i++) {
       if(node_links[i].is_active && node_links[i].addr != GATEWAY_ADDR) {
           printf("Active node found: %d\n", node_links[i].addr);
           uint16_t next_hop = node_links[i].addr; // 直接使用目标节点作为下一跳
           
           rup.target_node = node_links[i].addr;
           rup.next_hop = next_hop;
           
           packetbuf_copyfrom(&rup, sizeof(route_update_packet_t));
           broadcast_send(&broadcast);
           printf("Sent route: target=%d, next_hop=%d\n", 
                  node_links[i].addr, next_hop);
       }
   }
   printf("===== End Route Updates =====\n");
}
// 构建邻接矩阵
// 修改build_adjacency_matrix函数，优化性能
static void build_adjacency_matrix(void) {
    watchdog_periodic();
    uint8_t i, j;
    
    // 使用memset来初始化矩阵
    memset(ctx.cost, 0xFF, sizeof(ctx.cost));  // 0xFF会被设置为INF
    
    // 设置对角线为0
    for(i = 0; i < MAX_NODES; i++) {
        ctx.cost[i][i] = 0;
    }
    
    // 填充邻接矩阵
    for(i = 0; i < link_count; i++) {
        watchdog_periodic();  // 在循环中添加看门狗重置
        if(node_links[i].is_active) {
            int16_t cost = -node_links[i].rssi;
            ctx.cost[node_addr][node_links[i].addr] = cost;
            ctx.cost[node_links[i].addr][node_addr] = cost;
        }
    }
}

// 运行Dijkstra算法
static void run_dijkstra(uint16_t source) {
    watchdog_periodic();
    uint8_t i, count;
    
    // 初始化
    for(i = 0; i < MAX_NODES; i++) {
        ctx.visited[i] = 0;
        ctx.distance[i] = INF;
        ctx.parent[i] = 0;
    }
    ctx.distance[source] = 0;
    
    // Dijkstra主循环
    for(count = 0; count < MAX_NODES - 1; count++) {
        watchdog_periodic();
        uint16_t min = INF;
        uint8_t min_index = 0;
        
        // 找到未访问节点中距离最小的
        for(i = 0; i < MAX_NODES; i++) {
            if(!ctx.visited[i] && ctx.distance[i] <= min) {
                min = ctx.distance[i];
                min_index = i;
            }
        }
        
        ctx.visited[min_index] = 1;
        
        // 更新邻接节点的距离
        for(i = 0; i < MAX_NODES; i++) {
            if(!ctx.visited[i] && 
               ctx.cost[min_index][i] != INF && 
               ctx.distance[min_index] != INF && 
               ctx.distance[min_index] + ctx.cost[min_index][i] < ctx.distance[i]) {
                
                ctx.distance[i] = ctx.distance[min_index] + ctx.cost[min_index][i];
                ctx.parent[i] = min_index;
            }
        }
    }
}

// 找到到达目标节点的下一跳
static uint16_t find_next_hop(uint16_t dest) {
    if(node_type == NODE_TYPE_SENSOR) {
        // 传感器节点逻辑保持不变
        int16_t best_rssi = -128;
        uint16_t best_next = 0;
        
        for(uint8_t i = 0; i < link_count; i++) {
            if(node_links[i].is_active && 
               node_links[i].addr >= 2 && node_links[i].addr <= 10 && // Only consider relay nodes
               node_links[i].rssi > best_rssi) {
                best_rssi = node_links[i].rssi;
                best_next = node_links[i].addr;
            }
        }
        return best_next;
    }
    else if(node_type == NODE_TYPE_GATEWAY) {
        // 网关节点使用Dijkstra算法
        build_adjacency_matrix();
        run_dijkstra(node_addr);
        
        uint16_t current = dest;
        uint16_t next = dest;
        
        while(ctx.parent[current] != node_addr && current != node_addr) {
            next = current;
            current = ctx.parent[current];
        }
        return next;
    }
    else if(node_type == NODE_TYPE_RELAY) {
    cleanup_route_table();  // Clean expired routes
    
    // First try routing table
    for(uint8_t i = 0; i < route_entry_count; i++) {
        if(relay_route_table[i].dest_addr == dest) {
            return relay_route_table[i].next_hop;
        }
    }
    
    // If no route found, forward to gateway or strongest relay/gateway signal
    int16_t best_rssi = -128;
    uint16_t best_next = 0;
    
    for(uint8_t i = 0; i < link_count; i++) {
        if(node_links[i].is_active && 
           (node_links[i].addr == GATEWAY_ADDR || 
            (node_links[i].addr >= 2 && node_links[i].addr <= 10)) && 
           node_links[i].rssi > best_rssi) {
            best_rssi = node_links[i].rssi;
            best_next = node_links[i].addr;
        }
    }
    return best_next;
}
}
static void cleanup_route_table(void) {
    clock_time_t current_time = clock_time();
    uint8_t i = 0;
    
    while (i < route_entry_count) {
        if (current_time - relay_route_table[i].last_update >= ROUTING_INTERVAL * 3) {
            // Remove expired entry by shifting remaining entries
            memmove(&relay_route_table[i], 
                    &relay_route_table[i + 1], 
                    (route_entry_count - i - 1) * sizeof(relay_route_entry_t));
            route_entry_count--;
        } else {
            i++;
        }
    }
}
// 广播路由信息
static void broadcast_route_info(void) {
    routing_packet_t rp;
    rp.type = PACKET_TYPE_ROUTING;
    rp.source_addr = node_addr;
    rp.from_addr = node_addr;
    rp.rssi_sum = 0;
    rp.hop_count = 0;
    // printf("\n╔═══════════════ BROADCASTING ROUTE INFO ════════════╗\n");
    // printf("║ Node: %-3d broadcasting route information          ║\n", node_addr);
    // printf("╚═════════════════════════════════════════════════════╝\n");
    packetbuf_copyfrom(&rp, sizeof(routing_packet_t));
    broadcast_send(&broadcast);
}

// 计算距离（仅传感器节点使用）
static float calculate_distance(void) {
    if(node_type == NODE_TYPE_SENSOR) {
        uint16_t adc_value = adc_zoul.value(ZOUL_SENSORS_ADC1) >> 4;
        float voltage = (adc_value / 4095.0) * 3.3;
        float voltage_ratio = voltage / 3.3;
        float distance = 4.8/(voltage_ratio - 0.02);
        
        if (distance < 10.0) distance = 10.0;
        if (distance > 80.0) distance = 80.0;
        printf("Distance: %d cm\n", (int)distance);

        return distance;
    }
    return 0;
}

// 打印路由表
static void print_routing_table(void) {
    printf("\n╔════════════════════ NODE %d ROUTING TABLE ════════════════════╗\n", node_addr);
    printf("║ Neighbor | RSSI | Last Seen (s) | Status    | Role          ║\n");
    printf("╠═════════╪══════╪══════════════╪═══════════╪═══════════════╣\n");
    
    for(uint8_t i = 0; i < link_count; i++) {
        char *role;
        if(node_links[i].addr == GATEWAY_ADDR) role = "Gateway";
        else if(node_links[i].addr <= 10) role = "Relay";
        else role = "Sensor";
        
        printf("║ %8d | %4d | %12lu | %-9s | %-13s ║\n",
               node_links[i].addr,
               node_links[i].rssi,
               (unsigned long)(clock_time() - node_links[i].last_seen) / CLOCK_SECOND,
               node_links[i].is_active ? "Active" : "Inactive",
               role);
    }
    printf("╚═════════╧══════╧══════════════╧═══════════╧═══════════════╝\n");
}

// 广播回调函数
static void broadcast_recv(struct broadcast_conn *c, const linkaddr_t *from) {
    watchdog_periodic();
    leds_on(LEDS_GREEN);
    
    int16_t rssi = packetbuf_attr(PACKETBUF_ATTR_RSSI);
    uint8_t *data = packetbuf_dataptr();
    uint8_t packet_type = data[0];

    // 处理紧急消息 - 保持不变，因为需要广播
    if(packet_type == PACKET_TYPE_EMERGENCY) {
        emergency_packet_t *ep = (emergency_packet_t *)data;
        
        if(received_emergency_msgs[ep->source_addr] != ep->seq_no) {
            received_emergency_msgs[ep->source_addr] = ep->seq_no;
            
            if(node_type == NODE_TYPE_GATEWAY) {
                printf("\n╔══════════════ EMERGENCY MESSAGE ══════════════╗\n");
                printf("║ Source: Node %d                                ║\n", 
                       ep->source_addr);
                printf("║ Temperature: %d.%d°C                           ║\n", 
                       ep->temperature/1000, (ep->temperature%1000)/100);
                printf("║ RSSI: %d                                       ║\n", 
                       rssi);
                printf("╚══════════════════════════════════════════════╝\n");
            }
            else if(ep->ttl > 0) {  // 只有TTL大于0才转发
                ep->ttl--;          // 减少TTL
                packetbuf_copyfrom(ep, sizeof(emergency_packet_t));
                broadcast_send(c);
                
                printf("[Node %d] Forward emergency from Node %d, TTL=%d\n", 
                       node_addr, ep->source_addr, ep->ttl);
            }
        }
        return;
    }    
    // 处理路由发现消息 - 保持广播
    else if(packet_type == PACKET_TYPE_ROUTING) {
        routing_packet_t *rp = (routing_packet_t *)data;
        update_node_link(rp->source_addr, rssi);
        
        if(rp->from_addr != node_addr) {
            update_node_link(rp->from_addr, rssi);
        }
        
        if(rp->hop_count < 2 && rp->source_addr != node_addr) {
            rp->hop_count++;
            rp->from_addr = node_addr;
            rp->rssi_sum += rssi;
            
            packetbuf_copyfrom(rp, sizeof(routing_packet_t));
            broadcast_send(c);
        }
    }
    // 处理路由更新消息 - 保持广播
    else if(packet_type == PACKET_TYPE_ROUTE_UPDATE && node_type == NODE_TYPE_RELAY) {
        route_update_packet_t *rup = (route_update_packet_t *)data;
        cleanup_route_table();
        
        printf("\n===== Relay Received Route Update =====\n");
        printf("From Gateway: %d\n", rup->source_addr);
        printf("Target Node: %d\n", rup->target_node);
        printf("Next Hop: %d\n", rup->next_hop);
        
        uint8_t found = 0;
        for(uint8_t i = 0; i < route_entry_count; i++) {
            if(relay_route_table[i].dest_addr == rup->target_node) {
                relay_route_table[i].next_hop = rup->next_hop;
                relay_route_table[i].last_update = clock_time();
                found = 1;
                break;
            }
        }
        
        if(!found && route_entry_count < MAX_ROUTE_ENTRIES) {
            relay_route_table[route_entry_count].dest_addr = rup->target_node;
            relay_route_table[route_entry_count].next_hop = rup->next_hop;
            relay_route_table[route_entry_count].last_update = clock_time();
            route_entry_count++;
        }
    }
    // 数据包相关的处理全部移到unicast_recv中
    
    leds_off(LEDS_GREEN);
}

static void unicast_recv(struct unicast_conn *c, const linkaddr_t *from) {
    printf("Node %d (addr %02x:%02x) received unicast from %02x:%02x\n", 
       node_addr, 
       linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],
       from->u8[0], from->u8[1]);
    watchdog_periodic();
    leds_on(LEDS_GREEN);
    
    int16_t rssi = packetbuf_attr(PACKETBUF_ATTR_RSSI);
    uint8_t *data = packetbuf_dataptr();
    uint8_t packet_type = data[0];
    printf("[Node %d] Received unicast packet type: %d\n", node_addr, packet_type);  // 调试信息1

    if(packet_type == PACKET_TYPE_DATA) {
        data_packet_t *dp = (data_packet_t *)data;
        update_node_link(dp->source_addr, rssi);
        printf("[Node %d] Data packet from Node %d to Node %d\n", 
        node_addr, dp->source_addr, dp->dest_addr);  // 调试信息2
        if(node_type == NODE_TYPE_GATEWAY) {
            /*if(dp->source_addr > 10 && dp->hop_count == 1) {
                printf("\n[Gateway] Ignoring direct packet from sensor Node %d, waiting for relayed packet...\n", 
                   dp->source_addr);
                leds_off(LEDS_GREEN);
            return;
            }*/
           printf("[Gateway] Processing data packet, source=%d, hop=%d\n", 
                   dp->source_addr, dp->hop_count);  // 调试信息3
            int temp_whole = dp->temperature / 1000;
            int temp_decimal = (dp->temperature % 1000) / 100;
            if (temp_whole < 0) temp_whole = -temp_whole;
            if (temp_decimal < 0) temp_decimal = -temp_decimal;
            
            // 打印路径信息
            char path_str[128] = {0};
            int pos = 0;
            for(int i = 0; i < dp->path_length; i++) {
                pos += snprintf(path_str + pos, sizeof(path_str) - pos, 
                              "%d%s", dp->path[i], 
                              (i < dp->path_length - 1) ? " -> " : "");
            }
            
            printf("\n╔═══════════════ RECEIVED DATA ═════════════╗\n");
            printf("║ Source: Node %-3d                           ║\n", dp->source_addr);
            printf("║ Temperature: %d.%d°C                        ║\n", temp_whole, temp_decimal);
            printf("║ Status: %-5s                                ║\n", 
                   dp->distance_status ? "Free" : "Occupied");
            printf("║ Hop Count: %-3d                             ║\n", dp->hop_count);
            printf("║ RSSI: %-4d                                  ║\n", rssi);
            printf("║ Path: %-40s ║\n", path_str);
            printf("╚═════════════════════════════════════════════╝\n");
        }
        else if(dp->dest_addr == GATEWAY_ADDR && dp->source_addr != node_addr) {
            // 添加中继节点的调试信息
            printf("[Relay %d] Got data: src=%d, hop=%d, ttl=%d\n", 
                   node_addr, dp->source_addr, dp->hop_count, dp->ttl);
            
            if(dp->ttl == 0) {
                printf("[Relay %d] TTL expired for packet from Node %d\n", 
                       node_addr, dp->source_addr);
                leds_off(LEDS_GREEN);
                return;
            }
            
            dp->ttl--;
            if(dp->path_length < MAX_PATH_LENGTH) {
                dp->path[dp->path_length++] = node_addr;
            }
            
            dp->hop_count++;
            
            // 找到下一跳
            uint16_t next_hop = find_next_hop(GATEWAY_ADDR);
            if(next_hop != 0) {
                dp->next_hop = next_hop;
                
                // 创建目标地址
                linkaddr_t addr;
                addr.u8[1] = next_hop & 0xff;
                addr.u8[0] = (next_hop >> 8) & 0xff;
                
                packetbuf_copyfrom(dp, sizeof(data_packet_t));
                unicast_send(c, &addr);
                printf("[Relay %d] Forward: src=%d to next=%d, path=", 
                       node_addr, dp->source_addr, next_hop);
                // 打印当前路径
                for(int i = 0; i < dp->path_length; i++) {
                    printf("%d->", dp->path[i]);
                }
                printf("\n");
            } else {
                printf("[Relay %d] No route to gateway for packet from %d\n", 
                       node_addr, dp->source_addr);
            }
        }
    }
    
    leds_off(LEDS_GREEN);
}












PROCESS(init_process, "Initialization Process");
PROCESS(emergency_process, "Emergency Message Process");
PROCESS(sensor_reading_process, "Sensor Reading Process");
PROCESS(data_sending_process, "Data Sending Process");
PROCESS(discovery_process, "Route Discovery Process");
AUTOSTART_PROCESSES(&init_process);


// 紧急消息处理进程

PROCESS_THREAD(emergency_process, ev, data) {
    PROCESS_BEGIN();
    
    while(1) {
        PROCESS_WAIT_EVENT();
        
        if(ev == PROCESS_EVENT_MSG) {
            // 发送紧急消息
            emergency_packet_t ep;
            ep.type = PACKET_TYPE_EMERGENCY;
            ep.source_addr = node_addr;
            ep.temperature = current_sensor_data.temperature;
            ep.seq_no = ++emergency_seq;
            ep.ttl = 4;  // 设置初始TTL值为4，可以根据网络规模调整

            packetbuf_copyfrom(&ep, sizeof(emergency_packet_t));
            broadcast_send(&broadcast);
            
            printf("\n╔═══════════════ EMERGENCY ALERT ═══════════════╗\n");
            printf("║ HIGH TEMPERATURE DETECTED: %d.%d°C              ║\n", 
                   ep.temperature/1000, (ep.temperature%1000)/100);
            printf("║ Node: %d                                        ║\n", 
                   node_addr);
            printf("╚══════════════════════════════════════════════╝\n");
        }
    }
    
    PROCESS_END();
}
// 传感器数据采集进程

PROCESS_THREAD(sensor_reading_process, ev, data) {
    static struct etimer sensor_timer;
    
    PROCESS_BEGIN();
    
    if(node_type == NODE_TYPE_SENSOR) {
        // 初始化ADC和传感器
        adc_zoul.configure(SENSORS_HW_INIT, ZOUL_SENSORS_ADC1);
        etimer_set(&sensor_timer, SENSOR_READ_INTERVAL);
        
        while(1) {
            PROCESS_WAIT_EVENT();
            
            if(etimer_expired(&sensor_timer)) {
                // 更新传感器数据
                current_sensor_data.temperature = 
                    cc2538_temp_sensor.value(CC2538_SENSORS_VALUE_TYPE_CONVERTED);
                if(current_sensor_data.temperature < 0) {
                    current_sensor_data.temperature = -current_sensor_data.temperature;
                }
                // printf(" %d \n",current_sensor_data.temperature);
                current_sensor_data.distance = calculate_distance();
                current_sensor_data.distance_status = 
                current_sensor_data.distance < 20.0 ? 0 : 1;
                if(current_sensor_data.distance_status) {
                    leds_on(LEDS_BLUE);  // Turn on blue LED when free (distance >= 25cm)
                } else {
                    leds_off(LEDS_BLUE); // Turn off blue LED when occupied (distance < 25cm)
                }
                current_sensor_data.last_update = clock_time();

                // 检查是否需要发送紧急消息
                if(current_sensor_data.temperature >= EMERGENCY_TEMP_THRESHOLD * 1000) {
                    process_post(&emergency_process, PROCESS_EVENT_MSG, NULL);
                }
                
                etimer_reset(&sensor_timer);
            }
        }
    }
    
    PROCESS_END();
}

// 常规数据发送进程

PROCESS_THREAD(data_sending_process, ev, data) {
    static struct etimer send_timer;
    
    PROCESS_BEGIN();
    
    if(node_type == NODE_TYPE_SENSOR) {
        etimer_set(&send_timer, DATA_SEND_INTERVAL);
        
        while(1) {
            PROCESS_WAIT_EVENT();
            
            if(etimer_expired(&send_timer)) {
                uint16_t next_hop = find_next_hop(GATEWAY_ADDR);
                if(next_hop != 0) {
                    data_packet_t dp;
                    dp.type = PACKET_TYPE_DATA;
                    dp.source_addr = node_addr;
                    dp.dest_addr = GATEWAY_ADDR;
                    dp.temperature = current_sensor_data.temperature;
                    dp.distance_status = current_sensor_data.distance_status;
                    dp.next_hop = next_hop;
                    dp.hop_count = 1;
                    dp.path[0] = node_addr;
                    dp.path_length = 1;
                    dp.ttl = 4;
                    // 创建目标地址
                    linkaddr_t addr;
                    addr.u8[1] = next_hop & 0xff;   // low 8bit
                    addr.u8[0] = (next_hop >> 8) & 0xff;    //high 8bit
                    //NODE#1:0x0001,NODE#30:0x001E
                    printf("Node %d sending to addr %02x:%02x, my addr: %02x:%02x\n", 
                        node_addr, 
                        addr.u8[0], addr.u8[1],
                        linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1]);

                    packetbuf_copyfrom(&dp, sizeof(data_packet_t));
                    unicast_send(&unicast, &addr);  // 使用单播替代广播

                    int temp_whole = dp.temperature / 1000;
                    int temp_decimal = (dp.temperature % 1000) / 100;
                    if (temp_whole < 0) temp_whole = -temp_whole;
                    if (temp_decimal < 0) temp_decimal = -temp_decimal;

                    printf("\n╔═══════════════ SENT REGULAR DATA ═══════════════╗\n");
                    printf("║ Temperature: %d.%d°C                              ║\n", 
                           temp_whole, temp_decimal);
                    printf("║ Distance: %-5s                                   ║\n", 
                           dp.distance_status ? "Free (>=25cm)" : "Occupied (<25cm)");
                    printf("║ Next Hop: Node %d                                  ║\n", 
                           next_hop);
                    printf("╚══════════════════════════════════════════════════╝\n");
                }
                etimer_reset(&send_timer);
            }
        }
    }
    
    PROCESS_END();
}



// 路由发现进程

PROCESS_THREAD(discovery_process, ev, data) {
    static struct etimer routing_timer;
    static struct etimer network_timer;
    
    PROCESS_BEGIN();
    
    etimer_set(&routing_timer, ROUTING_INTERVAL);
    etimer_set(&network_timer, CLOCK_SECOND * 90); // 90秒重启一次网络连接

    while(1) {
        PROCESS_WAIT_EVENT();
        
        if(etimer_expired(&routing_timer)) {
            watchdog_periodic();
            // 检查节点活跃状态
            clock_time_t current_time = clock_time();
            for(uint8_t i = 0; i < link_count; i++) {
                if(current_time - node_links[i].last_seen > ROUTING_INTERVAL * 2) {
                    node_links[i].is_active = 0;
                }
            }
            
            // 所有节点都广播基本的路由发现信息
            broadcast_route_info();
            
            // 打印路由表
            print_routing_table();
            
            // 只有网关节点执行Dijkstra算法并广播路由更新
            if(node_type == NODE_TYPE_GATEWAY) {
                watchdog_periodic();
                build_adjacency_matrix();
                run_dijkstra(node_addr);
                broadcast_route_updates();
            }
            
            etimer_reset(&routing_timer);
        }
      if(etimer_expired(&network_timer)) {
           if(node_type == NODE_TYPE_GATEWAY) {
               printf("Periodic network reinitialization\n");
               broadcast_close(&broadcast);
               unicast_close(&unicast);
               clock_wait(CLOCK_SECOND/2);
               broadcast_open(&broadcast, 129, &broadcast_call);
               unicast_open(&unicast, 130, &unicast_call);
           }
           etimer_reset(&network_timer);
       }
    }
    
    PROCESS_END();
}

PROCESS_THREAD(init_process, ev, data) {
    PROCESS_BEGIN();

    printf("Node %d initialized with addr %02x:%02x\n", 
       node_addr,
       linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1]);

    // 初始化节点类型
    init_node_type();
    printf("Node %d initialized with addr %02x:%02x\n", 
       node_addr,
       linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1]);
    // 初始化紧急消息序列号记录
    memset(received_emergency_msgs, 0, sizeof(received_emergency_msgs));
    
    // 打印启动信息
    printf("\n╔═══════════════════════════════════════╗\n");
    switch(node_type) {
        case NODE_TYPE_GATEWAY:
            printf("║         Gateway Node %d Started          ║\n", node_addr);
            break;
        case NODE_TYPE_RELAY:
            printf("║          Relay Node %d Started          ║\n", node_addr);
            break;
        case NODE_TYPE_SENSOR:
            printf("║         Sensor Node %d Started          ║\n", node_addr);
            break;
    }
    printf("╚═══════════════════════════════════════╝\n");
    
    // 初始化无线通信
    NETSTACK_CONF_RADIO.set_value(RADIO_PARAM_CHANNEL, 11);
    NETSTACK_CONF_RDC.init();
    broadcast_open(&broadcast, 129, &broadcast_call);
    unicast_open(&unicast, 130, &unicast_call);  // 添加单播初始化
    // 启动所有进程
    process_start(&discovery_process, NULL);
    if(node_type == NODE_TYPE_SENSOR) {
        process_start(&sensor_reading_process, NULL);
        process_start(&data_sending_process, NULL);
        process_start(&emergency_process, NULL);
    }
    
    PROCESS_END();
}