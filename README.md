# Wireless Sensor Network Project

## Overview
This project implements a wireless sensor network (WSN) consisting of 7 nodes: 4 edge sensor nodes, 2 intermediate relay nodes, and 1 gateway node. The system is designed to collect environmental data and efficiently transmit it to a central gateway for visualization and analysis.

## Hardware and Framework
- **Hardware Platform**: Zolertia™ Re-Mote
- **Operating System**: Contiki OS
- **Communication Stack**: Rime

### Network Architecture
- 4 Edge Nodes (Node IDs: 11-14)
  - Responsible for data collection
  - Temperature and distance sensing capabilities
  - Automatic emergency alerts for high temperatures
  
- 2 Relay Nodes (Node IDs: 2-3)
  - Data forwarding and routing
  - Dynamic route maintenance
  - Support for multi-hop communication
  
- 1 Gateway Node (Node ID: 1)
  - Direct PC connection
  - Data collection and visualization
  - Network management and routing optimization

## Key Features
- **Multi-hop Communication**: Supports data transmission across multiple nodes
- **Dynamic Routing**: Uses Dijkstra's algorithm for optimal path finding
- **Automatic Route Discovery**: Periodic network topology updates
- **Emergency Alerts**: Automatic high-temperature event detection and broadcasting
- **Distance Detection**: Occupancy status monitoring based on distance measurements
- **Energy Efficiency**: Optimized radio duty cycling

## Technical Details
- **Communication Protocol**: Uses Contiki's Rime stack
- **Routing Algorithm**: Modified Dijkstra's algorithm with RSSI-based cost metrics
- **Data Types**:
  - Regular sensor data (temperature, distance)
  - Emergency alerts
  - Routing information
  - Network status updates

## Network Protocol
1. **Route Discovery**
   - Periodic topology updates
   - RSSI-based link quality assessment
   - Dynamic route table maintenance

2. **Data Transmission**
   - Unicast for regular data
   - Broadcast for emergency alerts
   - TTL-based packet control

3. **Network Management**
   - Automatic node type detection
   - Link quality monitoring
   - Route optimization

## Compilation and Setup Instructions

### Prerequisites
- Contiki OS
- Zoul compiler toolchain
- GNU Make
- Zolertia™ Re-Mote hardware

### Node ID Configuration
Node IDs need to be specified in hexadecimal format:
- Gateway Node: 0x01
- Relay Nodes: 0x02 or 0x03
- Sensor Nodes: 0x0B to 0x0E (11-14 in decimal)

### Building and Flashing
1. Compile and upload to device (replace XX with hex node ID):
```bash
make TARGET=zoul BOARD=remote-revb NODEID=0xXX wsn_node_v5.upload
```

Example node ID configurations:
```bash
# For Gateway (Node 1)
make TARGET=zoul BOARD=remote-revb NODEID=0x01 wsn_node_v5.upload

# For Relay Node (Node 2)
make TARGET=zoul BOARD=remote-revb NODEID=0x02 wsn_node_v5.upload

# For Sensor Node (Node 14)
make TARGET=zoul BOARD=remote-revb NODEID=0x0E wsn_node_v5.upload
```

2. Monitor serial output:
```bash
make TARGET=zoul BOARD=remote-revb PORT=/dev/ttyUSB0 login
```

Note: Adjust the PORT parameter according to your system's USB port assignment.

---

# 无线传感器网络项目

## 概述
本项目实现了一个由7个节点组成的无线传感器网络(WSN)：4个边缘传感器节点，2个中继节点和1个网关节点。该系统旨在收集环境数据并高效地将其传输到中央网关进行可视化和分析。

## 硬件与框架
- **硬件平台**：Zolertia™ Re-Mote
- **操作系统**：Contiki OS
- **通信协议栈**：Rime

### 网络架构
- 4个边缘节点（节点ID：11-14）
  - 负责数据采集
  - 具备温度和距离感测能力
  - 自动高温紧急警报
  
- 2个中继节点（节点ID：2-3）
  - 数据转发和路由
  - 动态路由维护
  - 支持多跳通信
  
- 1个网关节点（节点ID：1）
  - 直接连接PC
  - 数据收集和可视化
  - 网络管理和路由优化

## 主要特点
- **多跳通信**：支持跨多个节点的数据传输
- **动态路由**：使用Dijkstra算法进行最优路径查找
- **自动路由发现**：周期性网络拓扑更新
- **紧急警报**：自动高温事件检测和广播
- **距离检测**：基于距离测量的占用状态监控
- **能源效率**：优化的无线电占空比

## 技术细节
- **通信协议**：使用Contiki的Rime协议栈
- **路由算法**：基于RSSI代价度量的改进Dijkstra算法
- **数据类型**：
  - 常规传感器数据（温度、距离）
  - 紧急警报
  - 路由信息
  - 网络状态更新

## 网络协议
1. **路由发现**
   - 周期性拓扑更新
   - 基于RSSI的链路质量评估
   - 动态路由表维护

2. **数据传输**
   - 常规数据使用单播
   - 紧急警报使用广播
   - 基于TTL的数据包控制

3. **网络管理**
   - 自动节点类型检测
   - 链路质量监控
   - 路由优化

## 编译和设置说明

### 前置要求
- Contiki操作系统
- Zoul编译工具链
- GNU Make
- Zolertia™ Re-Mote硬件设备

### 节点ID配置
节点ID需要使用十六进制格式指定：
- 网关节点：0x01
- 中继节点：0x02 或 0x03
- 传感器节点：0x0B 至 0x0E（十进制的11-14）

### 编译和烧录
1. 编译并上传到设备（将XX替换为十六进制节点ID）：
```bash
make TARGET=zoul BOARD=remote-revb NODEID=0xXX wsn_node_v5.upload
```

节点ID配置示例：
```bash
# 网关节点（节点1）
make TARGET=zoul BOARD=remote-revb NODEID=0x01 wsn_node_v5.upload

# 中继节点（节点2）
make TARGET=zoul BOARD=remote-revb NODEID=0x02 wsn_node_v5.upload

# 传感器节点（节点14）
make TARGET=zoul BOARD=remote-revb NODEID=0x0E wsn_node_v5.upload
```

2. 监控串口输出：
```bash
make TARGET=zoul BOARD=remote-revb PORT=/dev/ttyUSB0 login
```

注意：根据您系统的USB端口分配情况调整PORT参数。
