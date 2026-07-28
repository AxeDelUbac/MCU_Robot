┌─────────────────────────────────────────────────────────────────┐
│                     STRUCTURE RobotDataPacket_t                │
├─────────────────────────┬───────────────────┬───────────────────┤
│        CHAMP            │      TAILLE       │     POSITION      │
├─────────────────────────┼───────────────────┼───────────────────┤
│ 1. startByte            │     1 octet       │    Offset 0       │
│ 2. dataLength           │     1 octets      │    Offset 1-2     │
│ 3. packetType           │     1 octet       │    Offset 3       │
│ 4. data (DataPacket_t)  │   voir détail ↓   │    Offset 4       │
│ 6. checksum             │     1 octet       │    Offset X       │
└─────────────────────────┴───────────────────┴───────────────────┘

┌─────────────────────────────────────────────────────────────────┐
│                      DataPacket_t (dans data)                  │
├─────────────────────────┬───────────────────┬───────────────────┤
│ motorData               │   8×4 = 32 octets │ 4 floats × 2      │
│  - wheelSpeedRPM[4]     │      16 octets    │ 4 floats          │
│  - motorPower[4]        │      16 octets    │ 4 floats          │
├─────────────────────────┼───────────────────┼───────────────────┤
│ odometryData            │   6×4 = 24 octets │ 6 floats          │
│  - positionX            │       4 octets    │ 1 float           │
│  - positionY            │       4 octets    │ 1 float           │
│  - orientation          │       4 octets    │ 1 float           │
│  - velocityX            │       4 octets    │ 1 float           │
│  - velocityY            │       4 octets    │ 1 float           │
│  - angularVelocity      │       4 octets    │ 1 float           │
├─────────────────────────┼───────────────────┼───────────────────┤
│ imuData                 │   9×4 = 36 octets │ 9 floats          │
│  - accel[3]             │      12 octets    │ 3 floats          │
│  - gyro[3]              │      12 octets    │ 3 floats          │
│  - mag[3]               │      12 octets    │ 3 floats          │
│  - roll                 │       4 octets    │ 1 float           │
│  - pitch                │       4 octets    │ 1 float           │
│  - yaw                  │       4 octets    │ 1 float           │
├─────────────────────────┼───────────────────┼───────────────────┤
│ TOTAL DataPacket_t      │      92 octets    │                   │
└─────────────────────────┴───────────────────┴───────────────────┘


# Data Reçu correspondant à
AA 03 01 80 80 06 0A

AA start byte
03 useful datalength
01 datatype
80 80 06 Direction x and Y, gearbox ratio
0A Checksum