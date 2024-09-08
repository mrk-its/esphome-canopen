# Object Dictionary Layout

Each ESPHome entity exposed via CANOpen is mapped to CANOpen Object Dictionary (OD), the mapping is defined in `canopen.entities` list in ESPHome config (yaml):
```
canopen:
  entities:
    - id: sensor1
      index: 2
    - id: switch2
      index: 4
```
OD Indices occupied by particular esphome entity are computed based on its `index` field in above entity mapping:

 - Index = 0x2000 + `index` * 0x10 - Metadata of entity (details in the table below)
 - Index = 0x2001 + `index` * 0x10 - Array of states of entity (read only), number of states / their OD types depends on entity type.
 - Index = 0x2002 + `index` * 0x10 - Array of commands for entity (write only), number of commands / their OD types depend on entity type.


| Index             | SubIndex | Object Name             | Type   | Access | Description     |
|-------------------|----------|-------------------------|--------|:------:|-|
| 0x2000            |          | Entity Types            | UINT8[]| R      | Array of EntityTypeCodes of mapped entities |
| 0x2010            |          | Entity 1                |        |        | |
|                   | 0x01     | Name                    | String | R      | |
|                   | 0x02     | Device Class            | String | R      | |
|                   | 0x03     | Unit                    | String | R      | |
|                   | 0x04     | State Class             | String | R      | |
|||||||
| 0x2011            |          | Entity 1 States         |        |        | Number of states / their types depends on entity type |
|                   | 0x01     | State #1                |        | R      | | 
|                   | 0x02     | State #2                |        | R      | |
|                   | K        | State #K                |        | R      | |
|||||||
| 0x2012            |          | Entity 1 Commands       |        |        | Number of commands / their types depends on entity type  |
|                   | 0x01     | Command #1              |        | W      | | 
|                   | 0x02     | Command #2              |        | W      | |
|                   | K        | Command #K              |        | W      | |
|||||||
| 0x2000 + 0x10 * N |          | Entity #N               | String |        | |
|                   | 0x01     | Name                    | String | R      | |
|                   | 0x02     | Device Class            | String | R      | |
|                   | 0x03     | Unit                    | String | R      | |
|                   | 0x04     | State Class             | String | R      | |
|||||||
| 0x2001 + 0x10 * N |          | Entity #N States        |        |        | |
|                   | 0x01     | State #1                |        | R      | |
|                   | 0x02     | State #2                |        | R      | |
|                   | K        | State #K                |        | R      | |
|||||||
| 0x2002 + 0x10 * N |          | Entity #N Commands      |        |        | |
|                   | 0x01     | Command #1              |        | W      | |
|                   | 0x02     | Command #2              |        | W      | |
|                   | K        | Command #K              |        | W      | |



## Sensor
EntityTypeCode: 1

| Index             | SubIndex | Object Name             | Type   | Access | Description     |
|-------------------|----------|-------------------------|--------|:------:|-|
| 0x2001 + 0x10 * N | 0x01     | Sensor #N State         | REAL32 | R      | |


## Sensor8
EntityTypeCode: 6

| Index             | SubIndex | Object Name             | Type   | Access | Description     |
|-------------------|----------|-------------------------|--------|:------:|-|
| 0x2000 + 0x10 * N | 0x07     | Sensor #N min_value     | REAL32 | R      | |
|                   | 0x08     | Sensor #N max_value     | REAL32 | R      | |
|||||||
| 0x2001 + 0x10 * N | 0x01     | Sensor #N State         | UINT8  | R      | value = state * (max_value - min_value + 1) / 256 |

## Sensor16
EntityTypeCode: 7

| Index             | SubIndex | Object Name             | Type   | Access | Description     |
|-------------------|----------|-------------------------|--------|:------:|-|
| 0x2000 + 0x10 * N | 0x07     | Sensor #N min_value     | REAL32 | R      | |
|                   | 0x08     | Sensor #N max_value     | REAL32 | R      | |
|||||||
| 0x2001 + 0x10 * N | 0x01     | Sensor #N State         | UINT16 | R      | value = state * (max_value - min_value + 1) / 65536 |


## Binary Sensor
EntityTypeCode: 2

| Index             | SubIndex | Object Name             | Type   | Access | Description     |
|-------------------|----------|-------------------------|--------|:------:|-----------------|
| 0x2001 + 0x10 * N | 0x01     | Binary Sensor #N State  | UINT8  | R      | 0 - off, 1 - on |


## Switch
EntityTypeCode: 3

| Index             | SubIndex | Object Name             | Type   | Access | Description     |
|-------------------|----------|-------------------------|--------|:------:|-----------------|
| 0x2001 + 0x10 * N | 0x01     | Switch #N State         | UINT8  | R      | 0 - off, 1 - on |
|||||||
| 0x2002 + 0x10 * N | 0x01     | Switch #N Command       | UINT8  | W      | 0 - off, 1 - on |

## Cover
EntityTypeCode: 4

| Index             | SubIndex | Object Name             | Type   | Access | Description     |
|-------------------|----------|-------------------------|--------|:------:|-|
| 0x2001 + 0x10 * N | 0x01     | Cover #N State          | UINT8  | R      | |
| 0x2001 + 0x10 * N | 0x02     | Cover #N Position       | UINT8  | R      | |
|||||||
| 0x2002 + 0x10 * N | 0x01     | Cover #N Command        | UINT8  | W      | |
| 0x2002 + 0x10 * N | 0x02     | Cover #N SetPosition    | UINT8  | W      | |

## Light

EntityTypeCode: 5

| Index             | SubIndex | Object Name             | Type   | Access | Description     |
|-------------------|----------|-------------------------|--------|:------:|-----------------|
| 0x2001 + 0x10 * N | 0x01     | Light #N State          | UINT8  | R      | 0 - off, 1 - on |
| 0x2001 + 0x10 * N | 0x02     | Light #N Brightness     | UINT8  | R      | 0..255          |
| 0x2001 + 0x10 * N | 0x03     | Light #N ColorTemp      | UINT16 | R      | 0..65535        |
|||||||
| 0x2002 + 0x10 * N | 0x01     | Light #N Command        | UINT8  | W      | 0 - off, 1 - on |
| 0x2002 + 0x10 * N | 0x02     | Light #N SetBrightness  | UINT8  | W      | 0..255          |
| 0x2002 + 0x10 * N | 0x03     | Light #N SetColorTemp   | UINT16 | W      | 0..65535        |

## Number
EntityTypeCode: 8

| Index             | SubIndex | Object Name             | Type   | Access | Description     |
|-------------------|----------|-------------------------|--------|:------:|-|
| 0x2001 + 0x10 * N | 0x01     | Sensor #N State         | REAL32 | R      | |

## Number8
EntityTypeCode: 9

| Index             | SubIndex | Object Name             | Type   | Access | Description     |
|-------------------|----------|-------------------------|--------|:------:|-|
| 0x2000 + 0x10 * N | 0x07     | Number #N min_value     | REAL32 | R      | |
|                   | 0x08     | Number #N max_value     | REAL32 | R      | |
|||||||
| 0x2001 + 0x10 * N | 0x01     | Number #N State         | UINT8  | R      | value = state * (max_value - min_value + 1) / 256 |

## Number16
EntityTypeCode: 10

| Index             | SubIndex | Object Name             | Type   | Access | Description     |
|-------------------|----------|-------------------------|--------|:------:|-|
| 0x2000 + 0x10 * N | 0x07     | Number #N min_value     | REAL32 | R      | |
|                   | 0x08     | Number #N max_value     | REAL32 | R      | |
|||||||
| 0x2001 + 0x10 * N | 0x01     | Number #N State         | UINT16 | R      | value = state * (max_value - min_value + 1) / 65536 |

