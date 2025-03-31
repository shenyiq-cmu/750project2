#ifndef TYPES_H
#define TYPES_H

/* Class definitions */
typedef enum {
    CLASS_1 = 0,                 // Class 1: 3-second period
    CLASS_2 = 1,                 // Class 2: 5-second period
    CLASS_3 = 2,                 // Class 3: 6-second period
    MAX_CLASSES
} class_id_t;

/* Data type definitions */
typedef enum {
    DATA_TYPE_INT8 = 0,          // 8-bit integer
    DATA_TYPE_INT16 = 1,         // 16-bit integer
    DATA_TYPE_INT32 = 2,         // 32-bit integer
    DATA_TYPE_FLOAT = 3,         // 32-bit float
    DATA_TYPE_DOUBLE = 4,        // 64-bit double
    NUM_DATA_TYPE
} data_type_t;

/* Packet type definition */
typedef enum{
    PACKET_TYPE_CONTROL = 0,
    PACKET_TYPE_DATA = 1
} packet_type_t;


#endif