#ifndef MSGQUEUE_H
#define MSGQUEUE_H

#include "terminal_cmd.h"
#include "freertos/FreeRTOS.h"
#include <string.h>
#include "freertos/task.h"

#define MAX_PACKET_SIZE          1400  // Maximum packet data size
#define MAX_QUEUE_SIZE           50    // Maximum packets per queue

/* Internal queue packet structure */
typedef struct {
    class_id_t class_id;          // Class identifier (0, 1, 2)
    uint32_t deadline;            // Absolute deadline for this packet (in ms)
    data_type_t data_type;        // Type of data contained
    uint16_t data_count;          // Number of data elements
    uint16_t size;                // Actual data size in bytes (not include header)
    uint8_t data[MAX_PACKET_SIZE]; // Packet data
} queue_packet_t;

/* Node structure for the linked list queue */
typedef struct queue_node {
    queue_packet_t packet;
    struct queue_node *next;
} queue_node_t;

/* Queue structure */
typedef struct {
    queue_node_t *head;
    queue_node_t *tail;
    int count;
} packet_queue_t;


void queue_init(packet_queue_t *queue);
bool queue_enqueue(packet_queue_t *queue, queue_packet_t *packet);
bool queue_enqueue_front(packet_queue_t *queue, queue_packet_t *packet);
bool queue_dequeue(packet_queue_t *queue, queue_packet_t *packet);
bool queue_peek(packet_queue_t *queue, queue_packet_t *packet);

#endif