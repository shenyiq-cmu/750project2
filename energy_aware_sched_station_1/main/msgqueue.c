#include "msgqueue.h"


/* Queue functions */
void queue_init(packet_queue_t *queue) {
    queue->head = NULL;
    queue->tail = NULL;
    queue->count = 0;
}

/* Add packet to the end of queue */
bool queue_enqueue(packet_queue_t *queue, queue_packet_t *packet) {
    if (queue->count >= MAX_QUEUE_SIZE) {
        return false;  // Queue is full
    }
    
    queue_node_t *new_node = (queue_node_t*)malloc(sizeof(queue_node_t));
    if (!new_node) {
        return false;  // Memory allocation failed
    }
    
    // Copy packet data
    memcpy(&new_node->packet, packet, sizeof(queue_packet_t));
    new_node->next = NULL;
    
    // Add to queue
    if (queue->count == 0) {
        // First packet
        queue->head = new_node;
        queue->tail = new_node;
    } else {
        // Add to end
        queue->tail->next = new_node;
        queue->tail = new_node;
    }
    
    queue->count++;
    return true;
}

/* Add packet to the front of queue */
bool queue_enqueue_front(packet_queue_t *queue, queue_packet_t *packet) {
    if (queue->count >= MAX_QUEUE_SIZE) {
        return false;  // Queue is full
    }
    
    queue_node_t *new_node = (queue_node_t*)malloc(sizeof(queue_node_t));
    if (!new_node) {
        return false;  // Memory allocation failed
    }
    
    // Copy packet data
    memcpy(&new_node->packet, packet, sizeof(queue_packet_t));
    
    // Add to front of queue
    if (queue->count == 0) {
        // First packet
        new_node->next = NULL;
        queue->head = new_node;
        queue->tail = new_node;
    } else {
        // Add to front
        new_node->next = queue->head;
        queue->head = new_node;
    }
    
    queue->count++;
    return true;
}

/* Remove and return packet from the front of queue */
bool queue_dequeue(packet_queue_t *queue, queue_packet_t *packet) {
    if (queue->count == 0) {
        return false;  // Queue is empty
    }
    
    queue_node_t *node = queue->head;
    
    // Copy packet data
    memcpy(packet, &node->packet, sizeof(queue_packet_t));
    
    // Update queue
    queue->head = node->next;
    queue->count--;
    
    if (queue->count == 0) {
        queue->tail = NULL;
    }
    
    // Free node
    free(node);
    return true;
}

/* Peek at the front packet without removing it */
bool queue_peek(packet_queue_t *queue, queue_packet_t *packet) {
    if (queue->count == 0) {
        return false;  // Queue is empty
    }
    
    // Copy packet data
    memcpy(packet, &queue->head->packet, sizeof(queue_packet_t));
    return true;
}