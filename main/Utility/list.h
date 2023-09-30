#ifndef UTILITY_LIST_H
#define UTILITY_LIST_H

#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include "esp_log.h"

#define TAG_LIST "List"

// Define a struct for a node in the linked list
typedef struct Node {
    uint8_t index;
    void *data;
    struct Node* next;
} Node;

void initializeList(Node** head);

void addToList(Node** head, uint8_t value, void* data);

uint8_t isInList(Node* head, uint8_t value, void** data);

uint32_t getListCount(Node* head);



#endif // UTILITY_LIST_H
