
#include "list.h"

// Function to initialize the list (create an empty list)
void initializeList(Node** head) {
    *head = NULL;
}

// Function to add a new uint8_t to the list
void addToList(Node** head, uint8_t value, void* data) {
    // Create a new node
    Node* newNode = (Node*)malloc(sizeof(Node));
    if (newNode == NULL) {
        ESP_LOGE(TAG_LIST, "Memory allocation failed.");
        return;
    }
    newNode->index = value;
    newNode->data = data;
    newNode->next = NULL;

    // If the list is empty, set the new node as the head
    if (*head == NULL) {
        *head = newNode;
    } else {
        // Otherwise, find the last node and append the new node
        Node* current = *head;
        while (current->next != NULL) {
            current = current->next;
        }
        current->next = newNode;
    }
}

// Function to check if a uint8_t is in the list
uint8_t isInList(Node* head, uint8_t value, void** data) {
    Node* current = head;
    while (current != NULL) {
        if (current->index == value) {
            *data = current->data; // Provide the custom data to the caller

            return 1; // Value found in the list
        }
        current = current->next;
    }
    return 0; // Value not found in the list
}

// Function to return the number of entries in the list
uint32_t getListCount(Node* head) {
    uint32_t count = 0;
    Node* current = head;
    while (current != NULL) {
        count++;
        current = current->next;
    }
    return count;
}