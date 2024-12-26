#include <iostream>

class node {
public:
    int data;
    node *next;
    node *prev;

    node(int data) {
        this->data = data;
        this->next = nullptr;
        this->prev = nullptr;
    }
};

class doubly_linkedlist {
public:
    node *head = nullptr;
    node *tail = nullptr;
    node *temp;
    int size = 0;

    void push_front(int data) {

        node *newNode = new node(data);

        if (!head) {
            head = newNode;
            tail = head;
        }
        else {
            head->prev = newNode;
            newNode->next = head;
            head = newNode;
        }

        size++;
    }

    void push_back(int data) {

        node *newNode = new node(data);

        if (!head) {
            head = newNode;
            tail = head;
        }
        else {
            newNode->prev = tail;
            tail->next = newNode;
            tail = newNode; // tail = tail->next;
        }
        size++;
    }

    int pop_front() {

        if (head == nullptr) {
            return -1;
        }
        temp = head;
        head = head->next;
        head->prev = nullptr;

        size--;
        return temp->data;
    }

    int pop_back() {

        if (head == nullptr) {
            return -2;
        }
        temp = tail;
        tail = tail->prev;
        tail->next = nullptr; //most likely forgotten
        size--;
        return temp->data;
    }

    void print() {
        for (temp = head; temp != nullptr; temp = temp->next) {
            std::cout << temp->data << ' ';
        }
        std::cout << '\n';
    }
};

int main() {

    doubly_linkedlist *d1 = new doubly_linkedlist();

    std::cout << d1->pop_front() << '\n';
    d1->push_front(10);
    d1->push_front(20);
    d1->push_front(30);
    d1->push_front(40);
    d1->push_back(50);
    d1->push_back(60);

    d1->print();

    std::cout << d1->pop_front() << '\n';
    std::cout << d1->pop_back() << '\n';
    std::cout << d1->pop_back() << '\n';

    d1->print();

    return 0;
}

/*
bool insertAt(int id, string name) { //by this insert mean insert_before
    node *newNode = new node(id, name);
    node *at = find(id);

    if (!unique(id)) return false;

    if (!head) { //if no node
      head = newNode;
      tail = head;
    }
    else if (at == nullptr || at == tail) { //if not found or found at tail
      tail->next = newNode;
      newNode->prev = tail;
      tail = newNode;
    }
    else if (at == head) { // found at head
      head->prev = newNode;
      newNode->next = head;
      head = newNode;
    }
    else { //insert before between
      at->prev->next = newNode;
      newNode->prev = at->prev;
      at->prev = newNode;
      newNode->next = at;
    }
    return true;
  }

  string popAt(int id) {
    temp = find(id);

    if (temp == nullptr) {
      return "not found";
    }
    else if (temp == tail) {
      tail = tail->prev;
      tail->next = nullptr;
    }
    else if (temp == head) {
      head = head->next;
      head->prev = nullptr;
    }
    else {
      temp->prev->next = temp->next;
      temp->next->prev = temp->prev;
    }

    return temp->name;
  }
*/