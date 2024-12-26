#include <iostream>

class Queue {
public:
    int head;
    int tail;
    int cap;
    int size;
    int *arr;

    Queue(int cap) {
        head = -1;
        tail = -1;
        this->cap = cap;
        size = 0;
        arr = new int[cap];
    }

    bool full() {
        return size == cap;
    }

    bool empty() {
        return size == 0;
    }

    void enqueue(int data) {
        if(full()) { //full case
            std::cout << "full";
            return;
        }

        if(empty()) { //first data input case
            head = 0;
            tail = 0;
        }
        else { //base case
            tail = (tail + 1 ) % cap;
        }

        size++;
        arr[tail] = data;
    }

    int dequeue() {
        if(empty()) {
            std::cout << "empty";
            return -1;
        }

        int temp = arr[head];

        if(head == tail) {
            head = -1;
            tail = -1;
        }
        else {
            head = (head + 1) % cap;
        }
        
        size--;
        return temp;
    }

    void print() {
        int i=head;

        while(true) {
            
            std::cout << arr[i] << ' ';

            if(i == tail) {
                break;
            }

            i = (i + 1) % cap;

        }
        std::cout << '\n';
    }

    void push_front(int data) { //not necessary
        if(full()) {
            std::cout << "full";
            return;
        }

        if(empty()) {
            head = 0;
            tail = 0;
        }
        else {
            head = (head - 1 + cap) % cap;
        }
        arr[head] = data;
        size++;
    }

    int pop_back () { //not necessary
        if(empty()) {
            std::cout << "empty";
            return -1;
        }
        int temp = arr[tail];
        tail = (tail - 1 + cap) % cap;
        size--;
        return temp; 
    }
};

int main() {

    Queue q1(8);

    // for(int i=0; i<5; i++) {
    //     q1.enqueue(i);
    // }

    //q1.print();
    q1.push_front(69);
    q1.push_front(420);
    q1.enqueue(1);
    q1.print();
    std::cout << q1.dequeue() << '\n';
    q1.print();
    q1.enqueue(2);
    q1.print();
    std::cout << q1.pop_back() << '\n';
    q1.print();
    return 0;
}