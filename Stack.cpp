#include <iostream>

class Stack {
public:
    int top = -1;
    int len;
    char *arr;

    Stack(int len) {
        this->len = len;
        this->arr = new char[len];
    }

    void push(char data) {
        if (!full()) {
            top++;
            arr[top] = data;
        }
        else {
            std::cout << "Stack is already full";
        }
    }

    int pop() {
        if (!empty()) {
            top--;
            return 0;
        }
        else {
            //std::cout << "Stack is empty";
            return -1;
        }
    }

    //additional func
    bool full() {
        return top >= len - 1 ? true : false;
    }

    bool empty() {
        return top <= -1 ? true : false;
    }
};

bool check(std::string word) {
    int len = word.length();

    Stack s1(len);

    for (int i = 0; i < len; i++) {
        if (word[i] == '(') {
            s1.push('(');
        }
        else if (word[i] == ')') {
            if (s1.pop() == -1) {
                return false;
            }
        }
        else {
            continue;
        }
    }

    if (s1.empty()) {
        return true;
    }

    return false;
}

int main() {

    std::string word;
    std::cin >> word;

    //word = "(5+25)*(55*2)";

    if (check(word)) {
        std::cout << "Pass";
    }
    else {
        std::cout << "Error";
    }

    return 0;
}