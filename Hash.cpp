#include <iostream>

using namespace std;

class Node {
public:
  int key;
  string value;
  Node *next;

  Node(int key, std::string value) {
    this->key = key;
    this->value = value;
    this->next = nullptr;
  }
};

class Hash {
public:
  Node **h; //point to an array that point to Node
  int size;

  Hash(int size) {
    this->h = new Node * [size];
    this->size = size;

    for (int i = 0; i < size; i++) {
      h[i] = nullptr;
    }
  }

  int hashFunc(int key) {
    return key % size;
  }

  void add(int key, string value) {
    int index = hashFunc(key);
    Node *temp = h[index];

    if (temp != nullptr) {
      while (temp->next != nullptr) {
        temp = temp->next;
      }
      temp->next = new Node(key, value);
    }
    else {
      h[index] = new Node(key, value); //temp == nullptr
    }
  }

  void print() {
    for (int i = 0; i < size; i++) {
      Node *temp = h[i];
      cout << i << " : ";
      while (temp != nullptr) {
        cout << temp->value << " ";
        temp = temp->next;
      }
      cout << endl;
    }
  }
};

int main() {

  Hash *h = new Hash(5);
  char in;
  int key;
  string val;

  do {
    cin >> in;
    switch (in) {
    case 'a':
      cin >> key;
      cin >> val;
      h->add(key, val);
      break;
    case 'p':
      h->print();
      break;
    }
  } while (in != 'x');

  return 0;
}