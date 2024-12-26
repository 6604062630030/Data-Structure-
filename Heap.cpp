#include <iostream>
#include <vector>

using namespace std;

class heap {
private:
  vector<int> h;

  int parent(int i) {
    return (i - 1) / 2;
  }

  int left(int i) {
    return 2 * i + 1;
  }

  int right(int i) {
    return 2 * i + 2;
  }

  void heapUp(int i) {
    int p = parent(i);

    if (i > 0 && h[p] < h[i]) {
      swap(h[p], h[i]);
      heapUp(p);
    }
  }

  void heapDown(int i) {
    int p = i;
    int leftc = left(i);
    int rightc = right(i);

    if (leftc < h.size() && h[leftc] > h[p]) {
      p = leftc;
    }

    if (rightc < h.size() && h[rightc] > h[p]) {
      p = rightc;
    }

    if (p != i) {
      swap(h[p], h[i]);
      heapDown(p);
    }
  }

public:
  void add(int data) {
    h.push_back(data);
    heapUp(h.size() - 1);
  }

  int del() {
    int temp = h[0];
    h[0] = h.back();
    h.pop_back();

    heapDown(0);
    return temp;
  }

  void display() {
    for (int i = 0; i < h.size(); i++) {
      cout << h[i] << " ";
    }
    cout << endl;
  }
};

int main() {
  heap maxHeap;
  char command;
  int value;

  while (true) {
    cin >> command;

    if (command == 'a') {      // Add data to heap
      cin >> value;
      maxHeap.add(value);
    }
    else if (command == 'p') {  // Display heap
      maxHeap.display();
    }
    else if (command == 'd') {  // Remove and display max value
      cout << maxHeap.del() << endl;
    }
    else if (command == 'e') {  // Exit the loop
      break;
    }
  }

  return 0;
}
