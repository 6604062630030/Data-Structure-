#include <iostream>
#include <queue>

using namespace std;

class Node {
public:
  int data;
  Node *left;
  Node *right;

  Node(int data) {
    this->data = data;
    this->left = nullptr;
    this->right = nullptr;
  }
};

class bst { //Duplicates are not allowed ver.
private:
  Node *root = nullptr;

  Node *Insert(Node *root, int data) {
    if (root == nullptr) return new Node(data);

    if (data < root->data) {
      root->left = Insert(root->left, data);
    }
    else if (data > root->data) { //else
      root->right = Insert(root->right, data);
    }
    else {//if data == root.data 
      cout << "data already added\n";
    }

    return balance(root); //after everything finish, update original root that has been balanced
  }

  Node *Delete(Node *root, int data) {
    if (root == nullptr) return root;

    else if (data < root->data) {
      root->left = Delete(root->left, data);
    }
    else if (data > root->data) {
      root->right = Delete(root->right, data);
    }
    else {
      if (root->left == nullptr && root->right == nullptr) {
        delete root;
        root = nullptr;
      }
      else if (root->left == nullptr) {
        Node *temp = root;
        root = root->right;
        delete temp;
      }
      else if (root->right == nullptr) {
        Node *temp = root;
        root = root->left;
        delete temp;
      }
      else {
        Node *min = findMin(root->right);
        root->data = min->data;
        root->right = Delete(root->right, min->data);
      }
    }

    return balance(root); //after everything finish, update original root that has been balanced
  }

  Node *findMin(Node *target) {//make sure return leaf correctly
    return target->left == nullptr ? target : findMin(target->left);
  }

  void preorder(Node *root) {
    if (root == nullptr) return;

    cout << root->data << " ";
    preorder(root->left);
    preorder(root->right);
  }

  void inorder(Node *root) {
    if (root == nullptr) return;

    inorder(root->left);
    cout << root->data << " ";
    inorder(root->right);
  }

  void postorder(Node *root) {
    if (root == nullptr) return;

    postorder(root->left);
    postorder(root->right);
    cout << root->data << " ";
  }

  //AVL ----------------------------------------------------
  int height(Node *root) {
    if (root == nullptr) return -1;
    return 1 + max(height(root->left), height(root->right));
  }

  int getBalanceFactor(Node *root) {
    if (root == nullptr) return 0;
    return height(root->left) - height(root->right);
  }

  Node *rotateLeft(Node *root) {
    Node *newRoot = root->right;
    Node *leftSubtreeOfNewRoot = newRoot->left;

    newRoot->left = root;
    root->right = leftSubtreeOfNewRoot;

    return newRoot;
  }
  Node *rotateRight(Node *root) {
    Node *newRoot = root->left;
    Node *rightSubtreeOfNewRoot = newRoot->right;

    newRoot->right = root;
    root->left = rightSubtreeOfNewRoot;

    return newRoot;
  }

  Node *balance(Node *root) {
    if (root == nullptr) return root;

    int balance = getBalanceFactor(root);

    // Left Left Case
    if (balance > 1 && getBalanceFactor(root->left) >= 0) {
      return rotateRight(root);
    }

    // Right Right Case
    if (balance < -1 && getBalanceFactor(root->right) <= 0) {
      return rotateLeft(root);
    }

    // Left Right Case
    if (balance > 1 && getBalanceFactor(root->left) < 0) {
      root->left = rotateLeft(root->left);
      return rotateRight(root);
    }

    // Right Left Case
    if (balance < -1 && getBalanceFactor(root->right) > 0) {
      root->right = rotateRight(root->right);
      return rotateLeft(root);
    }

    return root; //update the root
  }
  //AVL ----------------------------------------------------

public:
  void add(int data) {
    root = Insert(root, data);
  }

  void del(int data) {
    root = Delete(root, data);
  }

  void Preorder() {
    cout << "Preorder: ";
    preorder(root);
    cout << endl;
  }

  void Inorder() {
    cout << "Inorder: ";
    inorder(root);
    cout << endl;
  }

  void Postorder() {
    cout << "Postorder: ";
    postorder(root);
    cout << endl;
  }

  void Breadth() {
    cout << "Breath: ";
    if (root == nullptr) return;

    queue<Node *> Q;
    Q.push(root);

    while (!Q.empty()) {

      int size = Q.size();
      for (int i = 0; i < size; i++) {
        Node *temp = Q.front();
        cout << temp->data << ",";
        Q.pop();
        if (temp->left != nullptr) Q.push(temp->left);
        if (temp->right != nullptr) Q.push(temp->right);
      }
      cout << "|";
    }
    cout << endl;
  }

};

int main() {

  bst b;
  char command;
  int data;

  while (true) {
    cin >> command;
    if (command == 'a') {
      cin >> data;
      b.add(data);
    }
    else if (command == 'd') {
      cin >> data;
      b.del(data);
    }
    else if (command == 'p') {
      b.Preorder();
    }
    else if (command == 'i') {
      b.Inorder();
    }
    else if (command == 't') {
      b.Postorder();
    }
    else if (command == 'b') {
      b.Breadth();
    }
    else if (command == 'x') {
      break;
    }
  }

  return 0;
}