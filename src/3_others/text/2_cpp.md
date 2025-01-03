# C++

## Warmup Quiz

1. What will be the output of the following code?

```c++
#include <iostream>
int main() {
    int x = 5;
    int* p = &x;
    *p = 10;
    std::cout << x << '\n';
    return 0;
}
```

2. What does the `const` keyword do when applied to a variable?

3. What is the difference between `struct` and `class` in C++?

4. What is the purpose of a constructor in a class?

5. Explain the difference between the `pointer` and `reference`.

6. What will be the output of the following code?
```c++
#include <iostream>
#include <vector>
int main() {
    std::vector<int> v = {1, 2, 3};
    for (auto it = v.begin(); it != v.end(); ++it) {
        std::cout << *it << " ";
    }
    return 0;
}
```

7. In your onw words, explain what is the Standard Template Library (STL).

8. What will be the output of the following code?
```c++
#include <iostream>
class Base {
public:
    virtual void print() {
        std::cout << "Base class\n";
    }
};

class Derived : public Base {
public:
    void print() override {
        std::cout << "Derived class\n";
    }
};

int main() {
    Base* b = new Derived();
    b->print();
    delete b;
    return 0;
}
```

9. Explain difference between `std::array<T, N>` and `std::vector<T>`.

10. Explain the output of following Lambda-function based code.
```c++
#include <iostream>
int main() {
    int a = 10, b = 20;
    auto sum = [&]() -> int { return a + b; };
    b = 30;
    std::cout << sum() << '\n';
    return 0;
}
```

## Revisiting Fundamentals 

### Functions and Pointers

Functions are the building blocks of C++ programs, and pointers are fundamental for memory management. Let’s revisit these concepts with an example."

```c++
#include <iostream>
void swap(int* a, int* b) {
    int temp = *a;
    *a = *b;
    *b = temp;
}

int main() {
    int x = 5, y = 10;
    std::cout << "Before swap: x = " << x << ", y = " << y << '\n';
    swap(&x, &y);
    std::cout << "After swap: x = " << x << ", y = " << y << '\n';
    return 0;
}
```
Discussion Points:
 - What happens when you pass pointers versus values?
 - When would you use references instead of pointers?

### Object-Oriented Programming (OOP)

Object-oriented programming is key to structuring large projects in C++. Let’s review how classes and inheritance work.

```c++
#include <iostream>
#include <string>

class BankAccount {
private:
    std::string owner;
    double balance;

public:
    BankAccount(const std::string& owner, double balance) 
        : owner(owner), balance(balance) {}

    void deposit(double amount) {
        balance += amount;
    }

    void withdraw(double amount) {
        if (amount <= balance)
            balance -= amount;
        else
            std::cout << "Insufficient funds!\n";
    }

    void display() const {
        std::cout << owner << "'s balance: $" << balance << '\n';
    }
};

int main() {
    BankAccount account("John Doe", 1000.0);
    account.display();
    account.deposit(500);
    account.withdraw(300);
    account.display();
    return 0;
}
```

Discussion Points:
 - What is the purpose of the private keyword?
 - How does the const qualifier ensure safety in display()?

### Modern C++ Features

Raw pointers are error-prone. Smart pointers, introduced in C++11, simplify memory management.

```c++
#include <iostream>
#include <memory>

class MyClass {
public:
    MyClass() { std::cout << "Constructor called\n"; }
    ~MyClass() { std::cout << "Destructor called\n"; }
};

int main() {
    std::unique_ptr<MyClass> ptr = std::make_unique<MyClass>();
    // ptr goes out of scope, destructor is called automatically
    return 0;
}
```

Discussion Points:
 - What happens when the std::unique_ptr goes out of scope?
 - Compare std::shared_ptr and std::unique_ptr.


Example of lambda function usage:

```c++
#include <iostream>
#include <vector>
#include <algorithm>

int main() {
    std::vector<int> nums = {5, 2, 8, 3, 1};

    std::sort(nums.begin(), nums.end(), [](int a, int b) { return a < b; });

    for (int num : nums) {
        std::cout << num << ' ';
    }
    return 0;
}
```

Discussion Points:
 - How does the lambda function work in std::sort?
 - When should you use lambdas over named functions?

### Coding Challenge

Task: Create a simple program to manage student records, including adding and displaying students.
 - Use a Student class with properties for name, age, and grades.
 - Store students in a std::vector.
 - Implement a menu-driven program for user interaction.

```c++
#include <iostream>
#include <vector>
#include <string>

class Student {
private:
    std::string name;
    int age;
    std::vector<int> grades;

public:
    Student(const std::string& name, int age) : name(name), age(age) {}

    void addGrade(int grade) {
        grades.push_back(grade);
    }

    void display() const {
        std::cout << "Name: " << name << ", Age: " << age << ", Grades: ";
        for (int grade : grades) {
            std::cout << grade << ' ';
        }
        std::cout << '\n';
    }
};

int main() {
    std::vector<Student> students;

    // Add menu-driven functionality here
    return 0;
}
```