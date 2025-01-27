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

`std::unique_ptr`: 
 - Exclusive ownership: only one std::unique_ptr can point to a resource at a time.

```c++
#include <iostream>
#include <memory>

class MyClass {
public:
    MyClass(std::string name) : name_{name} { std::cout << "Constructor called " + name_ << std::endl; }
    ~MyClass() { std::cout << "Destructor called " + name_ << std::endl; }
private:
    std::string name_;
};

int main() {
    {   // Create a scope to demonstrate smart pointer behavior
        std::unique_ptr<MyClass> u_ptr = std::make_unique<MyClass>("unique");
        MyClass* raw_ptr = new MyClass("raw");
    }   // end of scope; u_ptr goes out of scope, destructor is called automatically
    // raw_ptr is not deleted, causing a potential memory leak
    
    return 0;
}
```

`std::shared_ptr`: 
 - Shared ownership: multiple std::shared_ptr can point to the same resource.
 - Reference counting: the resource is deleted when the last std::shared_ptr goes out of scope.

```c++
#include <iostream>
#include <memory>

class MyClass {
public:
    MyClass() { std::cout << "MyClass constructor\n"; }
    ~MyClass() { std::cout << "MyClass destructor\n"; }
};

int main() {
    std::shared_ptr<MyClass> sp1 = std::make_shared<MyClass>();
    std::cout << "Use count: " << sp1.use_count() << std::endl;

    {
        std::shared_ptr<MyClass> sp2 = sp1; 
        std::cout << "Use count: " << sp1.use_count() << std::endl;
    }

    std::cout << "Use count: " << sp1.use_count() << std::endl;
    return 0;
}
```

`std::weak_ptr`
 - Weak reference: does not affect the reference count of the shared resource.
 - Desn't increase the reference count.
 - Used to prevent circular references in shared ownership.


```c++
#include <iostream>
#include <memory>

class NodeB; // forward declaration

class NodeA {
public:
    std::shared_ptr<NodeB> strong_ptr; // Strong reference to NodeB
    std::weak_ptr<NodeB> weak_ptr; // Strong reference to NodeA
    NodeA() { std::cout << "NodeStrong constructor\n"; }
    ~NodeA() {  std::cout << "NodeStrong destructor\n";  }
};

class NodeB {
public:
    std::shared_ptr<NodeA> strong_ptr; // Strong reference to NodeA
    std::weak_ptr<NodeA> weak_ptr; // Strong reference to NodeA
    NodeB() { std::cout << "NodeWeak constructor" << std::endl; }
    ~NodeB() { std::cout << "NodeWeak destructor\n"; }
};

int main() {

    { // create scope
        std::cout << "Entering first scope..." << std::endl;
        // Create NodeA and NodeB, each referencing the other.
        auto a = std::make_shared<NodeA>();
        auto b = std::make_shared<NodeB>();
        a->strong_ptr = b; // NodeA has a strong reference to b
        b->strong_ptr = a; // NodeB has a strong reference to a
        std::cout << "Exiting first scope..." << std::endl;
    } // end scope

    // Here, a and b go out of scope, but each Node holds a strong pointer to the other.
    // Their reference counts never reach zero, so destructors are NOT called.
    // This leads to a memory leak because NodeA and NodeB remain alive, referencing each other.

    { // create new scope
        std::cout << "Entering second scope..." << std::endl;
        auto a = std::make_shared<NodeA>();
        auto b = std::make_shared<NodeB>();
        a->strong_ptr = b; // NodeA has a strong reference to b
        b->weak_ptr = a; // NodeB has a weak reference to a
        std::cout << "Exiting second scope..." << std::endl;
    }
    
    return 0;
}
```


Discussion Points:
 - What happens when the std::unique_ptr goes out of scope?
 - Compare std::shared_ptr and std::unique_ptr.
 - When should you use std::weak_ptr?
 - Should we use raw pointers in modern C++? ... No we should not!


### Functions as Object 

### Lambda Functions

Lambda functions (also called lambda expressions) in C++ are unnamed (anonymous) functions that you can define inline. They were introduced in C++11 to make it easier to create small, concise functions, especially for use with the Standard Template Library (STL) algorithms or as callbacks. Unlike regular functions, they can capture variables from their surrounding scope. This is incredibly useful for passing context to a function on-the-fly.

Syntax:
```c++
[ capture_list ] ( parameter_list ) -> return_type {
    // function body
}
```

 - capture_list: Specifies which variables from the enclosing scope are available inside the lambda and how they are captured (by value, by reference, etc.).
 - parameter_list: The parameters the lambda accepts (similar to a function’s parameter list).
 - return_type: Often omitted because it can be deduced by the compiler, but can be specified explicitly using -> return_type.
 - function body: The code that executes when the lambda is called.

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


```c++
#include <iostream>
#include <vector>
#include <algorithm>

int main() {
    std::vector<int> values = {1, 2, 3, 4, 5};
    int offset = 10;

    auto printValue = [](int val) {
        std::cout << val << " ";
    };

    // Capture everything by value (copy
    std::for_each(values.begin(), values.end(), [=](int val)  {
        // modifies copy of 'val', not `val` itself
        val += offset;
        // offset += 1; // error: 'offset' cannot be modified. Can use [=]() mutable {...} to modify
    });
    std::for_each(values.begin(), values.end(), printValue);

    // Capture everything by reference
    std::for_each(values.begin(), values.end(), [&](int &val) {
        val += offset; // modifies 'val' directly in the vector via reference
         offset += 1;
    });
    std::for_each(values.begin(), values.end(), printValue);
    
    std::cout << std::endl;
    return 0;
}
```

### `std::function`

A flexible, type-erased wrapper that can store function pointers, lambdas, or functor objects. It is part of the C++ Standard Library and is useful for creating callbacks or function objects that can be passed around like variables.

```c++
#include <iostream>
#include <functional>

int sum(int a, int b) {
    return a + b;
}

int main() {
    std::function<int(int, int)> func1 = sum;
    std::function<int(int, int)> func2 = [](int a, int b) { return a * b; };

    std::cout << "sum(3, 4): " << func1(3, 4) << std::endl;
    std::cout << "multiply(3, 4): " << func2(3, 4) << std::endl;

    return 0;
}
```

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