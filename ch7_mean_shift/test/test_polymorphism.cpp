#include <iostream>

class Base {
   public:
    virtual void f() {
        std::cout << "base " << '\n';
    }
};

class DriveA : Base {
    void f() override {
        std::cout << "A " << '\n';
    }
};

class DriveB : Base {
    void f() override {
        std::cout << "B " << '\n';
    }
};

void print_info(Base* b_ptr) {
    b_ptr->f();
}

int main(int argc, char** argv) {
    Base* b = new Base();
    DriveA* da = new DriveA();
    DriveB* db = new DriveB();

    print_info(b);
    print_info((Base*)da);
    print_info((Base*)db);

    return 0;
}
