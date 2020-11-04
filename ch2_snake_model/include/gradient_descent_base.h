#pragma once

class GradientDescentBase {
   public:
    void run(int max_iteration);

   private:
    virtual void initialize() = 0;
    virtual void update() = 0;
    virtual bool is_terminate(int current_iter, int max_iteration);
};