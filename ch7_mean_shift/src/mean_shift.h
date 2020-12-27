#pragma once

#include <memory>
class DataBase;

class MeanShift {
   public:
    MeanShift() = default;
    MeanShift(DataBase* db_ptr);
    void run(int max_iteration);

   private:
    DataBase* db_ptr_;
};