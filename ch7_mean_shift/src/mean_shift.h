#pragma once

#include <memory>
class DataBase;

class MeanShift {
   public:
    MeanShift(std::unique_ptr<DataBase>& db_ptr);
    void run(int max_iteration);

   private:
    std::unique_ptr<DataBase> db_ptr_;
};