#pragma once

#include <memory>
class DataBase;

class MeanShift {
   public:
    MeanShift(std::shared_ptr<DataBase>& db_ptr);
    void run(int max_iteration);

   private:
    std::shared_ptr<DataBase> db_ptr_;
};