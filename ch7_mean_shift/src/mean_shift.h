#pragma once

#include <memory>
class DataBase;

class MeanShift {
   public:
    MeanShift(std::unique_ptr<DataBase>& db_ptr);
    void run();

   private:
    std::unique_ptr<DataBase> db_ptr_;
};