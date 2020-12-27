#pragma once

#include <memory>
class DataBase;

class MeanShift {
   public:
    MeanShift() = default;
    MeanShift(const std::shared_ptr<DataBase>& db_ptr);
    void run(int max_iteration);

   private:
    bool compute_energy_ = false;
    std::shared_ptr<DataBase> db_ptr_;
};