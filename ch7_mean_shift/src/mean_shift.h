#pragma once
#include <memory>

class DataBase;
class WindowBase;
class MeanShift {
   public:
    MeanShift(std::unique_ptr<DataBase> db_ptr, std::unique_ptr<WindowBase> win_ptr);
    void run();

   private:
    std::unique_ptr<DataBase> db_ptr_;
    std::unique_ptr<WindowBase> win_ptr_;
};