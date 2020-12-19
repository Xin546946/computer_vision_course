#pragma once

class DataBase {
   public:
    virtual void init_mass_center() = 0;
    virtual void update_mass_center(const std::unique_ptr<WindowBase>& win_ptr) = 0;
    virtual void visualize() const;
    virtual bool is_convergent();

   private:
};