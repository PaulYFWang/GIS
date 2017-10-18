/*
 * ScreenManager is a singleton class that holds all global screen data
 * and screen manipulation functions.
 */

#ifndef SCREENMANAGER_H
#define SCREENMANAGER_H

#include "graphics.h"

using namespace std;

struct ScreenProperties
{
    double average_latitude;
    double average_longitude;
    double maximum_latitude;
    double minimum_latitude;
    double maximum_longitude;
    double minimum_longitude;
    
    double max_x;
    double min_x;
    double max_y;
    double min_y;
    
    double x_diff;
    double y_diff;
    
    ScreenProperties();
    
    ScreenProperties(
    double _average_latitude,
    double _average_longitude,
    double _maximum_latitude,
    double _minimum_latitude,
    double _maximum_longitude,
    double _minimum_longitude,
    
    double _max_x,
    double _min_x,
    double _max_y,
    double _min_y,
    
    double _x_diff,
    double _y_diff);
};

class ScreenManager {

private:
    
    //pointer to singleton class, is NULL when not instantiated
    static ScreenManager *instance;
    
    ScreenProperties global_properties;
    
    t_bound_box screen_state_prev;

    ScreenManager();
    
    virtual ~ScreenManager();
    
public:
    
    static ScreenManager* get_instance();

    static void destroy_instance();
    
    const ScreenProperties& get_global_properties();
    
    void set_global_properties(const ScreenProperties& new_properties);
    
    const t_bound_box& get_screen_state_prev();
    
    void set_screen_state_prev(const t_bound_box& new_screen_state);
    
    bool check_pan_oob();
    
    bool check_zoom_oob();
    
    void check_oob();
    
};

#endif /* SCREENMANAGER_H */

