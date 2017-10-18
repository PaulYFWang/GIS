#include "../inc/ScreenManager.h"

ScreenProperties::ScreenProperties(){
}

ScreenProperties::ScreenProperties(double _average_latitude, 
        double _average_longitude, double _maximum_latitude, 
        double _minimum_latitude, double _maximum_longitude, 
        double _minimum_longitude, double _max_x, double _min_x, 
        double _max_y, double _min_y, double _x_diff, double _y_diff){
    
    average_latitude = _average_latitude;
    average_longitude = _average_longitude;
    maximum_latitude = _maximum_latitude;
    minimum_latitude = _minimum_latitude;
    maximum_longitude = _maximum_longitude;
    minimum_longitude = _minimum_longitude;
    
    max_x = _max_x;
    min_x = _min_x;
    max_y = _max_y;
    min_y = _min_y;
    
    x_diff = _x_diff;
    y_diff = _y_diff;
}

ScreenManager* ScreenManager::instance;

ScreenManager::ScreenManager() {
}

ScreenManager::~ScreenManager() {
}

ScreenManager* ScreenManager::get_instance(){
    
    if(instance == NULL){
        //singleton object yet to be initialized
        instance = new ScreenManager;
    }
    //returns pointer to singleton
    return instance;
}

void ScreenManager::destroy_instance(){
    
    if(instance == NULL)
        return;
    else{
        delete instance;
        instance = NULL;
    }
}

const ScreenProperties& ScreenManager::get_global_properties(){
    return global_properties;
}

void ScreenManager::set_global_properties(const ScreenProperties& new_properties){
    global_properties = new_properties;
}

const t_bound_box& ScreenManager::get_screen_state_prev(){
    return screen_state_prev;
}
    
void ScreenManager::set_screen_state_prev(const t_bound_box& new_screen_state){
    screen_state_prev = new_screen_state;
}

bool ScreenManager::check_zoom_oob(){
    bool zoomed_oob = false;
    
    t_bound_box dim = get_visible_world();
    
    double x = dim.get_width();
    double y = dim.get_height();
    
    if((x > 8*global_properties.x_diff/9 || y > 8*global_properties.y_diff/9)||(x < 115.0 || y < 75.0)){
        set_visible_world(screen_state_prev);
        zoomed_oob = true;
    }
    
    if(zoomed_oob == false){
        screen_state_prev = dim;
    }
    
    return zoomed_oob;
}

bool ScreenManager::check_pan_oob(){
    bool altered = false;
    
    t_bound_box dim = get_visible_world();
    
    double x = dim.get_width();
    double y = dim.get_height();
            
    double x_left = dim.left();
    double x_right = dim.right();
    double y_top = dim.top();
    double y_bottom = dim.bottom();
    
    if(dim.bottom() < global_properties.min_y){
        y_bottom = global_properties.min_y;
        y_top = global_properties.min_y+y;
        altered = true;
    }
    if(dim.top() > global_properties.max_y){
        y_top = global_properties.max_y;
        y_bottom = global_properties.max_y-y;
        altered = true;
    }
    if(dim.left() < global_properties.min_x){
        x_left = global_properties.min_x;
        x_right = global_properties.min_x+x;
        altered = true;
    }
    if(dim.right() > global_properties.max_x){
        x_right = global_properties.max_x;
        x_left = global_properties.max_x-x;
        altered = true;
    }
    
    if(altered == true){
        t_bound_box altered_bound(x_left,y_bottom,x_right,y_top);
        set_visible_world(altered_bound);
        screen_state_prev = altered_bound;
    }
    
    return altered;
}

void ScreenManager::check_oob(){
    if(check_zoom_oob()){
        ;
        //cout << "zoomed out of bounds" << endl;
    }
    if(check_pan_oob()){
        ;
        //cout << "panned out of bounds" << endl;
    }
}
