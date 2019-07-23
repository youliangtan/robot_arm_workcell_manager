#include <iostream>
#include <memory>
#include <thread>

class FiduciallMarkerHandler{
    private:
        std::string marker_type;

    public:
        FiduciallMarkerHandler(const std::string& _group_name);
        
        ~FiduciallMarkerHandler();

        // ------ Execution -----

};
