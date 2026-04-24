#include <memory>
#include "internal/display/service/displayrepo.h"

// Abstraction - display module, service
class display {
private:
    std::shared_ptr<displayrepo> displayrepo; // DI
public:
    display();
    ~display();
};


// Constructor
display::display() {
}


// Destructor
display::~display()
{
}
