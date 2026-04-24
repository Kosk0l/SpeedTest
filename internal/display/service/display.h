#include <memory>
#include "displayrepo.h"

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
