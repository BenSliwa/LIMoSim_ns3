#include "deliveryutils.h"

#include "LIMoSim/settings/filehandler.h"

namespace LIMoSim {

using namespace utils::typedefs;

namespace delivery {
namespace utils {

StringVector loadDeliveryListFromFile(std::string _filename) {
    StringVector deliveryList;
    auto content = FileHandler::read(_filename);
    for (auto line : content) {
        // discard lines starting with "//"
        if (!(line.rfind("//", 0) == 0)) {
            deliveryList.push_back(line);
        }
    }
    // remove blank last line
    deliveryList.erase(deliveryList.end());
    return deliveryList;
}

} // namespace utils
} // namespace delivery
} // namespace LIMoSim
