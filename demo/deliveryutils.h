#ifndef DELIVERYUTILS_H
#define DELIVERYUTILS_H

#include "LIMoSim/utils/typedefs.h"

namespace LIMoSim {

using namespace utils::typedefs;

namespace delivery {
namespace utils {

/**
 * @brief loadDeliveryListFromFile
 * Extracts delivery list from file
 * while ignoring comments
 * @param _filename
 * @return delivery list
 */
StringVector loadDeliveryListFromFile(std::string _filename);

} // namespace utils
} // namespace delivery
} // namespace LIMoSim

#endif // DELIVERYUTILS_H
