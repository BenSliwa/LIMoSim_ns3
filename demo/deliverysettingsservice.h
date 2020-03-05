#ifndef DELIVERYSETTINGSSERVICE_H
#define DELIVERYSETTINGSSERVICE_H

#include <map>
#include <variant>
#include <string>

namespace LIMoSim {
namespace delivery {


typedef std::variant<bool, int, float, std::string> DeliverySettingVariant;
class DeliverySettingsService
{
public:
    static DeliverySettingsService* getInstance();

    // Delivery settings
    /**
     * @brief The DeliverySettingKey enum
     */
    enum DeliverySettingKey {
        // The way the drones and the truck are to interact
        // for launching and recovery.
        // Values are instances of  LIMoSim::DroneInteractionMode
        DRONE_INTERACTION_MODE,
        // Time the truck needs to unload
        // at delivery target in secs
        TRUCK_UNLOADING_DELAY,
        // Time the UAV needs to dock
        // on delivery truck in secs
        UAV_DOCKING_DELAY,
        // Time the UAV needs to launch
        // from  delivery truck in secs
        UAV_LAUNCH_DELAY,
        // Time the UAV needs to unload
        // at delivery target in secs
        UAV_UNLOADING_TIME
    };

    const DeliverySettingVariant* getDeliverySetting(DeliverySettingKey _settingKey);
    void setDeliverySetting(DeliverySettingKey _settingKey, DeliverySettingVariant _setting);
private:
    std::map<DeliverySettingKey, DeliverySettingVariant> m_settings;
    DeliverySettingsService();
};

} // namespace delivery
} // namespace LIMoSim

#endif // DELIVERYSETTINGSSERVICE_H
